#pragma once

#include <asio.hpp>

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iterator>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#if defined(__linux__) || defined(__APPLE__)
#include <termios.h>
#endif

#include "r2_serial/transfer_protocol.hpp"

class SerialConnector {
public:
  using packet_t = gdut::packet_manager<gdut::crc16_algorithm>::packet_t;

  SerialConnector(const std::string &port_name, asio::io_context &io_context)
      : state_(std::make_shared<State>(port_name, io_context)) {
    state_->startAsyncRead();
  }

  template <typename WriteHandler>
  void asyncSend(const packet_t &packet, WriteHandler &&handler) {
    std::vector<std::uint8_t> bytes(packet.begin(), packet.end());
    state_->enqueueWrite(std::move(bytes),
                         std::forward<WriteHandler>(handler));
  }

  template <typename ReadHandler> void asyncReceive(ReadHandler &&handler) {
    state_->addReceiveHandler(std::forward<ReadHandler>(handler));
  }

  void clearPendingWrites() { state_->clearPendingWrites(); }

  void setMinWriteInterval(std::chrono::milliseconds interval) {
    state_->setMinWriteInterval(interval);
  }

  void setRawReceiveHandler(
      std::function<void(const std::uint8_t *, std::size_t)> handler) {
    state_->setRawReceiveHandler(std::move(handler));
  }

  ~SerialConnector() noexcept {
    if (state_) {
      state_->close();
    }
  }

private:
  struct HandlerWrapperBase {
    virtual ~HandlerWrapperBase() = default;
    virtual void execute(std::error_code ec, packet_t packet) = 0;
  };

  template <typename Handler> struct HandlerWrapper : HandlerWrapperBase {
    explicit HandlerWrapper(Handler handler) : handler_(std::move(handler)) {}

    void execute(std::error_code ec, packet_t packet) override {
      handler_(ec, std::move(packet));
    }

    Handler handler_;
  };

  struct PendingWrite {
    std::vector<std::uint8_t> bytes;
    std::function<void(std::error_code, std::size_t)> handler;
  };

  struct State : std::enable_shared_from_this<State> {
    State(const std::string &port_name, asio::io_context &io_context)
        : serial_port(io_context, port_name), strand(io_context.get_executor()),
          write_timer(io_context) {
      serial_port.set_option(asio::serial_port_base::baud_rate(115200));
      serial_port.set_option(asio::serial_port_base::character_size(8));
      serial_port.set_option(asio::serial_port_base::parity(
          asio::serial_port_base::parity::none));
      serial_port.set_option(asio::serial_port_base::stop_bits(
          asio::serial_port_base::stop_bits::one));
      serial_port.set_option(asio::serial_port_base::flow_control(
          asio::serial_port_base::flow_control::none));
      configureRawMode();

      packet_manager.set_receive_function(
          [this](packet_t packet) { dispatchPacket(std::move(packet)); });
    }

    void configureRawMode() {
#if defined(__linux__) || defined(__APPLE__)
      termios options{};
      if (tcgetattr(serial_port.native_handle(), &options) != 0) {
        throw std::runtime_error(std::string("tcgetattr failed: ") +
                                 std::strerror(errno));
      }
      cfmakeraw(&options);
      options.c_cflag |= CLOCAL | CREAD;
      if (tcsetattr(serial_port.native_handle(), TCSANOW, &options) != 0) {
        throw std::runtime_error(std::string("tcsetattr failed: ") +
                                 std::strerror(errno));
      }
#endif
    }

    void startAsyncRead() {
      if (!is_running.load()) {
        return;
      }
      auto self = shared_from_this();
      serial_port.async_read_some(
          asio::buffer(read_buffer),
          asio::bind_executor(
              strand, [self](std::error_code ec, std::size_t bytes_read) {
                if (!self->is_running.load()) {
                  return;
                }
                if (!ec) {
                  if (self->raw_receive_handler) {
                    self->raw_receive_handler(self->read_buffer.data(),
                                              bytes_read);
                  }
                  self->packet_manager.receive(
                      self->read_buffer.data(),
                      self->read_buffer.data() + bytes_read);
                  self->startAsyncRead();
                } else if (ec != asio::error::operation_aborted) {
                  self->startAsyncRead();
                }
              }));
    }

    template <typename Handler>
    void enqueueWrite(std::vector<std::uint8_t> bytes, Handler &&handler) {
      auto self = shared_from_this();
      asio::post(strand,
                 [self, bytes = std::move(bytes),
                  handler = std::forward<Handler>(handler)]() mutable {
                   const bool idle = self->pending_writes.empty();
                   self->pending_writes.push(
                       PendingWrite{std::move(bytes), std::move(handler)});
                   if (idle) {
                     self->startNextWrite();
                   }
                 });
    }

    void clearPendingWrites() {
      auto self = shared_from_this();
      asio::post(strand, [self]() {
        std::error_code ignored;
        self->write_timer.cancel(ignored);
        while (!self->pending_writes.empty()) {
          self->pending_writes.pop();
        }
      });
    }

    void setMinWriteInterval(std::chrono::milliseconds interval) {
      auto self = shared_from_this();
      asio::post(strand, [self, interval]() {
        self->min_write_interval = interval.count() > 0 ? interval
                                                        : std::chrono::milliseconds(0);
      });
    }

    void startNextWrite() {
      if (!is_running.load() || pending_writes.empty()) {
        return;
      }

      // 统一串口限速：不管 ROS 哪个节点发来的包，最终写串口前都从这里排队。
      if (min_write_interval.count() > 0 && have_last_write_time) {
        const auto now = std::chrono::steady_clock::now();
        const auto earliest = last_write_time + min_write_interval;
        if (now < earliest) {
          auto self = shared_from_this();
          write_timer.expires_after(earliest - now);
          write_timer.async_wait(asio::bind_executor(
              strand, [self](std::error_code ec) {
                if (!ec && self->is_running.load()) {
                  self->startNextWrite();
                }
              }));
          return;
        }
      }

      last_write_time = std::chrono::steady_clock::now();
      have_last_write_time = true;

      auto self = shared_from_this();
      asio::async_write(
          serial_port, asio::buffer(pending_writes.front().bytes),
          asio::bind_executor(
              strand, [self](std::error_code ec, std::size_t bytes_written) {
                auto handler =
                    std::move(self->pending_writes.front().handler);
                self->pending_writes.pop();
                handler(ec, bytes_written);
                if (!ec) {
                  self->startNextWrite();
                }
              }));
    }

    template <typename Handler> void addReceiveHandler(Handler &&handler) {
      std::unique_ptr<HandlerWrapperBase> wrapper;
      packet_t packet;
      {
        std::lock_guard<std::mutex> lock(mutex);
        if (received_packets.empty()) {
          request_handlers.emplace(
              std::make_unique<HandlerWrapper<std::decay_t<Handler>>>(
                  std::forward<Handler>(handler)));
          return;
        }
        packet = std::move(received_packets.front());
        received_packets.pop();
        wrapper = std::make_unique<HandlerWrapper<std::decay_t<Handler>>>(
            std::forward<Handler>(handler));
      }
      auto self = shared_from_this();
      asio::dispatch(strand,
                     [self, wrapper = std::move(wrapper),
                      packet = std::move(packet)]() mutable {
                       wrapper->execute(std::error_code(), std::move(packet));
                     });
    }

    void setRawReceiveHandler(
        std::function<void(const std::uint8_t *, std::size_t)> handler) {
      auto self = shared_from_this();
      asio::post(strand, [self, handler = std::move(handler)]() mutable {
        self->raw_receive_handler = std::move(handler);
      });
    }

    void dispatchPacket(packet_t packet) {
      std::unique_ptr<HandlerWrapperBase> handler;
      {
        std::lock_guard<std::mutex> lock(mutex);
        if (request_handlers.empty()) {
          received_packets.push(std::move(packet));
          return;
        }
        handler = std::move(request_handlers.front());
        request_handlers.pop();
      }
      handler->execute(std::error_code(), std::move(packet));
    }

    void close() noexcept {
      if (!is_running.exchange(false)) {
        return;
      }
      std::error_code ignored;
      write_timer.cancel(ignored);
      serial_port.cancel(ignored);
      serial_port.close(ignored);
    }

    gdut::packet_manager<gdut::crc16_algorithm> packet_manager;
    asio::serial_port serial_port;
    asio::strand<asio::any_io_executor> strand;
    asio::steady_timer write_timer;
    std::chrono::milliseconds min_write_interval{std::chrono::milliseconds(10)};
    std::chrono::steady_clock::time_point last_write_time{};
    bool have_last_write_time{false};
    std::array<std::uint8_t, 1024> read_buffer{};
    std::atomic<bool> is_running{true};
    std::mutex mutex;
    std::queue<std::unique_ptr<HandlerWrapperBase>> request_handlers;
    std::queue<packet_t> received_packets;
    std::queue<PendingWrite> pending_writes;
    std::function<void(const std::uint8_t *, std::size_t)> raw_receive_handler;
  };

  std::shared_ptr<State> state_;
};
