#pragma once

#include <asio.hpp>
#include <string>
#include <functional>
#include <iterator>
#include <memory>
#include <atomic>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include "transfer_protocol.hpp"

class SerialConnector {
public:
  using packet_t = gdut::packet_manager<gdut::crc16_algorithm>::packet_t;

  SerialConnector(const std::string& portName, asio::io_context& io_context)
      : serialPort_(io_context, portName),
      strand_(io_context.get_executor()) {
    // 启动串口连接
    serialPort_.set_option(asio::serial_port_base::baud_rate(115200));
    serialPort_.set_option(asio::serial_port_base::character_size(8));
    serialPort_.set_option(
        asio::serial_port_base::parity(asio::serial_port_base::parity::none));
    serialPort_.set_option(asio::serial_port_base::stop_bits(
        asio::serial_port_base::stop_bits::one));
    serialPort_.set_option(asio::serial_port_base::flow_control(
        asio::serial_port_base::flow_control::none));

    if (!serialPort_.is_open()) {
      throw std::runtime_error("Failed to open serial port");
    }

    packetManager_.set_receive_function([this](packet_t packet) {
      std::unique_ptr<HandlerWrapperBase> handler;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!requestHandlersQueue_.empty()) {
          handler = std::move(requestHandlersQueue_.front());
          requestHandlersQueue_.pop();
        } else {
          receivedPacketsQueue_.push(std::move(packet));
          return;
        }
      }
      if (handler) {
        handler->execute(std::error_code(), packet);
      }
    });

    ioThread_ = std::thread([this]() {
      while (isRunning_) {
        std::vector<uint8_t> readBuffer(1024);
        asio::error_code ec;
        size_t bytesRead = serialPort_.read_some(asio::buffer(readBuffer), ec);
        if (!isRunning_) {
          return; // 连接被关闭，退出线程
        }
        if (ec) {
          if (ec == asio::error::operation_aborted) {
            return; // 连接被关闭，退出线程
          }
          // 处理其他错误
          continue;
        }
        packetManager_.receive(readBuffer.data(), readBuffer.data() + bytesRead);
      }
    });
  }

  // packet需要求调用者保证其生命周期至少持续到异步操作完成
  template<asio::completion_token_for<void(std::error_code, std::size_t)> WriteHandler>
  auto asyncSend(const packet_t& packet, WriteHandler&& handler) {
    return asio::async_write(serialPort_, asio::buffer(packet.begin(),
        std::distance(packet.begin(), packet.end())),
      asio::bind_executor(strand_, std::forward<WriteHandler>(handler)));
  }

  template<asio::completion_token_for<void(std::error_code, packet_t)> ReadHandler>
  auto asyncReceive(ReadHandler&& handler) {
    return asio::async_initiate<ReadHandler, void(std::error_code, packet_t)>(
        [this](auto&& handler) {
          {
            std::unique_lock<std::mutex> lock(mutex_);
            if (!receivedPacketsQueue_.empty()) {
              packet_t packet = std::move(receivedPacketsQueue_.front());
              receivedPacketsQueue_.pop();
              lock.unlock();
              asio::dispatch(strand_, [handler = std::move(handler),
                  packet = std::move(packet)]() mutable {
                handler(std::error_code(), std::move(packet));
              });
              return;
            } else {
              requestHandlersQueue_.emplace(
                std::make_unique<HandlerWrapper<std::decay_t<decltype(handler)>>>(
                  std::forward<decltype(handler)>(handler)));
            }
          }
        }, handler);
  }

  ~SerialConnector() noexcept {
    isRunning_ = false;
    serialPort_.cancel();
    serialPort_.close();
    if (ioThread_.joinable()) {
      ioThread_.join();
    }
  }

protected:
  struct HandlerWrapperBase {
    virtual ~HandlerWrapperBase() = default;
    virtual void execute(std::error_code ec, packet_t packet) = 0;
  };

  template<typename Handler>
  struct HandlerWrapper : HandlerWrapperBase {
    HandlerWrapper(Handler h) : handler_(std::move(h)) {}
    void execute(std::error_code ec, packet_t packet) override {
      handler_(ec, std::move(packet));
    }
    Handler handler_;
  };

private:
  gdut::packet_manager<gdut::crc16_algorithm> packetManager_;
  asio::serial_port serialPort_;
  asio::strand<asio::any_io_executor> strand_;
  std::atomic<bool> isRunning_{true};
  std::thread ioThread_;
  mutable std::mutex mutex_;
  std::queue<std::unique_ptr<HandlerWrapperBase>> requestHandlersQueue_;
  std::queue<packet_t> receivedPacketsQueue_;
};
