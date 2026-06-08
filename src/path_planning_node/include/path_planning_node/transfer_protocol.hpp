#ifndef MODULES_TRANSFER_PROTOCOL_HPP
#define MODULES_TRANSFER_PROTOCOL_HPP

#include "verification_algorithm.hpp"

#include <algorithm>
#include <cstdint>
#include <iterator>
#include <limits>
#include <memory>
#include <memory_resource>
#include <type_traits>
#include <vector>
#include <iterator>
#include <functional>

namespace gdut {

struct build_packet_t {};
inline constexpr build_packet_t build_packet;

struct from_whole_packet_t {};
inline constexpr from_whole_packet_t from_whole_packet;

template <typename VerifyAlgorithm> class data_packet {
  static_assert(
      std::is_base_of_v<verify_algorithm<VerifyAlgorithm>, VerifyAlgorithm>,
      "VerifyAlgorithm must be derived class of "
      "verify_algorithm<VerifyAlgorithm>");

public:
  using verify_algorithm_t = VerifyAlgorithm;
  static constexpr std::uint16_t header = 0xAA << 8 | 0x55;
  static constexpr std::uint16_t tail = 0x55 << 8 | 0xAA;
  static constexpr std::size_t header_size =
      sizeof(uint16_t) + sizeof(uint16_t) * 3; // header + size + code + crc
  static constexpr std::size_t tail_size = sizeof(uint16_t);

  static std::pmr::memory_resource *default_memory_resource() noexcept {
    return std::pmr::get_default_resource();
  }

  data_packet(std::pmr::memory_resource *mr = default_memory_resource())
      : m_data(mr) {}

  template <std::input_iterator It>
  data_packet(uint16_t code, It begin, It end, build_packet_t,
              std::pmr::memory_resource *mr = default_memory_resource())
      : m_data(mr) {
    static_assert(sizeof(std::iter_value_t<It>) == 1,
                  "The data size of the iterator must be 1");
    if (std::distance(begin, end) >
        static_cast<decltype(std::distance(begin, end))>(
            std::numeric_limits<uint16_t>::max() - sizeof(uint16_t) * 3)) {
      return; // Body size is too large to fit in a packet
    }
    // head (2 bytes) + size (2 bytes) + code (2 bytes) + crc (2 bytes) + body +
    // tail (2 bytes)
    const uint16_t total_size =
        static_cast<uint16_t>(header_size) +
        static_cast<uint16_t>(std::distance(begin, end)) +
        static_cast<uint16_t>(tail_size);
    m_data.resize(total_size);
    m_data[0] = (header >> 8) & 0xFF;
    m_data[1] = header & 0xFF;
    m_data[2] = (total_size >> 8) & 0xFF;
    m_data[3] = total_size & 0xFF;
    m_data[4] = (code >> 8) & 0xFF;
    m_data[5] = code & 0xFF;
    std::copy(begin, end, m_data.begin() + header_size);
    m_data[total_size - 2] = (tail >> 8) & 0xFF;
    m_data[total_size - 1] = tail & 0xFF;
    update_verification();
  }

  template <std::random_access_iterator It>
  data_packet(It begin, It end, from_whole_packet_t,
              std::pmr::memory_resource *mr = default_memory_resource())
      : m_data(mr) {
    if (std::distance(begin, end) < 2) {
      return; // Not enough data for packet header
    }
    if (*begin != ((header >> 8) & 0xFF) || *(begin + 1) != (header & 0xFF)) {
      return; // Invalid packet header
    }
    if (static_cast<std::size_t>(std::distance(begin, end)) <
        header_size + tail_size) {
      return; // Not enough data for header and tail, wait for more data
    }
    uint16_t size = *(begin + 2) << 8 | *(begin + 3);
    if (size < header_size + tail_size) {
      return; // Invalid packet size
    }
    if (static_cast<std::size_t>(std::distance(begin, end)) < size) {
      return;
    }
    if (*(begin + size - 2) != ((tail >> 8) & 0xFF) ||
        *(begin + size - 1) != (tail & 0xFF)) {
      return; // Invalid packet tail
    }
    m_data.resize(size);
    std::copy(begin, begin + size, m_data.begin());
    if (!verify_verification()) {
      m_data.clear();
      return;
    }
  }

  ~data_packet() = default;

  data_packet(const data_packet &packet,
              std::pmr::memory_resource *mr = default_memory_resource())
      : m_data(packet.m_data, mr) {}
  data_packet(data_packet &&packet) noexcept
      : m_data(std::move(packet.m_data)) {}

  data_packet &operator=(const data_packet &packet) {
    if (this != std::addressof(packet)) {
      m_data = packet.m_data;
    }
    return *this;
  }

  data_packet &operator=(data_packet &&packet) noexcept {
    if (this != std::addressof(packet)) {
      m_data = std::move(packet.m_data);
    }
    return *this;
  }

  [[nodiscard]] uint16_t size() const noexcept {
    if (m_data.size() < 4) {
      return 0;
    }
    return m_data[2] << 8 | m_data[3];
  }

  [[nodiscard]] uint16_t code() const noexcept {
    if (m_data.size() < 6) {
      return 0;
    }
    return m_data[4] << 8 | m_data[5];
  }

  [[nodiscard]] uint16_t crc() const noexcept {
    if (m_data.size() < 8) {
      return 0;
    }
    return m_data[6] << 8 | m_data[7];
  }

  [[nodiscard]] const uint8_t *data() const noexcept { return m_data.data(); }

  [[nodiscard]] const uint8_t *begin() const noexcept { return m_data.data(); }

  [[nodiscard]] const uint8_t *end() const noexcept {
    return m_data.data() + m_data.size();
  }

  [[nodiscard]] uint16_t body_size() const noexcept {
    const uint16_t packet_size = size();
    if (packet_size < header_size + tail_size) {
      return 0;
    }
    return packet_size - header_size - tail_size;
  }

  [[nodiscard]] const uint8_t *body_data() const noexcept {
    if (m_data.size() < header_size) {
      return nullptr;
    }
    return m_data.data() + header_size;
  }

  [[nodiscard]] const uint8_t *body_begin() const noexcept {
    if (m_data.size() < header_size) {
      return nullptr;
    }
    return m_data.data() + header_size;
  }

  [[nodiscard]] const uint8_t *body_end() const noexcept {
    if (m_data.size() < header_size) {
      return nullptr;
    }
    return m_data.data() + size();
  }

  void update_verification() noexcept {
    if (m_data.size() < header_size + tail_size) {
      return;
    }
    verify_algorithm_t va;
    va.calculate(m_data.begin(), m_data.end(),
                 m_data.begin() + header_size - sizeof(uint16_t));
  }

  [[nodiscard]] bool verify_verification() const noexcept {
    if (m_data.size() < header_size + tail_size) {
      return false;
    }
    verify_algorithm_t va;
    return va.verify(m_data.begin(), m_data.end(),
                     m_data.begin() + header_size - sizeof(uint16_t));
  }

  explicit operator bool() const noexcept {
    return m_data.size() >= (header_size + tail_size);
  }

private:
  /*
  struct {
    uint8_t header;
    uint16_t size;
    uint16_t code;
    uint16_t verify;
    uint8_t  payload[];
  } header;
  */
  std::pmr::vector<std::uint8_t> m_data;
};

template <typename VerifyAlgorithm> class packet_manager {
public:
  using packet_t = data_packet<VerifyAlgorithm>;

  packet_manager() : m_receive_buffer(std::pmr::get_default_resource()) {}
  ~packet_manager() = default;

  void set_send_function(
      std::function<void(const std::uint8_t *, const std::uint8_t *)> func) {
    m_send_function = std::move(func);
  }

  void set_receive_function(std::function<void(packet_t)> func) {
    m_receive_function = std::move(func);
  }

  void send(const packet_t &packet) {
    if (m_send_function && packet) {
      m_send_function(packet.data(), packet.data() + packet.size());
    }
  }

  template <std::input_iterator It> void receive(It begin, It end) {
    m_receive_buffer.resize(m_receive_buffer.size() +
                            std::distance(begin, end));
    std::copy(begin, end, m_receive_buffer.end() - std::distance(begin, end));
    while (true) {
      std::pmr::vector<std::uint8_t>::iterator packet_start;
      while (true) {
        packet_start =
            std::find(m_receive_buffer.begin(), m_receive_buffer.end(),
                      (packet_t::header >> 8) & 0xFF);
        if (packet_start == m_receive_buffer.end()) {
          // No header start found. Keep one trailing 0xAA to handle split
          // header across receive chunks: [..., 0xAA] + [0x55, ...].
          if (!m_receive_buffer.empty() &&
              m_receive_buffer.back() == ((packet_t::header >> 8) & 0xFF)) {
            m_receive_buffer.erase(m_receive_buffer.begin(),
                                   m_receive_buffer.end() - 1);
          } else {
            m_receive_buffer.clear();
          }
          return; // No packet header found, wait for more data
        }
        if ((packet_start + 1 != m_receive_buffer.end() &&
             *(packet_start + 1) != (packet_t::header & 0xFF))) {
          m_receive_buffer.erase(m_receive_buffer.begin(), packet_start + 1);
        } else {
          break; // Found potential packet header
        }
      }
      if (std::distance(packet_start, m_receive_buffer.end()) <
          static_cast<std::ptrdiff_t>(packet_t::header_size)) {
        return; // Not enough data for header, wait for more data
      }
      uint16_t size = *(packet_start + 2) << 8 | *(packet_start + 3);
      if (size < packet_t::header_size + packet_t::tail_size) {
        m_receive_buffer.erase(m_receive_buffer.begin(), packet_start + 1);
        continue; // Invalid size, resync stream
      }
      if (std::distance(packet_start, m_receive_buffer.end()) <
          static_cast<std::ptrdiff_t>(size)) {
        return; // Not enough data for whole packet, wait for more data
      }
      packet_t packet{packet_start, packet_start + size, from_whole_packet};
      if (packet) {
        if (m_receive_function) {
          m_receive_function(std::move(packet));
        }
      }
      m_receive_buffer.erase(m_receive_buffer.begin(),
                             packet_start +
                                 size); // Remove processed packet from buffer
    }
  }

private:
  std::function<void(const std::uint8_t *, const std::uint8_t *)>
      m_send_function;
  std::function<void(packet_t)> m_receive_function;
  std::pmr::vector<std::uint8_t> m_receive_buffer;
};

} // namespace gdut

#endif // MODULES_TRANSFER_PROTOCOL_HPP
