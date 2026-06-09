#pragma once

#include <array>
#include <bit>
#include <cstring>
#include <stdexcept>
#include <tuple>
#include <type_traits>

template <typename... Args> struct get_size {};

template <typename T>
struct get_size<T> : std::integral_constant<std::size_t, sizeof(T)> {};

template <typename T, std::size_t Size>
struct get_size<std::array<T, Size>>
    : std::integral_constant<std::size_t, sizeof(T) * Size> {};

template <typename T, typename... Args>
struct get_size<T, Args...>
    : std::integral_constant<std::size_t,
                             get_size<T>::value + get_size<Args...>::value> {};

template <typename... Args>
inline constexpr std::size_t get_size_v = get_size<Args...>::value;

template <typename T> struct is_array : std::false_type {};

template <template <typename, std::size_t> class Tmp, typename T,
          std::size_t Size>
struct is_array<Tmp<T, Size>>
    : std::is_same<Tmp<T, Size>, std::array<T, Size>> {};

template <typename T> struct get_array_type {};

template <template <typename, std::size_t> class Tmp, typename T,
          std::size_t Size>
struct get_array_type<Tmp<T, Size>> {
  using type = T;
};

template <typename T>
struct get_array_size : std::integral_constant<std::size_t, 0> {};

template <template <typename, std::size_t> class Tmp, typename T,
          std::size_t Size>
struct get_array_size<Tmp<T, Size>>
    : std::integral_constant<std::size_t, Size> {};

template <typename T> struct convert_to_array { using type = T; };

template <typename T, std::size_t Size>
struct convert_to_array<std::array<T, Size>> {
  using type = std::array<T, Size>;
};

struct format_to_string_t {};
struct format_to_array_t {};

inline constexpr format_to_string_t format_to_string{};
inline constexpr format_to_array_t format_to_array{};

template <typename T> struct always_false : std::false_type {};

template <typename... Args> struct payload_structure {
  static_assert((!std::is_pointer_v<Args> && ...));
  static_assert(((std::is_trivial_v<Args> || is_array<Args>::value) && ...));

  // 整个payload的大小
  static constexpr std::size_t size = get_size_v<Args...>;

  // 将参数格式化为二进制数据
  template <typename... FormatArgs,
            typename = std::enable_if_t<
                ((std::is_same_v<
                      typename convert_to_array<std::decay_t<FormatArgs>>::type,
                      Args> ||
                  std::is_same_v<std::decay_t<FormatArgs>, std::string> ||
                  std::is_convertible_v<std::decay_t<FormatArgs>, Args>)&&...)>>
  static std::array<char, size> format(format_to_array_t,
                                       FormatArgs &&...args) {
    using BufferType = std::array<char, size>;
    BufferType buffer{};
    std::size_t offset = 0;
    auto func = [&buffer, &offset](auto &&arg, auto &&struct_arg) {
      using ArgType = std::decay_t<decltype(arg)>;
      using StructArgType = std::decay_t<decltype(struct_arg)>;
      if constexpr (std::is_trivial_v<ArgType>) {
        write_to_buffer<BufferType, StructArgType>(buffer, offset, arg);
        offset += get_size_v<StructArgType>;
      } else if constexpr (is_array<ArgType>::value) {
        write_array_to_buffer<BufferType,
                              typename get_array_type<StructArgType>::type,
                              get_array_size<StructArgType>::value>(
            buffer, offset, arg);
        offset += get_size_v<StructArgType>;
      } else if constexpr (std::is_same_v<std::string, ArgType>) {
        StructArgType array{};
        std::size_t copy_size = std::min(arg.size(), sizeof(array));
        for (std::size_t i = 0; i < copy_size; ++i) {
          array[i] = convert_endian(arg[i]);
        }
        write_array_to_buffer<BufferType,
                              typename get_array_type<StructArgType>::type,
                              get_array_size<StructArgType>::value>(
            buffer, offset, array);
        offset += get_array_size<StructArgType>::value;
      } else {
        static_assert(always_false<ArgType>::value, "Unsupported type");
      }
    };
    (..., func(std::forward<FormatArgs>(args), Args{}));
    return buffer;
  }

  // 将参数格式化为二进制数据
  template <typename... FormatArgs,
            typename = std::enable_if_t<
                ((std::is_same_v<
                      typename convert_to_array<std::decay_t<FormatArgs>>::type,
                      Args> ||
                  std::is_same_v<std::decay_t<FormatArgs>, std::string> ||
                  std::is_convertible_v<std::decay_t<FormatArgs>, Args>)&&...)>>
  static std::string format(format_to_string_t, FormatArgs &&...args) {
    using BufferType = std::string;
    BufferType buffer;
    buffer.resize(size);
    std::size_t offset = 0;
    auto func = [&buffer, &offset](auto &&arg, auto &&struct_arg) {
      using ArgType = std::decay_t<decltype(arg)>;
      using StructArgType = std::decay_t<decltype(struct_arg)>;
      if constexpr (std::is_trivial_v<ArgType>) {
        write_to_buffer<BufferType, StructArgType>(buffer, offset, arg);
        offset += get_size_v<StructArgType>;
      } else if constexpr (is_array<ArgType>::value) {
        write_array_to_buffer<BufferType,
                              typename get_array_type<StructArgType>::type,
                              get_array_size<StructArgType>::value>(
            buffer, offset, arg);
        offset += get_size_v<StructArgType>;
      } else if constexpr (std::is_same_v<std::string, ArgType>) {
        StructArgType array{};
        std::size_t copy_size = std::min(arg.size(), sizeof(array));
        for (std::size_t i = 0; i < copy_size; ++i) {
          array[i] = convert_endian(arg[i]);
        }
        write_array_to_buffer<BufferType,
                              typename get_array_type<StructArgType>::type,
                              get_array_size<StructArgType>::value>(
            buffer, offset, array);
        offset += get_array_size<StructArgType>::value;
      } else {
        static_assert(always_false<ArgType>::value, "Unsupported type");
      }
    };
    (..., func(std::forward<FormatArgs>(args), Args{}));
    return buffer;
  }

  // 从二进制数据中解析出参数
  template <typename BufferType,
            typename = std::enable_if_t<
                std::is_same_v<BufferType, std::string> ||
                std::is_same_v<BufferType, std::array<char, size>>>>
  static std::tuple<Args...> parse(const BufferType &buffer) {
    if (buffer.size() != size) {
      throw std::runtime_error("Invalid buffer size");
    }
    std::size_t offset = 0;
    auto func = [&buffer, &offset](auto &&arg) {
      using ArgType = std::decay_t<decltype(arg)>;
      if constexpr (is_array<ArgType>::value) {
        auto array =
            read_array_from_buffer<BufferType,
                                   typename get_array_type<ArgType>::type,
                                   get_array_size<ArgType>::value>(buffer,
                                                                   offset);
        offset += get_size_v<ArgType>;
        return array;
      } else {
        auto value = read_from_buffer<BufferType, ArgType>(buffer, offset);
        offset += get_size_v<ArgType>;
        return value;
      }
    };
    std::tuple<Args...> result;
    std::apply([&func](auto &&...args) { (..., (args = func(args))); }, result);
    return result;
  }

protected:
  template <typename T> static T convert_endian(T value) {
    if constexpr (std::endian::native == std::endian::big) {
      return value;
    } else {
      T result = 0;
      for (std::size_t i = 0; i < sizeof(T); ++i) {
        result |= ((value >> (i * 8)) & 0xFF) << ((sizeof(T) - 1 - i) * 8);
      }
      return result;
    }
  }

  template <typename BufferType, typename T>
  static T read_from_buffer(const BufferType &buffer, std::size_t offset) {
    T value;
    std::memcpy(&value, buffer.data() + offset, sizeof(T));
    return convert_endian(value);
  }

  template <typename BufferType, typename T>
  static void write_to_buffer(BufferType &buffer, std::size_t offset,
                              const T &value) {
    T converted = convert_endian(value);
    std::memcpy(buffer.data() + offset, &converted, sizeof(T));
  }

  template <typename BufferType, typename T, std::size_t Size>
  static std::array<T, Size> read_array_from_buffer(const BufferType &buffer,
                                                    std::size_t offset) {
    std::array<T, Size> array;
    for (std::size_t i = 0; i < Size; ++i) {
      array[i] =
          read_from_buffer<BufferType, T>(buffer, offset + i * sizeof(T));
    }
    return array;
  }

  template <typename BufferType, typename T, std::size_t Size>
  static void write_array_to_buffer(BufferType &buffer, std::size_t offset,
                                    const std::array<T, Size> &array) {
    for (std::size_t i = 0; i < Size; ++i) {
      write_to_buffer<BufferType, T>(buffer, offset + i * sizeof(T), array[i]);
    }
  }
};
