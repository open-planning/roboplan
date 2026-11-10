#pragma once

#include <stdexcept>
#include <string>
#include <tl/expected.hpp>

/// @brief Helper function for checking if a tl::expected return type has a value or an error.
/// @details If a value exists, we return it as is. When handling unexpected values, if
/// the error is string convertible then we can pass it along through a `std::runtime_error`.
/// Otherwise we do not know the details of the underlying exception.
/// @return The unwrapped value, or throw an exception.
/// @throw std::runtime_error if the result is an error.
template <typename Expected> typename Expected::value_type handle_expected(Expected&& result) {
  if (result.has_value()) {
    if constexpr (std::is_void_v<typename Expected::value_type>) {
      return;  // Return nothing since it's a void type
    } else {
      return result.value();
    }
  } else {
    // TODO: Consider wrapping with a streamable option.
    if constexpr (std::is_convertible_v<typename Expected::error_type, std::string>) {
      throw std::runtime_error(std::string(result.error()));
    } else {
      throw std::runtime_error("Unknown error occurred.");
    }
  }
};

/// @brief Wrapper function binding tl::expected return types in class functions with nanobind.
/// @return The unwrapped value, or throw a runtime_error.
template <typename Class, typename Ret, typename Err, typename... Args>
auto unwrap_expected(tl::expected<Ret, Err> (Class::*method)(Args...)) {
  return [method](Class& self, Args... args) -> Ret {
    return handle_expected((self.*method)(args...));
  };
}

/// @brief Wrapper function binding tl::expected return types in const class functions with
/// nanobind.
/// @return The unwrapped value, or throw a runtime_error.
template <typename Class, typename Ret, typename Err, typename... Args>
auto unwrap_expected(tl::expected<Ret, Err> (Class::*method)(Args...) const) {
  return [method](const Class& self, Args... args) -> Ret {
    return handle_expected((self.*method)(args...));
  };
}

/// @brief Wrapper function binding tl::expected return types in free functions with nanobind.
/// @return The unwrapped value, or throw a runtime_error.
template <typename Ret, typename Err, typename... Args>
auto unwrap_expected(tl::expected<Ret, Err> (*method)(Args...)) {
  return [method](Args... args) -> Ret { return handle_expected((*method)(args...)); };
}
