#pragma once

#include <vector>

#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>

#include <roboplan/core/types.hpp>

NAMESPACE_BEGIN(NB_NAMESPACE)
NAMESPACE_BEGIN(detail)

/// @brief Loads a Python sequence into a roboplan::Span argument.
/// @details The elements are copied into a vector owned by this caster, which outlives the bound
/// function call. The Python type name matches std::vector's, so generated stubs are unchanged.
template <typename T> struct type_caster<roboplan::Span<T>> {
  using StorageCaster = make_caster<std::vector<T>>;

  NB_TYPE_CASTER(roboplan::Span<T>, StorageCaster::Name)

  bool from_python(handle src, uint8_t flags, cleanup_list* cleanup) noexcept {
    if (!storage_.from_python(src, flags, cleanup)) {
      return false;
    }
    value = roboplan::Span<T>(storage_.value);
    return true;
  }

private:
  StorageCaster storage_;
};

NAMESPACE_END(detail)
NAMESPACE_END(NB_NAMESPACE)
