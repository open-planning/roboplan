#pragma once

#include <memory>

// Forward declaration of the ProxQP dense solver. The solver is a private implementation
// detail of roboplan_oink; consumers of this header never need the proxsuite headers.
namespace proxsuite::proxqp::dense {
template <typename T> struct QP;
}  // namespace proxsuite::proxqp::dense

namespace roboplan::detail {

/// @brief Deleter for the forward-declared ProxQP solver.
///
/// Defined out-of-line in qp_backend.cpp, where the complete solver type is available.
/// This lets Oink hold the solver through a unique_ptr without instantiating the
/// (compile-time expensive) proxsuite templates in every translation unit that
/// constructs or destroys an Oink.
struct QpDeleter {
  void operator()(proxsuite::proxqp::dense::QP<double>* solver) const;
};

/// @brief Owning handle to the ProxQP dense solver, usable with an incomplete solver type.
using QpSolverPtr = std::unique_ptr<proxsuite::proxqp::dense::QP<double>, QpDeleter>;

}  // namespace roboplan::detail
