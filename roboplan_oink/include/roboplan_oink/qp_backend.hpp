#pragma once

#include <string>

#include <Eigen/Core>
#include <tl/expected.hpp>

#include <roboplan_oink/detail/qp_solver_fwd.hpp>
#include <roboplan_oink/oink_settings.hpp>

// Private interface to the ProxQP dense backend. This header deliberately avoids the
// roboplan scene/pinocchio headers so that qp_backend.cpp -- the only translation unit
// that instantiates the (compile-time expensive) proxsuite solver templates -- stays
// cheap to parse, and optimal_ik.cpp never sees the proxsuite headers at all.
namespace roboplan::detail {

/// @brief Initialize/update the ProxQP solver with the assembled QP and solve it.
///
/// @param solver The solver handle. (Re)constructed in place when init_required is true.
/// @param settings Oink solver settings applied when the solver is (re)constructed.
/// @param init_required Rebuild the solver (ProxQP fixes problem dimensions at construction).
/// @param num_variables Number of decision variables.
/// @param total_rows Number of inequality rows in A/lower/upper (0 for unconstrained).
/// @param H Quadratic objective matrix (num_variables x num_variables).
/// @param c Linear objective vector (num_variables).
/// @param A Inequality constraint matrix (total_rows x num_variables).
/// @param lower Inequality lower bounds (total_rows).
/// @param upper Inequality upper bounds (total_rows).
/// @param delta_q Pre-allocated output buffer for the solution.
/// @return void on success, error message on failure.
tl::expected<void, std::string>
solveQp(QpSolverPtr& solver, const OinkSettings& settings, bool init_required, int num_variables,
        int total_rows, const Eigen::MatrixXd& H, const Eigen::VectorXd& c,
        const Eigen::MatrixXd& A, const Eigen::VectorXd& lower, const Eigen::VectorXd& upper,
        Eigen::Ref<Eigen::VectorXd, 0, Eigen::InnerStride<Eigen::Dynamic>> delta_q);

}  // namespace roboplan::detail
