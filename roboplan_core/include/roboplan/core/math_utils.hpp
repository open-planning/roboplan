#pragma once

#include <Eigen/Dense>

namespace roboplan {

/// @brief Solves the damped least-squares step that cancels a task space error.
/// @details Computes `v = -J^T (J J^T + damping * I)^-1 e`, the minimum-norm joint velocity that
/// drives the error to zero, with Tikhonov damping to stay conditioned near singularities.
/// @param jacobian The task Jacobian, with one row per error coordinate.
/// @param error The task space error to cancel, with one entry per Jacobian row.
/// @param damping Tikhonov damping added to the diagonal of `J J^T`.
/// Larger values trade accuracy near singularities for a better conditioned solve.
/// @param jjt Scratch for `J J^T`, square with one row per error coordinate.
/// Any contents on entry are overwritten.
/// @param velocity Output joint velocity, with one entry per Jacobian column.
/// @return True if the step is usable, false if it came out NaN, which means the damped system
/// was still too ill-conditioned to factor.
template <typename JacobianType, typename ErrorType>
bool dampedLeastSquaresStep(const Eigen::MatrixBase<JacobianType>& jacobian,
                            const Eigen::MatrixBase<ErrorType>& error, double damping,
                            Eigen::Ref<Eigen::MatrixXd> jjt, Eigen::Ref<Eigen::VectorXd> velocity) {
  jjt.noalias() = jacobian * jacobian.transpose();
  jjt.diagonal().array() += damping;
  velocity.noalias() = -jacobian.transpose() * jjt.ldlt().solve(error);
  return !velocity.hasNaN();
}

}  // namespace roboplan
