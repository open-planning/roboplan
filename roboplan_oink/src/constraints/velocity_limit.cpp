#include <roboplan_oink/constraints/velocity_limit.hpp>

#include <stdexcept>

#include <roboplan_oink/optimal_ik.hpp>

namespace roboplan {

VelocityLimit::VelocityLimit(const Oink& oink, double dt, const Eigen::VectorXd& v_max)
    : dt(dt), v_max(v_max), num_variables(oink.num_variables) {
  // Validate that dt is positive
  if (dt <= 0.0) {
    throw std::invalid_argument("VelocityLimit: dt must be positive, got " + std::to_string(dt));
  }
  // Validate that v_max size matches num_variables at construction time
  if (v_max.size() != static_cast<Eigen::Index>(num_variables)) {
    throw std::invalid_argument("VelocityLimit: v_max size (" + std::to_string(v_max.size()) +
                                ") does not match oink.num_variables (" +
                                std::to_string(num_variables) + ")");
  }
}

int VelocityLimit::getNumConstraints(const Scene& /*scene*/) const { return num_variables; }

tl::expected<void, std::string> VelocityLimit::computeQpConstraints(
    const Scene& scene, Eigen::Ref<Eigen::MatrixXd> constraint_matrix,
    Eigen::Ref<Eigen::VectorXd> lower_bounds, Eigen::Ref<Eigen::VectorXd> upper_bounds) const {
  // Validate pre-allocated workspace dimensions
  if (constraint_matrix.rows() != num_variables || constraint_matrix.cols() != num_variables) {
    return tl::make_unexpected("VelocityLimit: constraint_matrix size mismatch. Expected (" +
                               std::to_string(num_variables) + " x " +
                               std::to_string(num_variables) + "), got (" +
                               std::to_string(constraint_matrix.rows()) + " x " +
                               std::to_string(constraint_matrix.cols()) + ")");
  }
  if (lower_bounds.size() != num_variables) {
    return tl::make_unexpected("VelocityLimit: lower_bounds size mismatch. Expected " +
                               std::to_string(num_variables) + ", got " +
                               std::to_string(lower_bounds.size()));
  }
  if (upper_bounds.size() != num_variables) {
    return tl::make_unexpected("VelocityLimit: upper_bounds size mismatch. Expected " +
                               std::to_string(num_variables) + ", got " +
                               std::to_string(upper_bounds.size()));
  }

  // Fill constraint matrix: identity matrix (write directly into workspace)
  // This creates constraints: -dt*v_max <= dq <= dt*v_max
  constraint_matrix.setIdentity();

  // For box constraints l <= G*dq <= u where G = I
  // Vectorized assignment
  lower_bounds = -dt * v_max;
  upper_bounds = dt * v_max;

  return {};
}

}  // namespace roboplan
