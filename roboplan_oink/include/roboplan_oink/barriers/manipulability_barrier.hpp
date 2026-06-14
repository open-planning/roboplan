#pragma once

#include <Eigen/Dense>
#include <string>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <roboplan_oink/optimal_ik.hpp>

namespace roboplan {

/// @brief Singularity-avoidance barrier based on the minimum singular value of the arm Jacobian.
///
/// Enforces the safety constraint:
///   h(q) = σ_min(q) − σ_safe ≥ 0
///
/// where σ_min(q) is the smallest singular value of the 6×n arm Jacobian evaluated at the
/// end-effector frame.  As σ_min → 0 the robot approaches a kinematic singularity and the
/// barrier pushes back proportionally through the discrete-time CBF condition:
///
///   ∇σ_min · Δq / dt ≥ −γ · α(h(q))
///
/// σ_min is preferred over the Yoshikawa manipulability index because its gradient
///   ∂σ_min/∂qᵢ = u_minᵀ (∂J/∂qᵢ) v_min
/// does not involve J⁺ and therefore remains well-conditioned at σ_min = 0.
///
/// The gradient is computed via forward finite differences: one FK + Jacobian call per arm
/// joint (n calls per control step).  Results are cached between computeBarrier() and
/// computeJacobian() to avoid recomputation.
///
/// Uses the saturating class-K function α(h) = γ·h/(1+|h|) (inherited from Barrier base).
struct ManipulabilityBarrier : public Barrier {
  /// @brief Construct a manipulability barrier.
  /// @param oink        Oink solver (provides v_indices, q_indices, num_variables).
  /// @param scene       Scene used to resolve the frame ID at construction time.
  /// @param frame_name  Name of the end-effector frame whose Jacobian is measured.
  /// @param dt          Timestep matching your control loop period.
  /// @param sigma_safe  Minimum σ_min the barrier enforces. Tune above the singularity
  ///                    threshold (~0.05 for UR5) to leave a comfortable margin.
  /// @param gain        Barrier gain γ — higher values give a stiffer response. Default 1.0.
  /// @param safe_displacement_gain  Gain for safe-displacement regularization. Default 1.0.
  /// @param safety_margin  Conservative margin subtracted from h before applying class-K.
  ///                       Default 0.0.
  /// @param fd_epsilon  Step size for finite-difference gradient. Default 1e-6.
  /// @throws std::runtime_error if frame_name is not found in the scene.
  ManipulabilityBarrier(const Oink& oink, const Scene& scene, const std::string& frame_name,
                        double dt, double sigma_safe, double gain = 1.0,
                        double safe_displacement_gain = 1.0, double safety_margin = 0.0,
                        double fd_epsilon = 1e-6);

  int getNumBarriers(const Scene& scene) const override;

  /// @brief Compute h(q) = σ_min(q) − σ_safe and cache σ_min for the Jacobian step.
  tl::expected<void, std::string> computeBarrier(const Scene& scene) override;

  /// @brief Compute ∂σ_min/∂q via forward finite differences.
  ///
  /// For each arm joint i:
  ///   jacobian_container(0, i) = (σ_min(q + ε·eᵢ) − σ_min(q)) / ε
  ///
  /// σ_min(q) is reused from the preceding computeBarrier() call.
  tl::expected<void, std::string> computeJacobian(const Scene& scene) override;

  /// @brief Evaluate h at a candidate configuration for post-solve validation.
  tl::expected<double, std::string>
  evaluateAtConfiguration(const pinocchio::Model& model, pinocchio::Data& data,
                          const Eigen::VectorXd& q) const override;

  const std::string frame_name;  ///< Name of the constrained frame.
  const double sigma_safe;       ///< Minimum σ_min the barrier enforces.
  const double fd_epsilon;       ///< Finite-difference step size.

  const Eigen::VectorXi v_indices;   ///< Velocity indices of the joint group.
  const Eigen::VectorXi q_indices;   ///< Position indices of the joint group.
  pinocchio::FrameIndex frame_id{};  ///< Eagerly resolved frame index.

private:
  double computeSigmaMin(const Eigen::VectorXd& q);

  pinocchio::Model model_;         ///< Copy of the model for FD computations.
  pinocchio::Data data_;           ///< Pinocchio data for FD computations.
  Eigen::MatrixXd full_jacobian_;  ///< 6 × model.nv workspace.
  Eigen::MatrixXd j_arm_;          ///< 6 × v_indices.size() arm-Jacobian workspace.
  double cached_sigma0_ = 0.0;     ///< σ_min cached by computeBarrier for reuse.
};

}  // namespace roboplan
