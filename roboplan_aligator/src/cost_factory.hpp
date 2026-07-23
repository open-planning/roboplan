#pragma once

// INTERNAL helper — NOT public API. Builds the aligator residual+wrapper for each cost spec
// (design §4.3) and wires the mutable-target closures (design §3.5). Keeps aligator/pinocchio out
// of the public headers. Shared by trajectory_optimizer.cpp (which resolves the target stacks) and
// test_costs.cpp. Verified symbols recorded in API_NOTES.md.

#include <functional>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include <aligator/modelling/costs/sum-of-costs.hpp>  // CostStackTpl

#include <roboplan_aligator/costs.hpp>  // spec types + CostHandle

#include "problem_builder.hpp"  // PhaseSpace

namespace roboplan {

class ReducedGroupModel;  // internal (src/reduced_group_model.hpp)

namespace aligator_detail {

using CostStack = aligator::CostStackTpl<double>;

// Each attach* builds the cost for one spec, adds it to `stack` (an IN-PROBLEM cost stack, never a
// temporary — the returned pointer must stay valid, §3.5) with overall `weight`, and returns a
// closure bound to the in-stack residual/cost that applies a new target. Frames are resolved
// against `rgm`; an unknown frame or a wrong-size spec field throws std::invalid_argument (a
// setup-time invariant violation).
std::function<void(const Eigen::Matrix4d&)>
attachFramePoseCost(CostStack& stack, const PhaseSpace& space, const ReducedGroupModel& rgm,
                    const FramePoseCost& spec, double weight);

std::function<void(const Eigen::VectorXd&)>
attachFrameAxisCost(CostStack& stack, const PhaseSpace& space, const ReducedGroupModel& rgm,
                    const FrameAxisCost& spec, double weight);

std::function<void(const Eigen::VectorXd&)>
attachConfigurationCost(CostStack& stack, const PhaseSpace& space, const ReducedGroupModel& rgm,
                        const ConfigurationCost& spec, double weight);

std::function<void(const Eigen::VectorXd&)>
attachControlCost(CostStack& stack, const PhaseSpace& space, const ReducedGroupModel& rgm,
                  const ControlCost& spec, double weight);

std::function<void(const Eigen::VectorXd&)>
attachVelocityCost(CostStack& stack, const PhaseSpace& space, const ReducedGroupModel& rgm,
                   const VelocityCost& spec, double weight);

}  // namespace aligator_detail

// Definition of the opaque CostHandle::Impl (declared in the public costs.hpp). Kept in this
// internal header so both trajectory_optimizer.cpp (which builds it) and cost_factory.cpp (which
// defines CostHandle's methods) see the complete type. Holds only std/Eigen closures — no aligator
// types leak here.
struct CostHandle::Impl {
  /// @brief Which setTarget overload is valid for this handle.
  enum class Kind { Pose, Vector } kind = Kind::Vector;

  /// @brief Expected target vector size for Kind::Vector (nq / nv / 3); unused for Pose.
  int expected_size = -1;

  /// @brief One setter per in-range stage (+ terminal), each bound to an in-problem residual.
  std::vector<std::function<void(const Eigen::Matrix4d&)>> pose_setters;
  std::vector<std::function<void(const Eigen::VectorXd&)>> vector_setters;
};

}  // namespace roboplan
