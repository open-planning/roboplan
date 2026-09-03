#pragma once

#include <functional>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include <aligator/modelling/costs/sum-of-costs.hpp>  // CostStackTpl

#include <roboplan_aligator/costs.hpp>  // spec types + CostHandle

#include <roboplan_aligator/problem_builder.hpp>  // PhaseSpace

namespace roboplan {

class ReducedGroupModel;

namespace aligator_detail {

using CostStack = aligator::CostStackTpl<double>;

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

struct CostHandle::Impl {
  enum class Kind { Pose, Vector } kind = Kind::Vector;

  int expected_size = -1;

  std::vector<std::function<void(const Eigen::Matrix4d&)>> pose_setters;
  std::vector<std::function<void(const Eigen::VectorXd&)>> vector_setters;
};

}  // namespace roboplan
