#pragma once

#include <variant>

#include <roboplan_aligator/constraints.hpp>

namespace roboplan {

/// @brief Unified constraint specification: any concrete constraint type accepted by
/// addConstraint().
/// @details Provides implicit conversion from each constraint struct so that `addConstraint`
/// accepts any of them directly (nanobind converts Python constraint objects to ConstraintSpec via
/// these constructors).
using ConstraintSpec = std::variant<PositionLimit, VelocityLimit, TorqueLimit, FramePoseConstraint,
                                    SelfCollisionConstraint, CollisionConstraint>;

}  // namespace roboplan
