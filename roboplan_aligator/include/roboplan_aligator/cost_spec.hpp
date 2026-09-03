#pragma once

#include <variant>

#include <roboplan_aligator/costs.hpp>

namespace roboplan {

/// @brief Unified cost specification: any concrete cost type accepted by addCost().
/// @details Provides implicit conversion from each cost struct so that `addCost` accepts any of
/// them directly (nanobind converts Python cost objects to CostSpec via these constructors).
using CostSpec =
    std::variant<FramePoseCost, FrameAxisCost, ConfigurationCost, ControlCost, VelocityCost>;

}  // namespace roboplan
