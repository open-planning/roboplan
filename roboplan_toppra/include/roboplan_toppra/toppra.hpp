#pragma once

#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>
#include <toppra/algorithm/toppra.hpp>

namespace roboplan {

/// @brief Time-parameterizes a joint-space path using TOPP-RA.
/// @details This directly uses https://github.com/hungpham2511/toppra
/// @param scene A pointer to the scene to use for path parameterization.
/// @param path The joint-space path to time parameterize.
/// @param dt The sample time of the output trajectory, in seconds.
/// @return A time-parameterized joint trajectory.
void pathParameterizeToppra(std::shared_ptr<Scene> scene, const JointPath& path, const double dt);

}  // namespace roboplan
