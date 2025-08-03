#include <chrono>
#include <stdexcept>

#include <roboplan/core/path_utils.hpp>
#include <roboplan_toppra/toppra.hpp>

namespace roboplan {

void pathParameterizeToppra(std::shared_ptr<Scene> scene, const JointPath& path, const double dt) {

  // TODO: Extract joint velocity + acceleration limits from scene and create constraints.

  // TODO: Create initial cubic spline with path and random times

  // TODO: Solve TOPP-RA problem

  // TODO: Evaluate the parameterized path at the specified times.
}

}  // namespace roboplan
