#include <nanobind/nanobind.h>

#include <modules/aligator.hpp>

namespace roboplan {

// Placeholder binding registration for the package scaffolding (Prompt 1). The public
// trajectory optimization surface (TrajOptOptions/TrajOptSeed/TrajOptResult, cost and
// constraint factories, TrajectoryOptimizer) is bound in later implementation prompts per
// the design doc §4.
void init_aligator(nanobind::module_& m) { (void)m; }

}  // namespace roboplan
