#pragma once

// Finite-difference helpers that touch the reduced phase space, so they require aligator (via
// problem_builder.hpp). Only the aligator-linked test targets include this.

#include <Eigen/Dense>

#include <roboplan_aligator/problem_builder.hpp>

namespace roboplan::testing {

// A deterministic, non-trivial state on the phase space (reproducible across runs): a fixed tangent
// offset from the neutral configuration.
inline Eigen::VectorXd deterministicState(const aligator_detail::PhaseSpace& space) {
  Eigen::VectorXd delta(space.ndx());
  for (int i = 0; i < space.ndx(); ++i) {
    delta(i) = 0.1 * (i + 1);
  }
  return space.integrate(space.neutral(), delta);
}

}  // namespace roboplan::testing
