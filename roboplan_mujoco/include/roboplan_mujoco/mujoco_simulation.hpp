#pragma once

#include <cstddef>
#include <memory>

#include <mujoco/mujoco.h>

namespace roboplan {
class MujocoModelBuilder;

/// @brief Deletes an `mjSpec` with the corresponding MuJoCo API.
struct MujocoSpecDeleter {
  void operator()(mjSpec* spec) const;
};

/// @brief Deletes an `mjModel` with the corresponding MuJoCo API.
struct MujocoModelDeleter {
  void operator()(mjModel* model) const;
};

/// @brief Deletes an `mjData` with the corresponding MuJoCo API.
struct MujocoDataDeleter {
  void operator()(mjData* data) const;
};

/// @brief Unique owner for a mutable MuJoCo model specification.
using MujocoSpecPtr = std::unique_ptr<mjSpec, MujocoSpecDeleter>;

/// @brief Unique owner for a compiled MuJoCo model.
using MujocoModelPtr = std::unique_ptr<mjModel, MujocoModelDeleter>;

/// @brief Unique owner for mutable MuJoCo simulation data.
using MujocoDataPtr = std::unique_ptr<mjData, MujocoDataDeleter>;

/// @brief Owns a compiled MuJoCo model and its mutable simulation state.
/// @details The model and data lifetimes are coupled so `mjData` never outlives the `mjModel`
/// from which it was allocated. Simulations are move only because both objects have unique
/// ownership.
class MujocoSimulation {
public:
  MujocoSimulation(const MujocoSimulation&) = delete;
  MujocoSimulation& operator=(const MujocoSimulation&) = delete;
  MujocoSimulation(MujocoSimulation&&) noexcept = default;

  /// @brief Returns the immutable compiled MuJoCo model.
  const mjModel& model() const;

  /// @brief Returns mutable MuJoCo simulation data.
  mjData& data();

  /// @brief Returns immutable MuJoCo simulation data.
  const mjData& data() const;

  /// @brief Restores the simulation data to the model defaults.
  /// @details Calls `mj_resetData` followed by `mj_forward`.
  void reset();

  /// @brief Recomputes MuJoCo position and velocity dependent quantities.
  void forward();

  /// @brief Advances the simulation by one or more model time steps.
  /// @param substeps Number of calls to `mj_step`. Must be greater than zero.
  void step(std::size_t substeps = 1);

  /// @brief Returns the current simulation time in seconds.
  double time() const;

private:
  friend class MujocoModelBuilder;

  /// @brief Constructs a simulation from a compatible compiled model and data pair.
  MujocoSimulation(MujocoModelPtr model, MujocoDataPtr data);

  /// @brief Compiled model used by the simulation.
  MujocoModelPtr model_;

  /// @brief Mutable state allocated from model_.
  MujocoDataPtr data_;
};
}  // namespace roboplan