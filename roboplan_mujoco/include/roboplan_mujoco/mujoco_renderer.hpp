#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <tl/expected.hpp>

#include <roboplan_mujoco/mujoco_simulation.hpp>

namespace roboplan {

/// @brief Optional planning data drawn on top of the current MuJoCo state.
struct MujocoPlanningOverlay {
  /// @brief Task space polylines in world coordinates.
  std::vector<std::vector<Eigen::Vector3d>> task_space_paths;

  /// @brief Full MuJoCo `qpos` vectors rendered as translucent robot poses.
  std::vector<Eigen::VectorXd> robot_configuration_qpos;

  /// @brief Root MuJoCo body names that identify robot geometry in ghost poses.
  /// @details A geometry is included when its body is one of these roots or a descendant.
  std::vector<std::string> ghost_root_bodies;
};

/// @brief Interactive native renderer for a MuJoCo simulation.
/// @details Displays the live simulation state and optional planning overlays. Press `P` to
/// toggle task space paths, `T` to toggle ghost robot configurations, and Escape to close the
/// window.
class MujocoRenderer {
public:
  /// @brief Creates a GLFW window and MuJoCo rendering context.
  /// @param simulation Simulation displayed by the renderer.
  /// @param title Window title.
  /// @return An owning renderer on success, else a string describing GLFW
  /// initialization or window creation failure.
  /// @note The simulation must outlive the renderer because the renderer stores
  /// non owning pointers to its model and data.
  static tl::expected<std::unique_ptr<MujocoRenderer>, std::string> create(MujocoSimulation& simulation, const std::string& title);

  /// @brief Releases the MuJoCo rendering context and GLFW window.
  ~MujocoRenderer();

  MujocoRenderer(const MujocoRenderer&) = delete;
  MujocoRenderer& operator=(const MujocoRenderer&) = delete;

  /// @brief Returns whether the render window remains open.
  bool isOpen() const;

  /// @brief Renders the current simulation state and processes window events.
  void renderFrame();

  /// @brief Replaces the planning overlay rendered in subsequent frames.
  /// @param overlay Task space paths and ghost configurations to display.
  void setPlanningOverlay(MujocoPlanningOverlay overlay);

private:
  /// @brief Private implementation containing GLFW and MuJoCo rendering resources.
  struct Impl;

  /// @brief Constructs a renderer from an initialized implementation.
  explicit MujocoRenderer(std::unique_ptr<Impl> impl);

  /// @brief Owned rendering resources and non owning simulation pointers.
  std::unique_ptr<Impl> impl_;
};
}  // namespace roboplan