#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include <tl/expected.hpp>

#include <roboplan_mujoco/mujoco_simulation.hpp>

namespace roboplan {
/// @brief Gains and force limit used when creating MuJoCo position actuators.
struct MujocoPositionActuatorOptions {
  /// @brief Proportional position gain.
  double kp{100.0};

  /// @brief Derivative velocity gain.
  double kv{20.0};

  /// @brief Symmetric actuator force limit.
  double max_force{100.0};
};

/// @brief Builds a MuJoCo model specification from a RoboPlan robot description.
/// @details Separates parsing and model editing from compilation. The builder owns the
/// mutable `mjSpec`; the compiled `mjModel` and `mjData` are returned together as a
/// MujocoSimulation.
class MujocoModelBuilder {
public:
  /// @brief Creates a mutable MuJoCo specification from a URDF file.
  /// @details Configures the MuJoCo URDF compiler to preserve visual geometry, imports
  /// URDF mimic relationships as MuJoCo equality constraints, and hides collision
  /// geometry from rendering without disabling it for physics.
  /// @param urdf_path Path to the URDF to import.
  /// @return A model builder on success, else a string describing the parsing or import
  /// error.
  static tl::expected<MujocoModelBuilder, std::string> fromUrdf(const std::filesystem::path& urdf_path);

  /// @brief Creates a mutable MuJoCo specification by attaching a URDF robot to an MJCF
  /// scene.
  /// @details The MJCF scene supplies world elements such as the ground, lights, and
  /// cameras. Its world body becomes the parent of the imported robot.
  /// @param urdf_path Path to the robot URDF to import.
  /// @param scene_mjcf_path Path to the MJCF scene that receives the robot.
  /// @return A model builder on success, else a string describing the parsing,
  /// attachment, or import error.
  static tl::expected<MujocoModelBuilder, std::string> fromUrdf(const std::filesystem::path& urdf_path, const std::filesystem::path& scene_mjcf_path);

  MujocoModelBuilder(const MujocoModelBuilder&) = delete;
  MujocoModelBuilder& operator=(const MujocoModelBuilder&) = delete;
  MujocoModelBuilder(MujocoModelBuilder&&) noexcept = default;

  /// @brief Returns the mutable MuJoCo model specification owned by this builder.
  /// @details The returned reference is valid until the builder is destroyed or moved.
  /// It can be used to add application specific bodies, geometry, or simulation options
  /// before calling compile().
  mjSpec& spec();

  /// @brief Adds unit gear position servos for the requested RoboPlan joints.
  /// @details Scalar joints receive one actuator. A planar joint is expanded to its
  /// MuJoCo `_TX`, `_TY`, and `_RZ` joints. Each target must exist, be a hinge or slide
  /// joint, and have no existing direct actuator.
  /// @param joint_names RoboPlan joint names to actuate.
  /// @param options Position gains and symmetric force limit for each actuator.
  /// @return Success, or a string describing an invalid option or joint.
  tl::expected<void, std::string> addPositionServos(const std::vector<std::string>& joint_names, const MujocoPositionActuatorOptions& options = {});

  /// @brief Adds MuJoCo contact exclusions from SRDF `disable_collisions` entries.
  /// @param srdf_path Path to the SRDF associated with the imported robot.
  /// @return Success, or a string describing a parse error or missing MuJoCo body.
  tl::expected<void, std::string> addCollisionExclusionsFromSrdf(const std::filesystem::path& srdf_path);

  /// @brief Compiles the current specification and allocates its simulation data.
  /// @return An owning simulation on success, else a string describing the MuJoCo
  /// compilation or allocation error.
  tl::expected<MujocoSimulation, std::string> compile();

private:
  /// @brief Constructs a builder that owns an already parsed MuJoCo specification.
  explicit MujocoModelBuilder(MujocoSpecPtr spec);

  /// @brief Mutable MuJoCo model specification being assembled.
  MujocoSpecPtr spec_;
};
}  // namespace roboplan