#include <filesystem>
#include <memory>
#include <vector>

#include <roboplan/core/path_utils.hpp>
#include <roboplan/core/scene.hpp>
#include <roboplan_example_models/resources.hpp>
#include <roboplan_rrt/rrt.hpp>
#include <roboplan_toppra/toppra.hpp>

using namespace roboplan;

int main(int /*argc*/, char* /*argv*/[]) {

  const auto share_prefix = example_models::get_package_share_dir();

  // Set up the scene
  const auto urdf_path = share_prefix / "ur_robot_model" / "ur5_gripper.urdf";
  const auto srdf_path = share_prefix / "ur_robot_model" / "ur5_gripper.srdf";
  const auto yaml_config_path = share_prefix / "ur_robot_model" / "ur5_config.yaml";
  const std::vector<std::filesystem::path> package_paths = {share_prefix};

  auto scene = std::make_shared<Scene>("example_rrt_scene", urdf_path, srdf_path, package_paths,
                                       yaml_config_path);

  // Set up the RRT
  RRTOptions options;
  options.group_name = "arm";
  options.max_connection_distance = 1.0;
  options.collision_check_step_size = 0.05;
  options.goal_biasing_probability = 0.15;
  options.max_nodes = 1000;
  options.max_planning_time = 5.0;
  auto rrt = RRT(scene, options);

  // Pick random initial and goal configurations and plan motion
  const auto q_full_opt = scene->randomCollisionFreePositions();
  if (!q_full_opt) {
    std::cout << "Failed to generate random collision free positions" << std::endl;
    return 1;
  }
  const auto& q_full = q_full_opt.value();
  scene->setJointPositions(q_full);

  const auto q_indices = scene->getJointGroupInfo("arm")->q_indices;

  JointConfiguration start;
  start.positions = q_full(q_indices);

  JointConfiguration goal;
  goal.positions = scene->randomCollisionFreePositions().value()(q_indices);

  std::cout << "Planning..." << std::endl;
  const auto maybe_path = rrt.plan(start, goal);
  if (!maybe_path) {
    std::cout << "Failed to plan path: " << maybe_path.error() << std::endl;
  }
  auto path = maybe_path.value();
  std::cout << "Found a path:\n" << path << std::endl;

  // Optionally include path shortcutting
  const auto include_shortcutting = true;
  if (include_shortcutting) {
    const auto max_iters = 1000;
    auto shortcutter = PathShortcutter(scene, "arm");
    path = shortcutter.shortcut(path, options.collision_check_step_size, max_iters);
    std::cout << "Shortcutted path:\n" << path << std::endl;
  }

  // Set up TOPP-RA to time-parameterize the path
  std::cout << "Generating trajectory..." << std::endl;
  const auto dt = 0.01;
  auto toppra = PathParameterizerTOPPRA(scene, "arm");
  const auto maybe_traj = toppra.generate(path, dt);
  if (!maybe_traj) {
    std::cout << "Failed to generate trajectory: " << maybe_traj.error() << std::endl;
  }
  const auto& traj = maybe_traj.value();
  std::cout << "Successfully generated trajectory with duration of " << traj.times.back()
            << " seconds." << std::endl;

  return 0;
}
