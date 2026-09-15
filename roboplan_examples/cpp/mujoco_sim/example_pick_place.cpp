#include <algorithm>
#include <array>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tl/expected.hpp>

#include <roboplan/core/geometry_wrappers.hpp>
#include <roboplan/core/scene.hpp>
#include <roboplan_cartesian_planning/cartesian_path_planner.hpp>
#include <roboplan_example_models/resources.hpp>
#include <roboplan_mujoco/mujoco_hardware_interface.hpp>
#include <roboplan_mujoco/mujoco_model_builder.hpp>
#include <roboplan_mujoco/mujoco_renderer.hpp>

using namespace roboplan;

namespace {

constexpr auto kGroup = "fr3_arm";
constexpr auto kTcp = "fr3_hand_tcp";
constexpr auto kRootBody = "fr3_link0";
constexpr double kOpenGripper = 0.04;
constexpr double kClosedGripper = 0.005;
constexpr double kRenderPeriod = 1.0 / 60.0;

struct World {
    Eigen::Vector3d table_size{0.8, 0.9, 0.1};
    Eigen::Vector3d table_position{0.55, 0.0, 0.25};
    Eigen::Vector3d block_size{0.04, 0.04, 0.08};
    Eigen::Vector3d pick_position{0.50, 0.25, 0.34};
    Eigen::Vector3d place_position{0.50, -0.25, 0.34};
    Eigen::Vector3d barrier_size{0.24, 0.12, 0.20};
    Eigen::Vector3d barrier_position{0.50, 0.0, 0.40};
};

// Position only, identity rotation.
Eigen::Matrix4d translation(const Eigen::Vector3d& position) {
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
    result.block<3, 1>(0, 3) = position;
    return result;
}

// Position with the gripper pointing straight down.
Eigen::Matrix4d pose(const Eigen::Vector3d& position) {
    Eigen::Matrix4d result = translation(position);
    result.block<3, 3>(0, 0) = Eigen::AngleAxisd(std::numbers::pi, Eigen::Vector3d::UnitX()).toRotationMatrix();
    return result;
}

void throwIfError(const tl::expected<void, std::string>& result, const std::string& action) {
    if (!result) {
        throw std::runtime_error(action + ": " + result.error());
    }
}

void addPlanningWorld(Scene& scene, const World& world) {
    throwIfError(scene.addBoxGeometry("table", "world", Box(world.table_size.x(), world.table_size.y(), world.table_size.z()), translation(world.table_position), {0.52, 0.34, 0.18, 1.0}), "Failed to add planning table");
    throwIfError(scene.addBoxGeometry("barrier", "world", Box(world.barrier_size.x() + 0.08, world.barrier_size.y() + 0.08, world.barrier_size.z() + 0.04), translation(world.barrier_position), {0.20, 0.20, 0.24, 1.0}), "Failed to add planning barrier");
    throwIfError(scene.addBoxGeometry("block", "world", Box(world.block_size.x(), world.block_size.y(), world.block_size.z()), translation(world.pick_position), {0.86, 0.10, 0.08, 1.0}), "Failed to add planning block");
    for (const auto* link : {"fr3_hand", "fr3_leftfinger", "fr3_rightfinger"}) {
        throwIfError(scene.setCollisions("block", link, false), "Failed to disable block collision with " + std::string(link));
    }
    throwIfError(scene.setCollisions("table", "block", false), "Failed to disable table-block collision");
    throwIfError(scene.setCollisions("table", "barrier", false), "Failed to disable table-barrier collision");
}

void setPlanningBlock(Scene& scene, const World& world, const bool attached, const Eigen::Vector3d& world_position = Eigen::Vector3d::Zero()) {
    throwIfError(scene.removeGeometry("block"), "Failed to remove planning block");
    const Eigen::Matrix4d block_pose = attached ? translation({0.0, 0.0, 0.002}) : translation(world_position);
    throwIfError(scene.addBoxGeometry("block", attached ? kTcp : "world", Box(world.block_size.x(), world.block_size.y(), world.block_size.z()), block_pose, {0.86, 0.10, 0.08, 1.0}), "Failed to add planning block");
    for (const auto* link : {"fr3_hand", "fr3_leftfinger", "fr3_rightfinger", "fr3_link6", "fr3_link7"}) {
        throwIfError(scene.setCollisions("block", link, false), "Failed to disable block collision with " + std::string(link));
    }
    if (!attached) {
        throwIfError(scene.setCollisions("table", "block", false), "Failed to disable table-block collision");
    }
}

void addBox(mjsBody& body, const char* name, const Eigen::Vector3d& size, const Eigen::Vector3d& position, const std::array<float, 4>& color, const bool collidable = true) {
    auto* geom = mjs_addGeom(&body, nullptr);
    mjs_setName(geom->element, name);
    geom->type = mjGEOM_BOX;
    for (int i = 0; i < 3; i++) {
        geom->size[i] = 0.5 * size[i];
        geom->pos[i] = position[i];
    }
    std::copy(color.begin(), color.end(), geom->rgba);
    if (!collidable) {
        geom->contype = 0;
        geom->conaffinity = 0;
    }
}

void addMujocoWorld(mjSpec& spec, const World& world) {
    auto* root = mjs_findBody(&spec, "world");
    addBox(*root, "table", world.table_size, world.table_position, {0.52F, 0.34F, 0.18F, 1.0F});
    addBox(*root, "barrier", world.barrier_size, world.barrier_position, {0.20F, 0.20F, 0.24F, 1.0F});

    auto* block = mjs_addBody(root, nullptr);
    mjs_setName(block->element, "block");
    block->mocap = true;
    std::copy(world.pick_position.data(), world.pick_position.data() + 3, block->pos);
    addBox(*block, "block_geom", world.block_size, Eigen::Vector3d::Zero(), {0.86F, 0.10F, 0.08F, 1.0F}, false);
}

JointTrajectory planCartesian(const std::shared_ptr<Scene>& scene, const std::vector<Eigen::Matrix4d>& targets) {
    const Eigen::VectorXd start_q = scene->getCurrentJointPositions();
    std::vector<Eigen::Matrix4d> waypoints{scene->forwardKinematics(start_q, kTcp)};
    waypoints.insert(waypoints.end(), targets.begin(), targets.end());
    const CartesianPath path({"world"}, {kTcp}, {waypoints});

    CartesianPlannerOptions options;
    options.group_name = kGroup;
    options.dt = 0.02;
    options.speed_mode = CartesianSpeedMode::Bounded;
    options.max_position_error = 0.05;
    options.max_orientation_error = 0.03;
    options.config_task_weight = 0.0;
    options.toppra_blend_deviation = 0.1;

    JointConfiguration start;
    start.positions = start_q;
    CartesianPathPlanner planner(scene, options);
    const auto result = planner.plan(path, start);
    if (!result) {
        throw std::runtime_error("Cartesian path planning failed: " + result.error());
    }
    return result.value();
}

void advancePlanningState(Scene& scene, const JointGroupInfo& group, const JointTrajectory& trajectory, Eigen::VectorXd& q, const int finger_index, const double finger_position) {
    q(group.q_indices) = trajectory.positions.back();
    q[finger_index] = finger_position;
    scene.setJointPositions(q);
}

void appendOverlay(const Scene& scene, const JointGroupInfo& group, const JointTrajectory& trajectory, const int finger_index, const double finger_position, MujocoHardwareInterface& hardware, MujocoPlanningOverlay& overlay, const Eigen::VectorXd& reference) {
    for (const auto& group_q : trajectory.positions) {
        Eigen::VectorXd q = reference;
        q(group.q_indices) = group_q;
        q[finger_index] = finger_position;
        overlay.task_space_paths.front().push_back(scene.forwardKinematics(q, kTcp).block<3, 1>(0, 3));
        auto qpos = hardware.toMujocoQpos(q);
        if (!qpos) {
            throw std::runtime_error("Failed to map overlay configuration to MuJoCo: " + qpos.error());
        }
        overlay.robot_configuration_qpos.push_back(std::move(qpos.value()));
    }
}

void stepUntil(MujocoHardwareInterface& hardware, const double end_time, const bool carrying, MujocoRenderer& renderer, const std::chrono::steady_clock::time_point wall_start, double& next_render_time) {
    const auto& model = hardware.simulation().model();
    auto& data = hardware.simulation().data();
    const int tcp_body = mj_name2id(&model, mjOBJ_BODY, kTcp);
    const int mocap_id = model.body_mocapid[mj_name2id(&model, mjOBJ_BODY, "block")];
    const Eigen::Vector3d local_offset(0.0, 0.0, 0.002);

    while (hardware.simulation().time() < end_time) {
        if (carrying) {
            for (int row = 0; row < 3; ++row) {
                data.mocap_pos[3 * mocap_id + row] = data.xpos[3 * tcp_body + row];
                for (int column = 0; column < 3; ++column) {
                    data.mocap_pos[3 * mocap_id + row] += data.xmat[9 * tcp_body + 3 * row + column] * local_offset[column];
                }
            }
            std::copy(data.xquat + 4 * tcp_body, data.xquat + 4 * tcp_body + 4, data.mocap_quat + 4 * mocap_id);
        }
        hardware.simulation().step();
        if (renderer.isOpen() && hardware.simulation().time() >= next_render_time) {
            renderer.renderFrame();
            next_render_time += kRenderPeriod;
            std::this_thread::sleep_until(wall_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(hardware.simulation().time())));
        }
    }
}

void executeTrajectory(MujocoHardwareInterface& hardware, const JointGroupInfo& group, const JointTrajectory& trajectory, Eigen::VectorXd& command, const int finger_index, const double finger_position, const bool carrying, MujocoRenderer& renderer, const std::chrono::steady_clock::time_point wall_start, double& next_render_time) {
    const double start_time = hardware.simulation().time();
    for (std::size_t i = 0; i < trajectory.times.size(); ++i) {
        command(group.q_indices) = trajectory.positions[i];
        command[finger_index] = finger_position;

        // The hardware interface maps this RoboPlan configuration to the corresponding MuJoCo position actuators.
        throwIfError(hardware.writePositionCommand(command), "Failed to write trajectory command");

        // Applications own the control loop: advance MuJoCo until the next trajectory sample while rendering at a lower rate.
        stepUntil(hardware, start_time + trajectory.times[i], carrying, renderer, wall_start, next_render_time);
    }
}

void moveGripper(MujocoHardwareInterface& hardware, Eigen::VectorXd& command, const int finger_index, const double finger_position, MujocoRenderer& renderer, const std::chrono::steady_clock::time_point wall_start, double& next_render_time) {
    command[finger_index] = finger_position;
    throwIfError(hardware.writePositionCommand(command), "Failed to write gripper command");
    stepUntil(hardware, hardware.simulation().time() + 0.4, false, renderer, wall_start, next_render_time);
}

}  // namespace

int main() {
    const auto models_dir = example_models::get_package_models_dir();
    const auto mujoco_model_dir = models_dir / "mujoco" / "franka";
    const auto source_model_dir = models_dir / "franka_robot_model";
    const auto urdf = mujoco_model_dir / "fr3.urdf";
    const auto srdf = source_model_dir / "fr3.srdf";

    // RoboPlan owns the planning representation: robot kinematics, semantic groups, collision geometry, and the current planning state.
    auto scene = std::make_shared<Scene>("pick_and_place", urdf, srdf,std::vector<std::filesystem::path>{example_models::get_package_share_dir(), mujoco_model_dir},source_model_dir / "fr3_config.yaml");

    const auto group = scene->getJointGroupInfo(kGroup).value();
    const int finger_index = scene->getJointPositionIndices({"fr3_finger_joint1"})[0];
    const World world;
    addPlanningWorld(*scene, world);

    Eigen::VectorXd home = scene->getCurrentJointPositions();
    home(group.q_indices) << 0.8, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398;
    home[finger_index] = kOpenGripper;
    scene->setJointPositions(home);
    Eigen::VectorXd planning_q = home;

    // Plan the pick and place motion with RoboPlan before creating the simulator. The resulting trajectories contain RoboPlan joint vectors.
    std::cout << "Planning home -> pick -> place..." << std::endl;
    const auto approach = planCartesian(scene, {pose({world.pick_position.x(), world.pick_position.y(), 0.44}), pose(world.pick_position + Eigen::Vector3d(0.0, 0.0, 0.002))});
    advancePlanningState(*scene, group, approach, planning_q, finger_index, kOpenGripper);

    setPlanningBlock(*scene, world, true);
    const auto carry = planCartesian(scene, {pose({world.pick_position.x(), world.pick_position.y(), 0.62}), pose({world.place_position.x(), world.place_position.y(), 0.62}), pose(world.place_position + Eigen::Vector3d(0.0, 0.0, 0.008))});

    // Import the robot URDF into an editable MuJoCo model and attach it to the example MJCF scene.
    auto builder = MujocoModelBuilder::fromUrdf(urdf, example_models::get_package_models_dir() / "mujoco" / "scene.xml").value();
    builder.spec().option.timestep = 0.001;
    builder.spec().option.integrator = mjINT_IMPLICITFAST;
    throwIfError(builder.addCollisionExclusionsFromSrdf(srdf), "Failed to add collision exclusions");

    // Add MuJoCo position servos only for joints commanded by this example.
    auto controlled_joints = group.joint_names;
    controlled_joints.emplace_back("fr3_finger_joint1");
    throwIfError(builder.addPositionServos(controlled_joints, {.kp = 4000.0, .kv = 300.0, .max_force = 100.0}), "Failed to add position servos");

    // The simulation world is MuJoCo specific and remains separate from the collision geometry used by RoboPlan during planning.
    addMujocoWorld(builder.spec(), world);

    // Compile the editable model and transfer ownership of the simulation to the hardware interface. The interface also builds the joint mapping between RoboPlan vectors and MuJoCo qpos, qvel, and actuator arrays.
    auto hardware = MujocoHardwareInterface::create(builder.compile().value(), *scene, controlled_joints).value();

    // Initialize MuJoCo from the same full configuration used by RoboPlan.
    throwIfError(hardware.reset(home), "Failed to reset MuJoCo hardware interface");
    Eigen::VectorXd command = home;

    // The renderer observes the simulation owned by the hardware interface.
    auto renderer = MujocoRenderer::create(hardware.simulation(), "RoboPlan MuJoCo pick and place").value();
    MujocoPlanningOverlay overlay;
    overlay.ghost_root_bodies = {kRootBody};
    overlay.task_space_paths.emplace_back();
    appendOverlay(*scene, group, approach, finger_index, kOpenGripper, hardware, overlay, home);
    appendOverlay(*scene, group, carry, finger_index, kClosedGripper, hardware, overlay, home);
    renderer->setPlanningOverlay(std::move(overlay));

    const auto wall_start = std::chrono::steady_clock::now();
    double next_render_time = 0.0;
    executeTrajectory(hardware, group, approach, command, finger_index, kOpenGripper, false, *renderer, wall_start, next_render_time);
    moveGripper(hardware, command, finger_index, kClosedGripper, *renderer, wall_start, next_render_time);
    executeTrajectory(hardware, group, carry, command, finger_index, kClosedGripper, true, *renderer, wall_start, next_render_time);
    moveGripper(hardware, command, finger_index, kOpenGripper, *renderer, wall_start, next_render_time);

    std::cout << "Pick and place example complete." << std::endl;
    while (renderer->isOpen()) {
        renderer->renderFrame();
    }
    return 0;
}