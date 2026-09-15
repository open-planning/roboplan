#include <algorithm>
#include <array>
#include <cassert>
#include <cstdio>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <roboplan_mujoco/mujoco_renderer.hpp>

namespace roboplan {

namespace {
// GLFW initialization is process-global. Reference counting lets multiple renderers share
// it without one renderer terminating GLFW while another is still alive.
std::mutex glfw_mutex;
std::size_t glfw_users = 0;

bool acquireGlfw() {
  const std::scoped_lock lock(glfw_mutex);
  if (glfw_users == 0 && glfwInit() == GLFW_FALSE) {
    return false;
  }
  ++glfw_users;
  return true;
}

void releaseGLFW() {
  const std::scoped_lock lock(glfw_mutex);
  assert(glfw_users > 0);
  --glfw_users;
  if (glfw_users == 0) {
    glfwTerminate();
  }
}
}  // namespace

struct MujocoRenderer::Impl {
  const mjModel* model;
  mjData* data;
  GLFWwindow* window;
  mjvCamera camera;
  mjvOption option;
  mjvScene scene;
  mjrContext context;
  MujocoDataPtr ghost_data;
  MujocoPlanningOverlay overlay;
  bool show_paths{true};
  bool show_robot_configuration_poses{false};
  std::vector<int> ghost_root_bodies;
  double last_cursor_x{0.0};
  double last_cursor_y{0.0};

  Impl(MujocoSimulation& simulation, GLFWwindow* window)
      : model(&simulation.model()), data(&simulation.data()), window(window) {
    assert(model != nullptr);
    assert(data != nullptr);
    assert(window != nullptr);

    mjv_defaultCamera(&camera);
    mjv_defaultOption(&option);
    mjv_defaultScene(&scene);
    mjr_defaultContext(&context);

    // Reserve space for the live model, path segments, and up to 20 ghost robot poses.
    const int max_geoms = static_cast<int>(std::clamp<mjtSize>(model->ngeom * 25 + 1024, 4096, 12000));
    mjv_makeScene(model, &scene, max_geoms);
    mjr_makeContext(model, &context, mjFONTSCALE_100);
    ghost_data.reset(mj_makeData(model));
    assert(ghost_data != nullptr);

    glfwSetWindowUserPointer(window, this);
    glfwSetKeyCallback(
        window, [](GLFWwindow* callback_window, const int key, int, const int action, const int) {
          if (action != GLFW_PRESS) {
            return;
          }

          auto* renderer = static_cast<Impl*>(glfwGetWindowUserPointer(callback_window));
          assert(renderer != nullptr);
          switch (key) {
          case GLFW_KEY_P:
            renderer->show_paths = !renderer->show_paths;
            break;
          case GLFW_KEY_T:
            renderer->show_robot_configuration_poses = !renderer->show_robot_configuration_poses;
            break;
          case GLFW_KEY_ESCAPE:
            glfwSetWindowShouldClose(callback_window, GLFW_TRUE);
            break;
          default:
            break;
          }
        });

    glfwSetMouseButtonCallback(
        window, [](GLFWwindow* callback_window, const int, const int action, const int) {
          if (action != GLFW_PRESS) {
            return;
          }
          auto* renderer = static_cast<Impl*>(glfwGetWindowUserPointer(callback_window));
          assert(renderer != nullptr);
          glfwGetCursorPos(callback_window, &renderer->last_cursor_x, &renderer->last_cursor_y);
        });

    glfwSetCursorPosCallback(window, [](GLFWwindow* callback_window, const double xpos, const double ypos) {
      auto* renderer = static_cast<Impl*>(glfwGetWindowUserPointer(callback_window));
      assert(renderer != nullptr);
      const double dx = xpos - renderer->last_cursor_x;
      const double dy = ypos - renderer->last_cursor_y;
      renderer->last_cursor_x = xpos;
      renderer->last_cursor_y = ypos;

      const bool left = glfwGetMouseButton(callback_window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
      const bool middle = glfwGetMouseButton(callback_window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
      const bool right = glfwGetMouseButton(callback_window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
      if (!left && !middle && !right) {
        return;
      }
      const bool shift = glfwGetKey(callback_window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || glfwGetKey(callback_window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
      int action = mjMOUSE_ZOOM;
      if (right) {
        action = shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
      } else if (left) {
        action = shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
      }
      int height = 0;
      glfwGetWindowSize(callback_window, nullptr, &height);
      const double viewport_height = std::max(1, height);
      mjv_moveCamera(renderer->model, action, dx / viewport_height, dy / viewport_height, &renderer->camera);
    });

    glfwSetScrollCallback(
        window, [](GLFWwindow* callback_window, const double, const double yoffset) {
          auto* renderer = static_cast<Impl*>(glfwGetWindowUserPointer(callback_window));
          assert(renderer != nullptr);
          mjv_moveCamera(renderer->model, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, &renderer->camera);
        });
  }

  ~Impl() {
    glfwMakeContextCurrent(window);
    mjr_freeContext(&context);
    mjv_freeScene(&scene);
    if (window != nullptr) {
      glfwDestroyWindow(window);
    }
    releaseGLFW();
  }

  bool isRobotBody(int body_id) const {
    // A ghost pose includes only geometry below one of the caller selected robot roots,
    // excluding world geometry copied into the temporary visualization scene.
    while (body_id > 0) {
      for (const int root_body : ghost_root_bodies) {
        if (body_id == root_body) {
          return true;
        }
      }
      body_id = model->body_parentid[body_id];
    }
    return false;
  }

  void appendPaths() {
    constexpr std::array<float, 4> rgba = {0.1f, 0.75f, 1.0f, 0.9f};
    constexpr mjtNum width = 0.004;
    for (const auto& path : overlay.task_space_paths) {
      for (std::size_t i = 1; i < path.size(); ++i) {
        if (scene.ngeom >= scene.maxgeom) {
          return;
        }
        const std::array<mjtNum, 3> from = {path[i - 1].x(), path[i - 1].y(), path[i - 1].z()};
        const std::array<mjtNum, 3> to = {path[i].x(), path[i].y(), path[i].z()};
        auto* geom = &scene.geoms[scene.ngeom++];
        mjv_initGeom(geom, mjGEOM_CAPSULE, nullptr, nullptr, nullptr, rgba.data());
        mjv_connector(geom, mjGEOM_CAPSULE, width, from.data(), to.data());
        geom->objtype = mjOBJ_UNKNOWN;
        geom->objid = -1;
        geom->category = mjCAT_DECOR;
      }
    }
  }

  void appendRobotConfigurationPoses() {
    if (ghost_root_bodies.empty() || overlay.robot_configuration_qpos.empty()) {
      return;
    }

    // Uniformly subsample long paths to keep the overlay bounded and readable.
    constexpr std::size_t kMaxRobotPoses = 20;
    const std::size_t pose_count = std::min(kMaxRobotPoses, overlay.robot_configuration_qpos.size());
    for (std::size_t pose_index = 0; pose_index < pose_count; ++pose_index) {
      if (scene.ngeom + model->ngeom >= scene.maxgeom) {
        return;
      }
      const std::size_t configuration_index = pose_count == 1 ? 0 : pose_index * (overlay.robot_configuration_qpos.size() - 1) / (pose_count - 1);
      const auto& qpos = overlay.robot_configuration_qpos[configuration_index];
      if (qpos.size() != model->nq) {
        continue;
      }

      mjv_copyData(ghost_data.get(), model, data);
      Eigen::Map<Eigen::VectorXd>(ghost_data->qpos, model->nq) = qpos;
      mj_kinematics(model, ghost_data.get());

      const int first_ghost_geom = scene.ngeom;
      mjv_addGeoms(model, ghost_data.get(), &option, nullptr, mjCAT_DYNAMIC, &scene);
      int output_geom = first_ghost_geom;
      for (int input_geom = first_ghost_geom; input_geom < scene.ngeom; ++input_geom) {
        mjvGeom& geom = scene.geoms[input_geom];
        const bool keep = geom.objtype == mjOBJ_GEOM && geom.objid >= 0 && isRobotBody(model->geom_bodyid[geom.objid]);

        if (!keep) {
          continue;
        }

        geom.rgba[0] = 1.0f;
        geom.rgba[1] = 0.45f;
        geom.rgba[2] = 0.1f;
        geom.rgba[3] = 0.15f;
        geom.transparent = 1;
        scene.geoms[output_geom++] = geom;
      }
      scene.ngeom = output_geom;
    }
  }
};

MujocoRenderer::MujocoRenderer(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {
  assert(impl_ != nullptr);
}

MujocoRenderer::~MujocoRenderer() = default;

tl::expected<std::unique_ptr<MujocoRenderer>, std::string> MujocoRenderer::create(MujocoSimulation& simulation, const std::string& title) {
  if (!acquireGlfw()) {
    return tl::unexpected("Failed to initialize GLFW");
  }

  GLFWwindow* window = glfwCreateWindow(1280, 720, title.c_str(), nullptr, nullptr);
  if (window == nullptr) {
    releaseGLFW();
    return tl::unexpected("Failed to create Mujoco render window");
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  return std::unique_ptr<MujocoRenderer>(new MujocoRenderer(std::make_unique<Impl>(simulation, window)));
}

bool MujocoRenderer::isOpen() const {
  assert(impl_ != nullptr);
  return glfwWindowShouldClose(impl_->window) == GLFW_FALSE;
}

void MujocoRenderer::renderFrame() {
  assert(impl_ != nullptr);
  glfwMakeContextCurrent(impl_->window);

  int width = 0;
  int height = 0;
  glfwGetFramebufferSize(impl_->window, &width, &height);
  const mjrRect viewport = {0, 0, width, height};

  mjv_updateScene(impl_->model, impl_->data, &impl_->option, nullptr, &impl_->camera, mjCAT_ALL, &impl_->scene);

  if (impl_->show_paths) {
    impl_->appendPaths();
  }

  if (impl_->show_robot_configuration_poses) {
    impl_->appendRobotConfigurationPoses();
  }

  mjr_render(viewport, &impl_->scene, &impl_->context);

  std::array<char, 256> overlay_state{};
  std::snprintf(overlay_state.data(), overlay_state.size(),"Press P to toggle path visualization [%s]\nPress T to toggle robot configuration poses [%s]", impl_->show_paths ? "on" : "off", impl_->show_robot_configuration_poses ? "on" : "off");
  mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, viewport, "Visualization controls", overlay_state.data(), &impl_->context);

  glfwSwapBuffers(impl_->window);
  glfwPollEvents();
}

void MujocoRenderer::setPlanningOverlay(MujocoPlanningOverlay overlay) {
  assert(impl_ != nullptr);
  impl_->overlay = std::move(overlay);
  impl_->ghost_root_bodies.clear();
  for (const auto& root_body : impl_->overlay.ghost_root_bodies) {
    const int body_id = mj_name2id(impl_->model, mjOBJ_BODY, root_body.c_str());
    if (body_id >= 0) {
      impl_->ghost_root_bodies.push_back(body_id);
    }
  }
}

}  // namespace roboplan