#include <array>
#include <cassert>
#include <cmath>
#include <filesystem>
#include <unordered_set>
#include <utility>

#include <tinyxml2.h>

#include <roboplan_mujoco/mujoco_model_builder.hpp>

namespace roboplan {
namespace {
constexpr int kErrorBufferSize = 2048;
constexpr int kUrdfCollisionGroup = 0;

/// @brief Hides duplicate collision geometry while preserving its physics behavior.
/// @details The MuJoCo URDF importer leaves collision geometry in group 0 and assigns
/// visual geometry to group 1. Setting only the alpha channel removes collision geometry
/// from rendering without changing its contact properties.
/// @see
/// https://github.com/google-deepmind/mujoco/blob/b62c3e886adfcfe220a694408ca8a41cee50b976/src/xml/xml_urdf.cc#L508
void hideUrdfCollisionGeometry(mjSpec& spec) {
  for (auto* element = mjs_firstElement(&spec, mjOBJ_GEOM); element != nullptr; element = mjs_nextElement(&spec, element)) {
    auto* geom = mjs_asGeom(element);
    if (geom != nullptr && geom->group == kUrdfCollisionGroup) {
      geom->rgba[3] = 0.0f;
    }
  }
}

/// @brief Returns whether a joint already has a directly attached actuator.
bool hasDirectActuator(const mjSpec& spec, const char* joint_name) {
  for (auto* element = mjs_firstElement(&spec, mjOBJ_ACTUATOR); element != nullptr; element = mjs_nextElement(&spec, element)) {
    const auto* actuator = mjs_asActuator(element);
    const char* target = actuator == nullptr ? nullptr : mjs_getString(actuator->target);
    if (actuator != nullptr && actuator->trntype == mjTRN_JOINT && target != nullptr &&
        std::strcmp(target, joint_name) == 0) {
      return true;
    }
  }
  return false;
}

/// @brief Recreates URDF mimic relationships as MuJoCo joint equality constraints.
tl::expected<void, std::string> addUrdfMimicConstraints(const tinyxml2::XMLElement& robot, mjSpec& spec) {
  for (const auto* joint = robot.FirstChildElement("joint"); joint != nullptr; joint = joint->NextSiblingElement("joint")) {
    const auto* mimic = joint->FirstChildElement("mimic");
    if (mimic == nullptr) {
      continue;
    }
    const char* joint_name = joint->Attribute("name");
    const char* source_name = mimic->Attribute("joint");
    if (joint_name == nullptr || source_name == nullptr) {
      return tl::unexpected("URDF Mimic joint is missing a name or source joint");
    }
    if (mjs_findElement(&spec, mjOBJ_JOINT, joint_name) == nullptr || mjs_findElement(&spec, mjOBJ_JOINT, source_name) == nullptr) {
      return tl::unexpected("URDF mimic relationship references a missing Mujoco joint: " + std::string(joint_name) + ", " + std::string(source_name));
    }

    double multiplier = 1.0;
    double offset = 0.0;
    mimic->QueryDoubleAttribute("multiplier", &multiplier);
    mimic->QueryDoubleAttribute("offset", &offset);

    auto* equality = mjs_addEquality(&spec, nullptr);
    if (equality == nullptr) {
      return tl::unexpected("Failed to add Mujoco mimic constraint for joint: " + std::string(joint_name));
    }
    const std::string equality_name = std::string(joint_name) + "_mimic";
    if (mjs_setName(equality->element, equality_name.c_str()) != 0) {
      return tl::unexpected("Failed to name Mujoco mimic constraint for joint: " + std::string(joint_name));
    }
    equality->type = mjEQ_JOINT;
    equality->objtype = mjOBJ_JOINT;
    mjs_setString(equality->name1, joint_name);
    mjs_setString(equality->name2, source_name);
    equality->data[0] = offset;
    equality->data[1] = multiplier;
    equality->active = true;
  }
  return {};
}
}  // namespace

MujocoModelBuilder::MujocoModelBuilder(MujocoSpecPtr spec) : spec_(std::move(spec)) {
  assert(this->spec_ != nullptr);
}

tl::expected<MujocoModelBuilder, std::string> MujocoModelBuilder::fromUrdf(const std::filesystem::path& urdf_path) {
  if (!std::filesystem::is_regular_file(urdf_path)) {
    return tl::unexpected("URDF file does not exist: " + urdf_path.string());
  }

  tinyxml2::XMLDocument document;
  if (document.LoadFile(urdf_path.string().c_str()) != tinyxml2::XML_SUCCESS) {
    return tl::unexpected("Failed to parse URDF XML: " + std::string(document.ErrorStr()));
  }

  auto* robot = document.FirstChildElement("robot");
  if (robot == nullptr) {
    return tl::unexpected("URDF has no <robot> root element");
  }

  auto* mujoco = robot->FirstChildElement("mujoco");
  if (mujoco == nullptr) {
    mujoco = document.NewElement("mujoco");
    robot->InsertEndChild(mujoco);
  }

  auto* compiler = mujoco->FirstChildElement("compiler");
  if (compiler == nullptr) {
    compiler = document.NewElement("compiler");
    mujoco->InsertEndChild(compiler);
  }

  compiler->SetAttribute("discardvisual", false);
  compiler->SetAttribute("fusestatic", false);
  const auto source_directory = std::filesystem::absolute(urdf_path).parent_path();
  compiler->SetAttribute("meshdir", source_directory.string().c_str());

  tinyxml2::XMLPrinter printer;
  document.Print(&printer);
  std::array<char, kErrorBufferSize> error_buffer;
  MujocoSpecPtr spec(mj_parseXMLString(printer.CStr(), nullptr, error_buffer.data(), error_buffer.size()));
  if (spec == nullptr) {
    return tl::unexpected(error_buffer[0] != '\0' ? error_buffer.data() : "Failed to import URDF into Mujoco");
  }
  if (const auto mimic_result = addUrdfMimicConstraints(*robot, *spec); !mimic_result) {
    return tl::unexpected(mimic_result.error());
  }
  hideUrdfCollisionGeometry(*spec);
  return MujocoModelBuilder(std::move(spec));
}

tl::expected<MujocoModelBuilder, std::string> MujocoModelBuilder::fromUrdf(const std::filesystem::path& urdf_path, const std::filesystem::path& scene_mjcf_path) {
  auto robot_builder = fromUrdf(urdf_path);
  if (!robot_builder) {
    return tl::unexpected(robot_builder.error());
  }

  if (!std::filesystem::is_regular_file(scene_mjcf_path)) {
    return tl::unexpected("Scene MJCF file does not exist: " + scene_mjcf_path.string());
  }

  std::array<char, kErrorBufferSize> error_buffer{};
  MujocoSpecPtr scene_spec(mj_parseXML(scene_mjcf_path.string().c_str(), nullptr, error_buffer.data(), error_buffer.size()));
  if (scene_spec == nullptr) {
    return tl::unexpected(error_buffer[0] != '\0' ? error_buffer.data() : "Failed to import MJCF Scene file");
  }

  auto* world = mjs_findBody(scene_spec.get(), "world");
  if (world == nullptr) {
    return tl::unexpected("MJCF Scene file has no world body");
  }

  // Attachment must deep copy the robot because robot_builder is destroyed on return.
  if (mjs_setDeepCopy(scene_spec.get(), 1) != 0) {
    return tl::unexpected("Failed to enable deepcopy attachment for the MJCF Scene");
  }

  if (mjs_attach(world->element, robot_builder->spec().element, "", "") == nullptr) {
    const char* attach_error = mjs_getError(scene_spec.get());
    return tl::unexpected(attach_error != nullptr && attach_error[0] != '\0' ? attach_error : "Failed to attach robot to MJCF Scene");
  }

  return MujocoModelBuilder(std::move(scene_spec));
}

mjSpec& MujocoModelBuilder::spec() {
  assert(spec_ != nullptr);
  return *spec_;
}

tl::expected<void, std::string> MujocoModelBuilder::addPositionServos(const std::vector<std::string>& joint_names, const MujocoPositionActuatorOptions& options) {
  if (!std::isfinite(options.kp) || options.kp <= 0.0 || !std::isfinite(options.kv) || options.kv < 0.0 || !std::isfinite(options.max_force) || options.max_force <= 0.0) {
    return tl::unexpected("Position actuator gains must satisfy kp > 0.0 and kv >= 0.0 and max_force > 0.0");
  }
  std::vector<std::string> expanded_joint_names;
  std::unordered_set<std::string> unique_joint_names;
  for (const auto& joint_name : joint_names) {
    if (mjs_findElement(spec_.get(), mjOBJ_JOINT, joint_name.c_str()) != nullptr) {
      expanded_joint_names.push_back(joint_name);
    } else {
      // Prepared planar joints use three scalar MuJoCo coordinates.
      for (const char* suffix : {"_TX", "_TY", "_RZ"}) {
        const std::string split_name = joint_name + suffix;
        if (mjs_findElement(spec_.get(), mjOBJ_JOINT, split_name.c_str()) == nullptr) {
          return tl::unexpected("Mujoco joint does not exist: " + joint_name);
        }
        expanded_joint_names.push_back(split_name);
      }
    }
  }

  for (const auto& mujoco_joint_name : expanded_joint_names) {
    if (!unique_joint_names.insert(mujoco_joint_name).second) {
      return tl::unexpected("Mujoco joint is repeated: " + mujoco_joint_name);
    }
    auto* element = mjs_findElement(spec_.get(), mjOBJ_JOINT, mujoco_joint_name.c_str());
    auto* joint = element == nullptr ? nullptr : mjs_asJoint(element);
    if (joint == nullptr || (joint->type != mjJNT_HINGE && joint->type != mjJNT_SLIDE)) {
      return tl::unexpected("Mujoco position actuators require scalar joints: " + mujoco_joint_name);
    }
    if (hasDirectActuator(*spec_, mujoco_joint_name.c_str())) {
      return tl::unexpected("Mujoco joint already has a direct actuator: " + mujoco_joint_name);
    }
  }

  for (const auto& mujoco_joint_name : expanded_joint_names) {
    auto* joint = mjs_asJoint(mjs_findElement(spec_.get(), mjOBJ_JOINT, mujoco_joint_name.c_str()));
    auto* actuator = mjs_addActuator(spec_.get(), nullptr);
    if (actuator == nullptr) {
      return tl::unexpected("Failed to add Mujoco actuator for joint: " + mujoco_joint_name);
    }
    const std::string actuator_name = mujoco_joint_name + "_position";
    if (mjs_setName(actuator->element, actuator_name.c_str()) != 0) {
      return tl::unexpected("Failed to configure Mujoco actuator for joint: " + mujoco_joint_name);
    }
    mjs_setString(actuator->target, mujoco_joint_name.c_str());
    actuator->trntype = mjTRN_JOINT;
    actuator->gaintype = mjGAIN_FIXED;
    actuator->gainprm[0] = options.kp;
    actuator->biastype = mjBIAS_AFFINE;
    actuator->biasprm[1] = -options.kp;
    actuator->biasprm[2] = -options.kv;
    actuator->gear[0] = 1.0;
    actuator->forcelimited = mjLIMITED_TRUE;
    actuator->forcerange[0] = -options.max_force;
    actuator->forcerange[1] = options.max_force;
    if (joint->limited != mjLIMITED_FALSE && joint->range[0] < joint->range[1]) {
      actuator->ctrllimited = mjLIMITED_TRUE;
      actuator->ctrlrange[0] = joint->range[0];
      actuator->ctrlrange[1] = joint->range[1];
    }
  }

  return {};
}

tl::expected<void, std::string> MujocoModelBuilder::addCollisionExclusionsFromSrdf(const std::filesystem::path& srdf_path) {
  if (!std::filesystem::is_regular_file(srdf_path)) {
    return tl::unexpected("SRDF file does not exist: " + srdf_path.string());
  }

  tinyxml2::XMLDocument document;
  if (document.LoadFile(srdf_path.string().c_str()) != tinyxml2::XML_SUCCESS) {
    return tl::unexpected("Failed to parse SRDF XML: " + std::string(document.ErrorStr()));
  }

  const auto* robot = document.FirstChildElement("robot");
  if (robot == nullptr) {
    return tl::unexpected("SRDF has no <robot> root element");
  }

  for (const auto* disabled = robot->FirstChildElement("disable_collisions"); disabled != nullptr; disabled = disabled->NextSiblingElement("disable_collisions")) {
    const char* first = disabled->Attribute("link1");
    const char* second = disabled->Attribute("link2");
    if (first == nullptr || second == nullptr) {
      return tl::unexpected("SRDF disable_collisions element is missing a link1 or link2 attribute");
    }
    if (mjs_findBody(spec_.get(), first) == nullptr || mjs_findBody(spec_.get(), second) == nullptr) {
      return tl::unexpected("SRDF collision exclusion references a missing Mujoco body: " + std::string(first) + ", " + std::string(second));
    }
    auto* exclusion = mjs_addExclude(spec_.get());
    if (exclusion == nullptr) {
      return tl::unexpected("Failed to add Mujoco collision exclusion: " + std::string(first) + ", " + std::string(second));
    }
    mjs_setString(exclusion->bodyname1, first);
    mjs_setString(exclusion->bodyname2, second);
  }
  return {};
}

tl::expected<MujocoSimulation, std::string> MujocoModelBuilder::compile() {
  MujocoModelPtr model(mj_compile(spec_.get(), nullptr));
  if (model == nullptr) {
    const char* error = mjs_getError(spec_.get());
    return tl::unexpected(error != nullptr && error[0] != '\0' ? error : "Failed to compile Mujoco model");
  }

  MujocoDataPtr data(mj_makeData(model.get()));
  if (data == nullptr) {
    return tl::unexpected("Failed to allocate Mujoco Data object");
  }

  mj_forward(model.get(), data.get());
  return MujocoSimulation(std::move(model), std::move(data));
}

}  // namespace roboplan