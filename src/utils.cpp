#include <vector>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/srdf.hpp"

#include <roboplan/utils.hpp>

namespace roboplan {

int add(int a, int b) { 
    std::cout << "Adding " << a << " and " << b << ".\n";
    return a + b;
}

void createPinocchioModel(const std::string& urdf_path, const std::string& srdf_path) {

    const std::vector<std::string> package_paths{"/home/sebastian/workspace/roboplan_ws/src/roboplan"};

    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_path, model);

    pinocchio::GeometryModel visual_model;
    pinocchio::urdf::buildGeom(model, urdf_path, pinocchio::VISUAL, visual_model, package_paths);

    pinocchio::GeometryModel collision_model;
    pinocchio::urdf::buildGeom(model, urdf_path, pinocchio::COLLISION, collision_model, package_paths);
    collision_model.addAllCollisionPairs();
    pinocchio::srdf::removeCollisionPairs(model, collision_model, srdf_path);

    std::cout << "Built a Pinocchio model with " << std::to_string(model.nq) << " DOFs\n";
    return;
}

} // namespace roboplan
