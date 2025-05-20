#include <iostream>

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

#include <roboplan/utils.hpp>

namespace roboplan {

NB_MODULE(roboplan, m) {
    m.def("add", &add);
    m.def("createPinocchioModel", &createPinocchioModel);
}

}  // namespace roboplan
