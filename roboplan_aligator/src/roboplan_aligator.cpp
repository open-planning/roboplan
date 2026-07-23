#include <roboplan_aligator/roboplan_aligator.hpp>

#ifndef ROBOPLAN_ALIGATOR_VERSION
#define ROBOPLAN_ALIGATOR_VERSION "unknown"
#endif

namespace roboplan {

std::string aligatorVersion() { return ROBOPLAN_ALIGATOR_VERSION; }

}  // namespace roboplan
