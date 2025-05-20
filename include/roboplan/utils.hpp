#pragma once

#include <iostream>
#include <string>

namespace roboplan {

/// @brief Adds two numbers
/// @param a The first number
/// @param b The second number
/// @return The sum of the two numbers
int add(int a, int b);

/// @brief Creates a dummy Pinocchio model. 
void createPinocchioModel(const std::string& urdf_path, const std::string& srdf_path);

}  // namespace roboplan
