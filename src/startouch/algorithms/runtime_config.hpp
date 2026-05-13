#pragma once

#include <string>

namespace startouch::algorithms {

std::string resolve_robot_kinematics_config_path();
std::string resolve_robot_urdf_path();
bool motion_planning_debug_enabled();
bool ik_diagnostics_debug_enabled();

}  // namespace startouch::algorithms
