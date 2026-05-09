#include <pybind11/pybind11.h>
#include <pybind11/stl.h>  // 支持 std::vector 到 Python list 的转换
#include <pybind11/eigen.h>  // 支持 Eigen 矩阵到 Python 的转换
#include <pybind11/functional.h>
// #include "startouch/algorithms/interfaces_simpletest_wopin.hpp" 
#include "startouch/algorithms/interfaces_0825fasttest.hpp" 

namespace py = pybind11;
PYBIND11_MODULE(startouch, m) {
    py::class_<ArmController>(m, "ArmController")
        .def(py::init([](py::kwargs kwargs) {
        //std::string permutation_matrix = kwargs.contains("permutation_matrix") ? kwargs["permutation_matrix"].cast<std::string>() : "/media/sdk/data/aproject/Xrobot/Nxyarm/openarm_can/param_csv/permutationMatrix.csv";
        //std::string pi_b = kwargs.contains("pi_b") ? kwargs["pi_b"].cast<std::string>() : "/media/sdk/data/aproject/Xrobot/Nxyarm/openarm_can/param_csv/pi_b.csv";
        //std::string pi_fr = kwargs.contains("pi_fr") ? kwargs["pi_fr"].cast<std::string>() : "/media/sdk/data/aproject/Xrobot/Nxyarm/openarm_can/param_csv/pi_fr.csv";   
        bool gripper_exist = kwargs.contains("gripper_exist") ? kwargs["gripper_exist"].cast<bool>() : false;
        std::string can_interface = kwargs.contains("can_interface") ? kwargs["can_interface"].cast<std::string>() : "can0";
            bool enable_fd = kwargs.contains("enable_fd") ? kwargs["enable_fd"].cast<bool>() : false;
            return new ArmController(gripper_exist,can_interface, enable_fd);
        }))

        // .def("printConfig", &ArmController::printConfig,"A function which queries the initial params configuration of the arm controller.")
        .def("set_joint", &ArmController::set_joint, 
            "Set the target joint positions with a given time duration.\n"
            "Parameters:\n"
            "  q_end (list of float): The target joint angles.\n"
            "  tf (float): The time duration to reach the target positions.",
            py::arg("q_end"), py::arg("tf") = 4.0)

        .def("set_joint_raw", &ArmController::set_joint_raw,py::arg("q_end"),py::arg("v_end"))

        .def("move_joint_waypoints", &ArmController::move_joint_waypoints,
            "Move through joint waypoints with YAML-configured trajectory limits.\n"
            "speed_percent in (0, 1] overrides time_sec. If neither is provided, "
            "the YAML default_speed_percent is used.",
            py::arg("waypoints"),
            py::arg("time_sec") = 0.0,
            py::arg("speed_percent") = -1.0,
            py::arg("control_hz") = 400.0)

        .def("gravity_compensation", &ArmController::identify_gravity_compensation,"identify_gravity_compensation function.")        
        
        .def("set_end_effector_pose", &ArmController::set_end_effector_pose, 
            "Set the end effector's target pose.\n"
            "\n"
            "This function sets the target position and orientation (pose) for the end effector. "
            "The target position is specified by the `target_pos` vector, and the target orientation "
            "is given by the Euler angles `target_euler`. The `tf` parameter defines the time to reach "
            "the target pose, with a default value of 4.0 seconds.\n"
            "\n"
            "Parameters:\n"
            "  target_pos (Eigen::Vector3d): Target position as a 3D vector (x, y, z).\n"
            "  target_euler (Eigen::Vector3d): Target orientation as Euler angles (roll, pitch, yaw).\n"
            "  tf (float, optional): Time to reach the target pose, in seconds. Default is 4.0.\n"
            , py::arg("target_pos"), py::arg("target_euler"), py::arg("tf") = 4.0)
        

        .def("set_end_effector_pose_raw", &ArmController::set_end_effector_pose_raw, 
            "Set the end effector's target pose.\n"
            "\n"
            "This function sets the target position and orientation (pose) for the end effector. "
            "The target position is specified by the `target_pos` vector, and the target orientation "
            "is given by the Euler angles `target_euler`. The `tf` parameter defines the time to reach "
            "\n"
            "Parameters:\n"
            "  target_pos (Eigen::Vector3d): Target position as a 3D vector (x, y, z).\n"
            "  target_euler (Eigen::Vector3d): Target orientation as Euler angles (roll, pitch, yaw).\n"
            , py::arg("target_pos"), py::arg("target_euler"))

        .def("solve_ik", &ArmController::solve_ik,
            "Solve IK for target end-effector pose.\n"
            "\n"
            "Parameters:\n"
            "  target_pos (list[float]): Target position [x, y, z].\n"
            "  target_euler (list[float]): Target euler [roll, pitch, yaw].\n"
            "  q_seed (list[float], optional): IK initial seed, length=6. If empty, uses current joint.\n"
            "\n"
            "Returns:\n"
            "  tuple[list[float], bool]: (q_sol, success)\n",
            py::arg("target_pos"),
            py::arg("target_euler"),
            py::arg("q_seed") = std::vector<double>{})
            
            

        .def("get_end_effector_pose", &ArmController::get_end_effector_pose)
        .def("get_joint_positions", &ArmController::get_joint)
        .def("get_joint_velocities", &ArmController::get_joint_velocities)
        .def("get_joint_torques", &ArmController::get_joint_torques)
        .def("openGripper", &ArmController::openGripper)
        .def("closeGripper", &ArmController::closeGripper)
        .def("setGripperPosition_raw", &ArmController::setGripperPosition_raw,py::arg("position") )
        .def("setGripperPosition", &ArmController::setGripperPosition,py::arg("position") )
        .def("setGripperDistance",
             py::overload_cast<double, double, double>(&ArmController::setGripperDistance),
             py::arg("distance"), py::arg("kp") = 8.0, py::arg("kd") = 0.1)
        .def("get_gripper_position", &ArmController::get_gripper_position)
        .def("get_gripper_distance", &ArmController::get_gripper_distance)
        .def("cleanup", &ArmController::cleanup,py::call_guard<py::gil_scoped_release>())
        ;
}
