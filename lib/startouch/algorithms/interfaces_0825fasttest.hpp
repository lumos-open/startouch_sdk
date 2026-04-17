#pragma once

#include <Eigen/Dense>      // 提供 Eigen::Vector3d, Matrix3d, Matrix4d 等
#include <vector>          // 提供 std::vector
#include <utility>
#include <memory>
#include <atomic>

#include "startouch/damiao_motor/dm_motor_constants.hpp"


class ArmController {
    public:
        ArmController(
            bool  gripper_exist = false,
            const std::string& can_interface = "can0", 
            bool enable_fd = false,
            startouch::damiao_motor::ControlMode motor_control_mode =
                //startouch::damiao_motor::ControlMode::POS_VEL_MODE
                startouch::damiao_motor::ControlMode::MIT_MODE
        );

        ~ArmController();
        void set_joint(const std::vector<double>& q_end, double tf = 3.0);
        void set_joint_raw(const std::vector<double>& q_end,const std::vector<double>& v_end);

        // void set_joint_raw_ik(const std::vector<double>& q_end,const std::vector<double>& v_end, const std::vector<double>& q_now);
        void identify_gravity_compensation();
        void identify_gravity_compensation_kdl();
        void set_end_effector_pose(const std::vector<double>& target_pos, const std::vector<double>& target_euler ,double tf) ;
        void set_end_effector_pose_raw(const std::vector<double>& target_pos, const std::vector<double>& target_euler);
        std::pair<std::vector<double>, bool> solve_ik(
            const std::vector<double>& target_pos,
            const std::vector<double>& target_euler,
            const std::vector<double>& q_seed = {});
         
       

        std::pair<Eigen::Vector3d, Eigen::Vector3d> get_end_effector_pose();
        Eigen::VectorXd get_joint();
        Eigen::VectorXd get_joint_cached();
        Eigen::VectorXd get_joint_velocities();
        Eigen::VectorXd get_joint_velocities_cached();
        Eigen::VectorXd get_joint_torques();
        void openGripper();
        void closeGripper();
        void setGripperPosition_raw(double position);
        void setGripperPosition(double position);
        double get_gripper_position();
        void cleanup();
    private:
        // 线程对象
        std::atomic<bool> closed{false};
        class Impl;  // 声明私有实现类
        std::unique_ptr<Impl> pimpl_;  // 私有实现的指针
    };
