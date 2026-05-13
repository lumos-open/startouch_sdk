from typing import List, Tuple, Union, Optional, Dict, Any
import numpy as np
import os
import sys
import startouch


def quaternion_to_euler_wxyz(quat: np.ndarray) -> np.ndarray:
    """
    将四元数转换为欧拉角（roll, pitch, yaw）
    参数：
        quat: np.ndarray, 长度为 4 的数组 [w, x, y, z]
    返回：
        roll, pitch, yaw: 以弧度为单位的欧拉角
    """
    w, x, y, z = quat
    # 计算 roll (x 轴旋转)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # 计算 pitch (y 轴旋转)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.pi / 2 * np.sign(sinp)  # 使用 90 度限制
    else:
        pitch = np.arcsin(sinp)

    # 计算 yaw (z 轴旋转)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw])


def quaternion_to_euler_xyzw(q: np.ndarray) -> np.ndarray:
    """
    将四元数转换为欧拉角 (ZYX顺序，通常用于机械臂)
    输入: q = [qx, qy, qz, qw] 或 [w, x, y, z]
    输出: [rx, ry, rz] (弧度)
    """
    if len(q) == 4:
        # 根据您的数据，看起来是[qx, qy, qz, qw]格式
        x, y, z, w = q[0], q[1], q[2], q[3]
    else:
        raise ValueError("四元数必须是4维向量")
    
    # 转换为欧拉角 (ZYX顺序，即先绕Z轴，再绕Y轴，最后绕X轴)
    # roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)  # 使用90度
    else:
        pitch = np.arcsin(sinp)
    
    # yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return np.array([roll, pitch, yaw])  # [rx, ry, rz]


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    将欧拉角（roll, pitch, yaw）转换为四元数。

    参数：
        roll: 绕 x 轴的旋转角（弧度）
        pitch: 绕 y 轴的旋转角（弧度）
        yaw: 绕 z 轴的旋转角（弧度）

    返回：
        np.ndarray: 长度为 4 的四元数数组 [w, x, y, z]
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return np.array([w, x, y, z])


class MotionProgram:
    """
    结构化运动程序。连续 movej 会在底层合并成同一组路点；sleep 是明确阻断点。
    """

    def __init__(self):
        self._items = []

    @staticmethod
    def _check_time_speed_exclusive(time_sec: float, speed_percent: float) -> None:
        if time_sec > 0.0 and speed_percent > 0.0:
            raise ValueError("time_sec and speed_percent are mutually exclusive")

    @staticmethod
    def _as_2d_points(values, width: int, name: str) -> List[List[float]]:
        arr = np.asarray(values, dtype=float)
        if arr.ndim == 1:
            if arr.shape[0] != width:
                raise ValueError(f"{name} must have {width} values")
            arr = arr.reshape(1, width)
        elif arr.ndim == 2:
            if arr.shape[1] != width:
                raise ValueError(f"each {name} row must have {width} values")
        else:
            raise ValueError(f"{name} must be a 1D point or 2D point list")
        return arr.tolist()

    def movej(
        self,
        waypoints: Union[List[List[float]], np.ndarray],
        time_sec: float = 0.0,
        speed_percent: float = -1.0,
    ) -> "MotionProgram":
        self._check_time_speed_exclusive(float(time_sec), float(speed_percent))
        item = startouch.MotionProgramItem()
        item.type = "movej"
        item.waypoints = self._as_2d_points(waypoints, 6, "waypoint")
        item.time_sec = float(time_sec)
        item.speed_percent = float(speed_percent)
        self._items.append(item)
        return self

    def movel(
        self,
        poses: Union[List[List[float]], np.ndarray],
        time_sec: float = 0.0,
        speed_percent: float = -1.0,
        blend_radius_m: float = 0.0,
    ) -> "MotionProgram":
        self._check_time_speed_exclusive(float(time_sec), float(speed_percent))
        item = startouch.MotionProgramItem()
        item.type = "movel"
        item.poses = self._as_2d_points(poses, 6, "pose")
        item.time_sec = float(time_sec)
        item.speed_percent = float(speed_percent)
        item.blend_radius_m = float(blend_radius_m)
        self._items.append(item)
        return self

    def movep(
        self,
        poses: Union[List[List[float]], np.ndarray],
        time_sec: float = 0.0,
        speed_percent: float = -1.0,
        blend_radius_m: float = 0.002,
    ) -> "MotionProgram":
        self._check_time_speed_exclusive(float(time_sec), float(speed_percent))
        item = startouch.MotionProgramItem()
        item.type = "movep"
        item.poses = self._as_2d_points(poses, 6, "pose")
        item.time_sec = float(time_sec)
        item.speed_percent = float(speed_percent)
        item.blend_radius_m = float(blend_radius_m)
        self._items.append(item)
        return self

    def sleep(self, sleep_sec: float) -> "MotionProgram":
        item = startouch.MotionProgramItem()
        item.type = "sleep"
        item.sleep_sec = float(sleep_sec)
        self._items.append(item)
        return self

    @property
    def items(self):
        return self._items


class SingleArm:
    """
    Base class for a single robot arm.

    Args:
        config (Dict[str, sAny]): Configuration dictionary for the robot arm

    Attributes:
        config (Dict[str, Any]): Configuration dictionary for the robot arm
        num_joints (int): Number of joints in the arm
    """

    def __init__(self,can_interface_ = "can0",gripper =True,enable_fd_ = False):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        parent_dir = os.path.dirname(current_dir)
        permutation_matrix = os.path.join(parent_dir, 'param_csv_gripper', 'permutationMatrix.csv')
        pi_b = os.path.join(parent_dir, 'param_csv_gripper', 'pi_b.csv')
        pi_fr = os.path.join(parent_dir, 'param_csv_gripper', 'pi_fr.csv')
        self.arm = startouch.ArmController(can_interface = can_interface_,enable_fd = enable_fd_,gripper_exist  =gripper,
                                           permutation_matrix=permutation_matrix,pi_b =pi_b,pi_fr = pi_fr)


    def go_home(self) -> bool:
        """
        Move the robot arm to a pre-defined home pose.

        Returns:
            bool: True if the action was successful, False otherwise
        """
        q_start = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 设置回到起点的位姿
        self.arm.set_joint(q_start, tf=3.0)
        return True

    def gravity_compensation(self) -> bool:
        self.arm.gravity_compensation()  
        return True


    # 带时间规划
    def set_joint(
        self,
        positions: Union[List[float], np.ndarray],  # Shape: (num_joints, 单位rad)
        tf: float = 2.0,  # 默认时间：4.0秒
        ctrl_hz: float = 400.0,  # 默认控制频率：300.0Hz
    ) -> bool:
        """
        Move the arm to the given joint position(s).

        Args:
            positions: Desired joint position(s). Shape: (6)
            **kwargs: Additional arguments
        带有时间规划的多项式

        """
        # Current pybind interface only accepts q_end and tf.
        _ = ctrl_hz
        self.arm.set_joint(q_end=positions, tf=tf)
        return True

    # 关节伺服
    def set_joint_raw(
        self,
        positions: Union[List[float], np.ndarray],  # Shape: (num_joints,单位rad)
        velocities: Union[List[float], np.ndarray],  # Shape: (num_joints,)
    ) -> bool:
        """
        Move the arm to the given joint position.

        Args:
            positions: Desired joint position(s). Shape: (6)
            **kwargs: Additional arguments
        角度透传 没有规划！！ 谨慎使用

        """
        self.arm.set_joint_raw(q_end = positions,v_end = velocities)
        return True

    def move_joint_waypoints(
        self,
        waypoints: Union[List[List[float]], np.ndarray],
        time_sec: float = 0.0,
        speed_percent: float = -1.0,
        ctrl_hz: float = 400.0,
    ) -> float:
        MotionProgram._check_time_speed_exclusive(float(time_sec), float(speed_percent))
        return self.arm.move_joint_waypoints(
            waypoints=MotionProgram._as_2d_points(waypoints, 6, "waypoint"),
            time_sec=time_sec,
            speed_percent=speed_percent,
            control_hz=ctrl_hz,
        )

    def move_pose_waypoints(
        self,
        poses: Union[List[List[float]], np.ndarray],
        time_sec: float = 0.0,
        speed_percent: float = -1.0,
        ctrl_hz: float = 400.0,
        position_tolerance_m: float = 0.005,
        orientation_tolerance_rad: float = 0.05,
    ) -> float:
        MotionProgram._check_time_speed_exclusive(float(time_sec), float(speed_percent))
        return self.arm.move_pose_waypoints(
            poses=MotionProgram._as_2d_points(poses, 6, "pose"),
            time_sec=time_sec,
            speed_percent=speed_percent,
            control_hz=ctrl_hz,
            position_tolerance_m=position_tolerance_m,
            orientation_tolerance_rad=orientation_tolerance_rad,
        )

    def move_l(
        self,
        poses: Union[List[List[float]], np.ndarray],
        time_sec: float = 0.0,
        speed_percent: float = -1.0,
        blend_radius_m: float = 0.0,
        ctrl_hz: float = 400.0,
        position_tolerance_m: float = 0.003,
        orientation_tolerance_rad: float = 0.05,
    ) -> float:
        MotionProgram._check_time_speed_exclusive(float(time_sec), float(speed_percent))
        return self.arm.move_l(
            poses=MotionProgram._as_2d_points(poses, 6, "pose"),
            time_sec=time_sec,
            speed_percent=speed_percent,
            blend_radius_m=blend_radius_m,
            control_hz=ctrl_hz,
            position_tolerance_m=position_tolerance_m,
            orientation_tolerance_rad=orientation_tolerance_rad,
        )

    def move_p(
        self,
        poses: Union[List[List[float]], np.ndarray],
        time_sec: float = 0.0,
        speed_percent: float = -1.0,
        blend_radius_m: float = 0.002,
        ctrl_hz: float = 400.0,
        position_tolerance_m: float = 0.003,
        orientation_tolerance_rad: float = 0.05,
    ) -> float:
        MotionProgram._check_time_speed_exclusive(float(time_sec), float(speed_percent))
        return self.arm.move_p(
            poses=MotionProgram._as_2d_points(poses, 6, "pose"),
            time_sec=time_sec,
            speed_percent=speed_percent,
            blend_radius_m=blend_radius_m,
            control_hz=ctrl_hz,
            position_tolerance_m=position_tolerance_m,
            orientation_tolerance_rad=orientation_tolerance_rad,
        )

    def run_motion_program(
        self,
        program: Union[MotionProgram, List[Any]],
        ctrl_hz: float = 400.0,
    ) -> float:
        items = program.items if isinstance(program, MotionProgram) else program
        return self.arm.run_motion_program(items, control_hz=ctrl_hz)


    # 带时间规划
    def set_end_effector_pose_euler(
        self,
        pos: Optional[Union[List[float], np.ndarray]] = None,  # Shape: (3,)
        euler: Optional[Union[List[float], np.ndarray]] = None,  # Shape: (3,)
        tf: float = 2.0,  # 默认时间：4.0秒 但位置速度控制模式和linear插值模式下不生效
    ) -> bool:
        """
        /**
        * @brief 设置末端执行器的目标位姿
        * 
        * 此函数用于设置末端执行器的目标位置和姿态。目标位置由三维向量 `target_pos` 指定，
        * 目标姿态通过欧拉角 `target_euler` 给出。`tf` 参数定义了到达目标位姿的时间，默认为 4.0 秒。
        *
        * @param target_pos 目标位置，三维向量 (x, y, z)
        * @param target_euler 目标姿态，欧拉角 (roll, pitch, yaw)
        * @param tf 目标位姿的到达时间，单位为秒，默认值为 4.0
        */
        """
        self.arm.set_end_effector_pose(target_pos=pos, target_euler=euler, tf=tf)
        return True
    
    def set_end_effector_pose_euler_raw(
        self,
        pos: Optional[Union[List[float], np.ndarray]] = None,  # Shape: (3,)
        euler: Optional[Union[List[float], np.ndarray]] = None,  # Shape: (3,)
    ) -> bool:
        """
        /**
        * @brief 设置末端执行器的目标位姿
        * 
        * 此函数用于设置末端执行器的目标位置和姿态。目标位置由三维向量 `target_pos` 指定，
        * 目标姿态通过欧拉角 `target_euler` 给出。`tf` 参数定义了到达目标位姿的时间，默认为 4.0 秒。
        *
        * @param target_pos 目标位置，三维向量 (x, y, z)
        * @param target_euler 目标姿态，欧拉角 (roll, pitch, yaw)
        */
        """
        self.arm.set_end_effector_pose_raw(target_pos=pos, target_euler=euler)
        return True
    
    # 带时间规划
    def set_end_effector_pose_quat(
        self,
        pos: Optional[Union[List[float], np.ndarray]] = None,  # Shape: (3,)
        quat: Optional[Union[List[float], np.ndarray]] = None,  # Shape: (4,)
        tf: float = 2.0,  # 
    ) -> bool:
        euler = quaternion_to_euler_wxyz(quat)
        self.arm.set_end_effector_pose(target_pos=pos, target_euler=euler, tf=tf)
        return True



    # 透传、适合伺服控制
    def set_end_effector_pose_quat_raw(
        self,
        pos: Optional[Union[List[float], np.ndarray]] = None,  # Shape: (3,)
        quat: Optional[Union[List[float], np.ndarray]] = None,  # Shape: (4,)
    ) -> bool:
        euler = quaternion_to_euler_wxyz(quat)  
        self.arm.set_end_effector_pose_raw(target_pos=pos, target_euler=euler)
        return True
    




    def get_joint_positions(self) -> np.ndarray:
        """
        Get the current joint position(s) of the arm.

        Args:
            joint_names: Name(s) of the joint(s) to get positions for. Shape: (num_joints,) or single string. If None,
                            return positions for all joints.

        """
        return self.arm.get_joint_positions()

    def get_joint_velocities(self) -> np.ndarray:
        """
        Get the current joint velocity(ies) of the arm.

        """
        return self.arm.get_joint_velocities()

    def get_joint_torques(self) -> np.ndarray:
        """
        Get the current joint torque(s) of the arm.        """
        return self.arm.get_joint_torques()
    

    def get_ee_pose_quat(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get the current end effector pose of the arm.

        Returns:
            End effector pose as (position, quaternion)
            Shapes: position (3,), quaternion (4,) [w, x, y, z]
        """
        result = self.arm.get_end_effector_pose()
        pos_t = result[0]#类型np.ndarray
        rpy_t = result[1]
        quat = euler_to_quaternion(rpy_t[0], rpy_t[1], rpy_t[2])
        # 返回位置和四元数
        return pos_t,quat

    def get_ee_pose_euler(
        self,
    ) -> Tuple[np.ndarray, np.ndarray]:

        result = self.arm.get_end_effector_pose()
        pos_t = result[0]#类型np.ndarray
        rpy_t = result[1]
        return pos_t,rpy_t
    
    # q_seed 数值解的初始猜疑值
    def solve_ik(self, pos, quat, q_seed=None):
        if len(pos) != 3:
            raise ValueError(f"pos must be length 3, got {len(pos)}")
        if len(quat) != 4:
            raise ValueError(f"quat must be length 4 (wxyz), got {len(quat)}")

        euler = quaternion_to_euler_wxyz(quat)  # [roll, pitch, yaw]

        if q_seed is None:
            q, ok = self.arm.solve_ik(list(pos), list(euler))
        else:
            if len(q_seed) != 6:
                raise ValueError(f"q_seed must be length 6, got {len(q_seed)}")
            q, ok = self.arm.solve_ik(list(pos), list(euler), list(q_seed))

        return q, ok


    def openGripper(self) -> bool:
        #把夹爪开到最大
        self.arm.openGripper()
        return True


    def closeGripper(self) -> bool:
        #把夹爪闭合
        self.arm.closeGripper()
        return True

    def setGripperPosition_raw(self, position:float) -> bool:
        #角度透传模式   不规划
        #设置夹爪开合程度  0是闭合 1是开合
        self.arm.setGripperPosition_raw(position)
        return True
    
    def setGripperPosition(self, position:float) -> bool:
        #设置夹爪开合程度  0是闭合 1是开合  #设置0时有点问题，有提示
        self.arm.setGripperPosition(position)
        return True

    def setGripperDistance(self, distance:float, kp:float = None, kd:float = None) -> bool:
        if kp is None:
            self.arm.setGripperDistance(distance)
        elif kd is None:
            self.arm.setGripperDistance(distance, kp)
        else:
            self.arm.setGripperDistance(distance, kp, kd)
        return True

    def setGripperDistance_raw(self, distance:float, kp:float = 8.0, kd:float = 0.1) -> bool:
        self.setGripperDistance(distance, kp, kd)
        return True

    def get_gripper_position(self) -> float:
        #设置夹爪开合程度  0是闭合 1是开合
        return self.arm.get_gripper_position()

    def get_gripper_distance(self) -> float:
        #设置夹爪开合程度  0是闭合 1是开合
        return self.arm.get_gripper_distance()

    def cleanup(self):
        # 或者可以直接在析构函数中释放资源
        print("销毁 SingleArm 对象")
        self.arm.cleanup()
