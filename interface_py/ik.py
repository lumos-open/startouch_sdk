#!/usr/bin/env python3
import argparse
import time
from typing import Optional, Tuple

import numpy as np

from ik_common import run_teleop_loop
from ik_process_utils import apply_process_tuning
from startouchclass import SingleArm

INIT_SLEEP_SEC = 3.0


class LocalArmBackend:
    def __init__(self, arm: SingleArm):
        self.arm = arm

    def get_initial_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        default_pos = np.array([0.2833, 0.0, 0.17605], dtype=float)
        default_euler = np.array([0.0, 0.0, 0.0], dtype=float)
        try:
            pos_, euler_ = self.arm.get_ee_pose_euler()
            return np.array(pos_, dtype=float), np.array(euler_, dtype=float)
        except Exception as e:
            print(f"读取当前末端位姿失败，使用默认初始位姿: {e}")
            return default_pos, default_euler

    def get_initial_gripper(self) -> Optional[float]:
        try:
            return float(self.arm.get_gripper_position())
        except Exception as e:
            print(f"读取当前夹爪位置失败，启动时不发送夹爪命令: {e}")
            return None

    def send_pose(self, pos: np.ndarray, euler: np.ndarray) -> None:
        self.arm.set_end_effector_pose_euler_raw(pos, euler)

    def send_gripper(self, gripper_pos: float) -> None:
        self.arm.setGripperPosition(gripper_pos)

    def go_home(self) -> None:
        self.arm.go_home()

    def print_state(
        self,
        pos: np.ndarray,
        euler: np.ndarray,
        gripper_pos: Optional[float],
    ) -> None:
        cur_pos, cur_euler = self.arm.get_ee_pose_euler()
        cur_gripper = self.arm.get_gripper_position()
        q = self.arm.get_joint_positions()
        print("Q:", q)
        print(
            f"target pos = {pos}, target euler(rpy) = {euler}, target gripper = {gripper_pos}"
        )
        print(
            f"current pos = {np.array(cur_pos)}, current euler(rpy) = {np.array(cur_euler)}, "
            f"current gripper = {float(cur_gripper):.3f}"
        )

    def cleanup(self) -> None:
        self.arm.cleanup()


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "单臂笛卡尔键盘遥操作。"
            "直连 SocketCAN 双进程时默认启用 CPU/调度隔离；"
            "若仍 Communication lost，可改用 ik_arm_server + ik_teleop。"
        )
    )
    parser.add_argument("--can", default="can0", help="CAN 接口，例如 can0 / can1")
    parser.add_argument(
        "--no-tuning",
        action="store_true",
        help="禁用双进程 CPU/调度隔离（仅调试时使用）",
    )
    args = parser.parse_args()

    apply_process_tuning(args.can, enabled=not args.no_tuning)
    arm = SingleArm(can_interface_=args.can, enable_fd_=False)
    time.sleep(INIT_SLEEP_SEC)

    backend = LocalArmBackend(arm)
    try:
        run_teleop_loop(backend, args.can)
    finally:
        backend.cleanup()


if __name__ == "__main__":
    main()
