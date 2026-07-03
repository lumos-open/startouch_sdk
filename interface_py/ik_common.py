#!/usr/bin/env python3
"""Shared Cartesian keyboard teleop loop for ik.py / ik_teleop.py."""

from __future__ import annotations

import math
import sys
import termios
import time
import tty
from typing import Optional, Protocol, Tuple

import numpy as np

POS_STEP = 0.005
RPY_STEP = math.radians(2.0)
GRIPPER_STEP = 0.05


class ArmBackend(Protocol):
    def get_initial_pose(self) -> Tuple[np.ndarray, np.ndarray]: ...

    def get_initial_gripper(self) -> Optional[float]: ...

    def send_pose(self, pos: np.ndarray, euler: np.ndarray) -> None: ...

    def send_gripper(self, gripper_pos: float) -> None: ...

    def go_home(self) -> None: ...

    def print_state(
        self,
        pos: np.ndarray,
        euler: np.ndarray,
        gripper_pos: Optional[float],
    ) -> None: ...

    def cleanup(self) -> None: ...


def getch() -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def print_help(can_label: str) -> None:
    print(f"""
========== Cartesian Keyboard Control ({can_label}) ==========
Position:
  w/s : +X / -X
  a/d : +Y / -Y
  r/f : +Z / -Z

Orientation (RPY):
  i/k : +Roll / -Roll
  j/l : +Pitch / -Pitch
  u/o : +Yaw / -Yaw

Gripper:
  z/x : open / close

Other:
  space : print current pose
  q     : go home and quit
===============================================
""")


def run_teleop_loop(backend: ArmBackend, can_label: str) -> None:
    print_help(can_label)
    pos, euler = backend.get_initial_pose()
    gripper_pos = backend.get_initial_gripper()
    print(f"initial pos = {pos}, initial euler(rpy) = {euler}")
    if gripper_pos is not None:
        print(f"initial gripper = {gripper_pos:.3f}")

    while True:
        key = getch()
        pose_updated = False
        gripper_updated = False

        if key == "w":
            pos[0] += POS_STEP
            pose_updated = True
        elif key == "s":
            pos[0] -= POS_STEP
            pose_updated = True
        elif key == "a":
            pos[1] += POS_STEP
            pose_updated = True
        elif key == "d":
            pos[1] -= POS_STEP
            pose_updated = True
        elif key == "r":
            pos[2] += POS_STEP
            pose_updated = True
        elif key == "f":
            pos[2] -= POS_STEP
            pose_updated = True
        elif key == "i":
            euler[0] += RPY_STEP
            pose_updated = True
        elif key == "k":
            euler[0] -= RPY_STEP
            pose_updated = True
        elif key == "j":
            euler[1] += RPY_STEP
            pose_updated = True
        elif key == "l":
            euler[1] -= RPY_STEP
            pose_updated = True
        elif key == "u":
            euler[2] += RPY_STEP
            pose_updated = True
        elif key == "o":
            euler[2] -= RPY_STEP
            pose_updated = True
        elif key == "z":
            if gripper_pos is None:
                gripper_pos = 0.0
            gripper_pos = min(1.0, gripper_pos + GRIPPER_STEP)
            gripper_updated = True
        elif key == "x":
            if gripper_pos is None:
                gripper_pos = 0.0
            gripper_pos = max(0.0, gripper_pos - GRIPPER_STEP)
            gripper_updated = True
        elif key == " ":
            try:
                backend.print_state(pos, euler, gripper_pos)
            except Exception as e:
                print(f"读取当前末端状态失败: {e}")
        elif key == "q":
            print("Exit.")
            backend.go_home()
            time.sleep(4)
            break

        if pose_updated or gripper_updated:
            st = time.time()
            if pose_updated:
                backend.send_pose(pos, euler)
            if gripper_updated and gripper_pos is not None:
                backend.send_gripper(gripper_pos)
            print("", time.time() - st)
