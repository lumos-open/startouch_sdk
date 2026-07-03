#!/usr/bin/env python3
"""Lightweight keyboard client for ik_arm_server.py."""

from __future__ import annotations

import argparse
import json
import socket
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import numpy as np

from ik_arm_server import socket_path_for_can
from ik_common import run_teleop_loop


class RemoteArmBackend:
    def __init__(self, can_interface: str, socket_path: Path):
        self.can_interface = can_interface
        self.socket_path = socket_path

    def _request(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
            sock.connect(str(self.socket_path))
            sock.sendall((json.dumps(payload) + "\n").encode("utf-8"))
            data = sock.recv(65536).decode("utf-8").strip()
        if not data:
            raise RuntimeError(f"empty response from {self.socket_path}")
        response = json.loads(data)
        if not response.get("ok", False):
            raise RuntimeError(response.get("error", "remote request failed"))
        return response

    def get_initial_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        default_pos = np.array([0.2833, 0.0, 0.17605], dtype=float)
        default_euler = np.array([0.0, 0.0, 0.0], dtype=float)
        try:
            response = self._request({"cmd": "get_state"})
            return (
                np.array(response["pos"], dtype=float),
                np.array(response["euler"], dtype=float),
            )
        except Exception as e:
            print(f"读取远端初始位姿失败，使用默认值: {e}")
            return default_pos, default_euler

    def get_initial_gripper(self) -> Optional[float]:
        try:
            response = self._request({"cmd": "get_state"})
            return float(response["gripper"])
        except Exception as e:
            print(f"读取远端夹爪位置失败: {e}")
            return None

    def send_pose(self, pos: np.ndarray, euler: np.ndarray) -> None:
        self._request(
            {
                "cmd": "set_pose",
                "pos": [float(x) for x in pos],
                "euler": [float(x) for x in euler],
            }
        )

    def send_gripper(self, gripper_pos: float) -> None:
        self._request({"cmd": "set_gripper", "position": float(gripper_pos)})

    def go_home(self) -> None:
        self._request({"cmd": "go_home"})

    def print_state(
        self,
        pos: np.ndarray,
        euler: np.ndarray,
        gripper_pos: Optional[float],
    ) -> None:
        response = self._request({"cmd": "get_state"})
        print("Q:", response.get("joints"))
        print(
            f"target pos = {pos}, target euler(rpy) = {euler}, target gripper = {gripper_pos}"
        )
        print(
            f"current pos = {np.array(response['pos'])}, "
            f"current euler(rpy) = {np.array(response['euler'])}, "
            f"current gripper = {float(response['gripper']):.3f}"
        )

    def cleanup(self) -> None:
        return


def main() -> None:
    parser = argparse.ArgumentParser(description="单臂键盘遥操作客户端（连接 ik_arm_server.py）")
    parser.add_argument("--can", default="can0", help="目标 CAN 接口，例如 can0 / can1")
    parser.add_argument("--socket", default="", help="自定义 Unix socket 路径")
    args = parser.parse_args()

    socket_path = Path(args.socket) if args.socket else socket_path_for_can(args.can)
    if not socket_path.exists():
        raise SystemExit(
            f"socket 不存在: {socket_path}\n"
            f"请先启动: sudo python3 ik_arm_server.py --can {args.can}"
        )

    backend = RemoteArmBackend(args.can, socket_path)
    run_teleop_loop(backend, f"{args.can} (remote)")


if __name__ == "__main__":
    main()
