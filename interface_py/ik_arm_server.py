#!/usr/bin/env python3
"""
Single-arm IK server for dual-process teleop without USB middleware.

Keeps SingleArm + 400Hz control loop in a dedicated process. A lightweight
ik_teleop.py client only handles keyboard input and sends pose commands over a
Unix socket, reducing cross-process scheduling interference.

Usage:
  # terminal 1
  sudo python3 ik_arm_server.py --can can0
  # terminal 2
  python3 ik_teleop.py --can can0

  # terminal 3
  sudo python3 ik_arm_server.py --can can1
  # terminal 4
  python3 ik_teleop.py --can can1
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import time
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import numpy as np

from ik_process_utils import apply_process_tuning
from startouchclass import SingleArm

INIT_SLEEP_SEC = 3.0
SOCKET_TIMEOUT_SEC = 1.0


def socket_path_for_can(can_interface: str) -> Path:
    override = os.environ.get("STARTOUCH_IK_SOCKET_DIR", "").strip()
    base = Path(override) if override else Path("/tmp")
    safe_name = can_interface.replace("/", "_")
    return base / f"startouch_ik_{safe_name}.sock"


class ArmService:
    def __init__(self, arm: SingleArm):
        self.arm = arm

    def handle(self, request: Dict[str, Any]) -> Dict[str, Any]:
        cmd = request.get("cmd")
        if cmd == "set_pose":
            pos = request.get("pos")
            euler = request.get("euler")
            if not isinstance(pos, list) or not isinstance(euler, list):
                return {"ok": False, "error": "set_pose requires pos/euler lists"}
            if len(pos) != 3 or len(euler) != 3:
                return {"ok": False, "error": "set_pose expects 3D pos and euler"}
            self.arm.set_end_effector_pose_euler_raw(pos, euler)
            return {"ok": True}
        if cmd == "set_gripper":
            position = request.get("position")
            if position is None:
                return {"ok": False, "error": "set_gripper requires position"}
            self.arm.setGripperPosition(float(position))
            return {"ok": True}
        if cmd == "go_home":
            self.arm.go_home()
            return {"ok": True}
        if cmd == "get_state":
            pos, euler = self.arm.get_ee_pose_euler()
            return {
                "ok": True,
                "pos": [float(x) for x in pos],
                "euler": [float(x) for x in euler],
                "gripper": float(self.arm.get_gripper_position()),
                "joints": [float(x) for x in self.arm.get_joint_positions()],
            }
        if cmd == "ping":
            return {"ok": True, "pong": True}
        return {"ok": False, "error": f"unknown cmd: {cmd}"}

    def cleanup(self) -> None:
        self.arm.cleanup()


def serve(can_interface: str, socket_path: Path) -> None:
    if socket_path.exists():
        socket_path.unlink()

    apply_process_tuning(can_interface)
    arm = SingleArm(can_interface_=can_interface, enable_fd_=False)
    time.sleep(INIT_SLEEP_SEC)
    service = ArmService(arm)

    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(socket_path))
    server.listen(4)
    os.chmod(socket_path, 0o666)
    print(f"[ik_arm_server] {can_interface} listening on {socket_path}")

    stop = False

    def _stop_handler(signum, frame) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _stop_handler)
    signal.signal(signal.SIGTERM, _stop_handler)

    try:
        while not stop:
            server.settimeout(SOCKET_TIMEOUT_SEC)
            try:
                conn, _ = server.accept()
            except socket.timeout:
                continue
            with conn:
                conn.settimeout(SOCKET_TIMEOUT_SEC)
                try:
                    payload = conn.recv(65536).decode("utf-8").strip()
                    if not payload:
                        continue
                    request = json.loads(payload)
                    response = service.handle(request)
                except Exception as exc:
                    response = {"ok": False, "error": str(exc)}
                conn.sendall((json.dumps(response) + "\n").encode("utf-8"))
    finally:
        server.close()
        if socket_path.exists():
            socket_path.unlink()
        service.cleanup()
        print(f"[ik_arm_server] {can_interface} stopped")


def main() -> None:
    parser = argparse.ArgumentParser(description="单臂 IK/CAN 服务进程（配合 ik_teleop.py）")
    parser.add_argument("--can", default="can0", help="CAN 接口，例如 can0 / can1")
    parser.add_argument("--socket", default="", help="自定义 Unix socket 路径")
    args = parser.parse_args()

    socket_path = Path(args.socket) if args.socket else socket_path_for_can(args.can)
    serve(args.can, socket_path)


if __name__ == "__main__":
    main()
