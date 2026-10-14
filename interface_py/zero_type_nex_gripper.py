#!/usr/bin/env python3
"""Zero only the TypeNex gripper motor on a SocketCAN interface.

This script sends management frames directly to gripper motor CAN ID 0x07.
It intentionally does not construct ArmController and never calls set_zero_all().
The DM protocol has no dedicated set-zero success code, so success is verified
from the first post-command status feedback that reports a near-zero position.
"""

from __future__ import annotations

import argparse
import fcntl
from pathlib import Path
import shutil
import socket
import struct
import subprocess
import sys
import time


CMD_ENABLE = 0xFC
CMD_DISABLE = 0xFD
CMD_SET_ZERO = 0xFE
DM4310_POSITION_LIMIT_RAD = 12.5
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_SFF_MASK = 0x000007FF


def parse_can_id(value: str) -> int:
    try:
        can_id = int(value, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("CAN ID must be decimal or 0x-prefixed hexadecimal") from exc
    if not 0 <= can_id <= 0x7FF:
        raise argparse.ArgumentTypeError("Only standard CAN IDs in [0x000, 0x7FF] are supported")
    return can_id


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Zero only a TypeNex gripper motor.")
    parser.add_argument("--can", default="can1", help="SocketCAN interface (default: can1)")
    parser.add_argument(
        "--gripper-can-id",
        type=parse_can_id,
        default=0x07,
        help="Gripper command CAN ID (default: 0x07)",
    )
    parser.add_argument(
        "--feedback-can-id",
        type=parse_can_id,
        default=0x00,
        help="Motor feedback/Master ID (default: 0x00)",
    )
    parser.add_argument("--attempts", type=int, default=3, help="Frames sent per operation (default: 3)")
    parser.add_argument(
        "--ack-timeout",
        type=float,
        default=0.5,
        help="Seconds to wait for zero-position feedback (default: 0.5)",
    )
    parser.add_argument(
        "--zero-tolerance-rad",
        type=float,
        default=0.02,
        help="Maximum absolute feedback position accepted as zero (default: 0.02 rad)",
    )
    parser.add_argument("--execute", action="store_true", help="Actually send CAN frames")
    return parser.parse_args()


def management_frame(can_id: int, command: int) -> str:
    return f"{can_id:03X}#FFFFFFFFFFFFFF{command:02X}"


def check_environment(args: argparse.Namespace) -> None:
    if args.attempts < 1 or args.attempts > 10:
        raise SystemExit("--attempts must be in [1, 10]")
    if args.ack_timeout <= 0:
        raise SystemExit("--ack-timeout must be greater than zero")
    if args.zero_tolerance_rad <= 0:
        raise SystemExit("--zero-tolerance-rad must be greater than zero")
    if args.gripper_can_id > 0x0F:
        raise SystemExit("--gripper-can-id must fit the 4-bit motor ID in DM feedback")
    if shutil.which("cansend") is None:
        raise SystemExit("Missing command: cansend (install package can-utils)")
    try:
        socket.if_nametoindex(args.can)
    except OSError as exc:
        raise SystemExit(f"CAN interface does not exist: {args.can}") from exc

    result = subprocess.run(
        ["ip", "link", "show", "dev", args.can],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    if result.returncode != 0:
        raise SystemExit(f"Unable to inspect {args.can}: {result.stdout.strip()}")
    first_line = result.stdout.splitlines()[0] if result.stdout else ""
    if "UP" not in first_line.split("<", 1)[-1].split(">", 1)[0].split(","):
        raise SystemExit(f"CAN interface is not UP: {first_line}")


def send_reliably(can_interface: str, frame: str, attempts: int) -> None:
    for index in range(attempts):
        result = subprocess.run(
            ["cansend", can_interface, frame],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        if result.returncode != 0:
            detail = result.stderr.strip() or result.stdout.strip() or "unknown cansend error"
            raise RuntimeError(f"cansend failed: {detail}")
        if index + 1 < attempts:
            time.sleep(0.01)


def open_feedback_socket(can_interface: str) -> socket.socket:
    feedback_socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    feedback_socket.bind((can_interface,))
    feedback_socket.setblocking(False)
    return feedback_socket


def drain_feedback_socket(feedback_socket: socket.socket) -> None:
    while True:
        try:
            feedback_socket.recv(16)
        except BlockingIOError:
            return


def decode_dm4310_position(data: bytes) -> float:
    raw_position = (data[1] << 8) | data[2]
    return (
        raw_position * (2.0 * DM4310_POSITION_LIMIT_RAD) / 0xFFFF
        - DM4310_POSITION_LIMIT_RAD
    )


def wait_for_zero_feedback(
    feedback_socket: socket.socket,
    feedback_can_id: int,
    motor_can_id: int,
    timeout: float,
    tolerance_rad: float,
) -> float:
    deadline = time.monotonic() + timeout
    last_position = None
    last_status = None

    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            detail = "no matching feedback received"
            if last_position is not None:
                detail = (
                    f"last position={last_position:.6f} rad, "
                    f"status=0x{last_status:X}"
                )
            raise RuntimeError(f"zero-position feedback timeout: {detail}")

        feedback_socket.settimeout(remaining)
        try:
            frame = feedback_socket.recv(16)
        except TimeoutError:
            continue

        if len(frame) != 16:
            continue
        raw_can_id, data_length, data = struct.unpack("=IB3x8s", frame)
        if raw_can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG):
            continue
        if (raw_can_id & CAN_SFF_MASK) != feedback_can_id or data_length < 8:
            continue

        status = data[0] >> 4
        reported_motor_id = data[0] & 0x0F
        if reported_motor_id != (motor_can_id & 0x0F):
            continue

        position = decode_dm4310_position(data)
        last_position = position
        last_status = status
        if status >= 0x8:
            raise RuntimeError(f"motor reported fault 0x{status:X} after set-zero")
        if status not in (0x0, 0x1):
            raise RuntimeError(f"motor reported unexpected status 0x{status:X} after set-zero")
        if abs(position) <= tolerance_rad:
            return position


def main() -> int:
    args = parse_args()
    check_environment(args)

    enable_frame = management_frame(args.gripper_can_id, CMD_ENABLE)
    disable_frame = management_frame(args.gripper_can_id, CMD_DISABLE)
    zero_frame = management_frame(args.gripper_can_id, CMD_SET_ZERO)

    print("TYPE NEX GRIPPER ZERO CALIBRATION")
    print(f"CAN interface : {args.can}")
    print(f"Gripper CAN ID: 0x{args.gripper_can_id:03X}")
    print(f"Feedback CAN ID: 0x{args.feedback_can_id:03X}")
    print(f"Disable frame : {disable_frame}")
    print(f"Enable frame  : {enable_frame}")
    print(f"Set-zero frame: {zero_frame}")
    print("This changes the motor zero reference and cannot be undone automatically.")
    print("Stop every arm/teleoperation process using this CAN interface first.")

    if not args.execute:
        print("ZERO_RESULT=PREVIEW_ONLY")
        print("Add --execute only after checking the CAN interface and motor ID.")
        return 0

    confirmation = input("Type ZERO to disable the gripper and begin: ").strip()
    if confirmation != "ZERO":
        print("ZERO_RESULT=CANCELLED")
        return 2

    lock_path = Path(f"/tmp/startouch_gripper_zero_{args.can}_{args.gripper_can_id:03X}.lock")
    lock_path.touch(exist_ok=True)
    motor_enabled = False
    with lock_path.open("r+") as lock_file:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX)
        feedback_socket = None
        try:
            feedback_socket = open_feedback_socket(args.can)
            print("[1/4] Disabling gripper motor...")
            send_reliably(args.can, disable_frame, args.attempts)

            print("[2/4] Manually place both fingers at the true fully-closed reference.")
            print("      Do not force the linkage beyond its mechanical stop.")
            closed_confirmation = input("Type CLOSED while holding that position: ").strip()
            if closed_confirmation != "CLOSED":
                print("ZERO_RESULT=CANCELLED")
                return 2

            print("[3/4] Enabling and saving the current motor position as zero...")
            send_reliably(args.can, enable_frame, args.attempts)
            motor_enabled = True
            time.sleep(0.05)
            drain_feedback_socket(feedback_socket)
            send_reliably(args.can, zero_frame, args.attempts)
            zero_position = wait_for_zero_feedback(
                feedback_socket,
                args.feedback_can_id,
                args.gripper_can_id,
                args.ack_timeout,
                args.zero_tolerance_rad,
            )
            print(f"      Zero feedback confirmed: position={zero_position:.6f} rad")

            print("[4/4] Disabling gripper motor...")
            send_reliably(args.can, disable_frame, args.attempts)
            motor_enabled = False
        except KeyboardInterrupt:
            print("\nInterrupted.", file=sys.stderr)
            print("ZERO_RESULT=FAILED", file=sys.stderr)
            return 130
        except Exception as exc:  # noqa: BLE001
            print(f"ERROR: {exc}", file=sys.stderr)
            print("ZERO_RESULT=FAILED", file=sys.stderr)
            return 1
        finally:
            if motor_enabled:
                try:
                    send_reliably(args.can, disable_frame, args.attempts)
                except Exception as exc:  # noqa: BLE001
                    print(f"WARNING: final disable failed: {exc}", file=sys.stderr)
            if feedback_socket is not None:
                feedback_socket.close()

    print("ZERO_RESULT=SUCCESS")
    print("The motor returned fault-free feedback with position near zero.")
    print("Run the gripper test and confirm that fully closed feedback is near motor=0 rad.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
