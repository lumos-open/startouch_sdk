#!/usr/bin/env python3
"""Interactively control and read a TypeLJ gripper opening distance."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import re
import sys
import time


SDK_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = SDK_ROOT / "src" / "config" / "robot_kinematics.yaml"
TYPE_LJ_MAX_DISTANCE_M = 0.08


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Interactively control a TypeLJ gripper by opening distance."
    )
    parser.add_argument("--can", default="can0", help="SocketCAN interface (default: can0)")
    parser.add_argument("--kp", type=float, default=20.0, help="Position gain (default: 20.0)")
    parser.add_argument("--kd", type=float, default=0.1, help="Damping gain (default: 0.1)")
    parser.add_argument(
        "--settle",
        type=float,
        default=0.5,
        help="Seconds to wait before reading feedback (default: 0.5)",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Skip the startup confirmation prompt.",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if args.kp <= 0.0:
        raise SystemExit("--kp must be positive")
    if args.kd < 0.0:
        raise SystemExit("--kd must be non-negative")
    if args.settle < 0.0:
        raise SystemExit("--settle must be non-negative")


def configured_gripper_type() -> str:
    try:
        config = CONFIG_PATH.read_text(encoding="utf-8")
    except OSError as exc:
        raise SystemExit(f"Unable to read SDK config {CONFIG_PATH}: {exc}") from exc

    match = re.search(
        r"^gripper_control:\s*$.*?^\s*type:\s*(TypeFZ|TypeLJ|TypeNex)\s*(?:#.*)?$",
        config,
        re.MULTILINE | re.DOTALL,
    )
    if match is None:
        raise SystemExit(f"Cannot find gripper_control.type in {CONFIG_PATH}")
    return match.group(1)


def parse_distance(value: str) -> float:
    """Parse a distance; bare numbers and values ending in mm are millimetres."""
    text = value.strip().lower().replace(" ", "")
    if text.endswith("mm"):
        distance_m = float(text[:-2]) / 1000.0
    elif text.endswith("m"):
        distance_m = float(text[:-1])
    else:
        distance_m = float(text) / 1000.0

    if not 0.0 <= distance_m <= TYPE_LJ_MAX_DISTANCE_M:
        raise ValueError("distance must be in [0, 80] mm")
    return distance_m


def print_state(arm, label: str) -> None:
    state = arm.get_gripper_state()
    print(
        f"[{label}] distance={state.distance_m * 1000.0:.2f} mm "
        f"position={state.position:.4f} "
        f"feedback_valid={state.feedback_valid} "
        f"feedback_age={state.feedback_age_ms:.1f} ms "
        f"error=0x{state.error_code:X}"
    )


def interactive_loop(arm, args: argparse.Namespace) -> None:
    print("\nInput target opening distance. Examples: 40, 40mm, 0.04m")
    print("Commands: s/status = read state, q/quit = exit")
    print_state(arm, "current")

    while True:
        try:
            command = input("TypeLJ distance (mm)> ").strip()
        except EOFError:
            print()
            return

        if not command:
            continue
        if command.lower() in {"q", "quit", "exit"}:
            return
        if command.lower() in {"s", "status"}:
            print_state(arm, "current")
            continue

        try:
            distance_m = parse_distance(command)
        except ValueError as exc:
            print(f"Invalid input: {exc}", file=sys.stderr)
            continue

        try:
            arm.setGripperDistance(distance_m, args.kp, args.kd)
            print(f"[command] target={distance_m * 1000.0:.2f} mm")
            time.sleep(args.settle)
            print_state(arm, "feedback")
        except RuntimeError as exc:
            print(f"Command failed: {exc}", file=sys.stderr)


def main() -> int:
    args = parse_args()
    validate_args(args)

    gripper_type = configured_gripper_type()
    if gripper_type != "TypeLJ":
        raise SystemExit(
            f"Configured gripper is {gripper_type}, not TypeLJ. "
            f"Set gripper_control.type to TypeLJ in {CONFIG_PATH}."
        )

    print(f"TypeLJ gripper distance test on {args.can}")
    print(f"Allowed opening: 0-{TYPE_LJ_MAX_DISTANCE_M * 1000:.0f} mm")
    print("This initializes the arm controller and enables motors; no joint motion is commanded.")
    print("Stop other programs using the same CAN interface before continuing.")
    if not args.yes and input("Type YES to continue: ").strip() != "YES":
        print("Cancelled.")
        return 2

    # Make native SDK config discovery deterministic when launched from elsewhere.
    os.chdir(SDK_ROOT)
    sys.path.insert(0, str(SDK_ROOT / "interface_py"))
    from startouchclass import SingleArm

    arm = None
    try:
        arm = SingleArm(can_interface_=args.can, gripper=True)
        time.sleep(0.5)
        interactive_loop(arm, args)
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130
    finally:
        if arm is not None:
            arm.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
