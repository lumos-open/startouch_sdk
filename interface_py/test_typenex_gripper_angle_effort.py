#!/usr/bin/env python3
"""Interactively test the native TypeNex angle-effort API."""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import re
import sys
import time


SDK_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = SDK_ROOT / "src" / "config" / "robot_kinematics.yaml"
TYPE_NEX_MAX_ANGLE_RAD = 2.065597169735


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Interactively test native TypeNex angle-effort control."
    )
    parser.add_argument("--can", default="can0", help="SocketCAN interface (default: can0)")
    parser.add_argument(
        "--effort",
        type=float,
        required=True,
        help="Non-negative motor output-shaft torque target/limit in Nm",
    )
    parser.add_argument(
        "--settle",
        type=float,
        default=0.5,
        help="Seconds to wait before reading feedback (default: 0.5)",
    )
    parser.add_argument("--yes", action="store_true", help="Skip startup confirmation")
    args = parser.parse_args()
    if not math.isfinite(args.effort) or args.effort < 0.0:
        parser.error("--effort must be finite and non-negative")
    if not math.isfinite(args.settle) or args.settle < 0.0:
        parser.error("--settle must be finite and non-negative")
    return args


def require_type_nex_config() -> None:
    try:
        config = CONFIG_PATH.read_text(encoding="utf-8")
    except OSError as exc:
        raise SystemExit(f"Unable to read SDK config {CONFIG_PATH}: {exc}") from exc
    match = re.search(
        r"^gripper_control:\s*$.*?^\s*type:\s*(\S+)\s*(?:#.*)?$",
        config,
        re.MULTILINE | re.DOTALL,
    )
    if match is None:
        raise SystemExit(f"Cannot find gripper_control.type in {CONFIG_PATH}")
    if match.group(1) != "TypeNex":
        raise SystemExit(
            f"Configured gripper is {match.group(1)}, not TypeNex. "
            f"Set gripper_control.type to TypeNex in {CONFIG_PATH}."
        )


def parse_angle(value: str) -> float:
    """Parse radians by default; a ``deg`` suffix selects degrees."""
    text = value.strip().lower().replace(" ", "")
    if text.endswith("deg"):
        angle = math.radians(float(text[:-3]))
    elif text.endswith("rad"):
        angle = float(text[:-3])
    else:
        angle = float(text)
    if not math.isfinite(angle) or not 0.0 <= angle <= TYPE_NEX_MAX_ANGLE_RAD:
        raise ValueError(
            f"angle must be finite and in [0, {TYPE_NEX_MAX_ANGLE_RAD:.12f}] rad"
        )
    return angle


def print_state(arm, label: str) -> None:
    state = arm.get_gripper_state()
    angle = float(arm.get_gripper_angle())
    print(
        f"[{label}] angle={angle:.6f} rad ({math.degrees(angle):.2f} deg) "
        f"distance={state.distance_m:.5f} m position={state.position:.4f} "
        f"effort={state.effort_nm:.3f} Nm "
        f"commanded_effort={state.commanded_effort_nm:.3f} Nm "
        f"force_control={state.force_control_active} "
        f"feedback_valid={state.feedback_valid} "
        f"feedback_age={state.feedback_age_ms:.1f} ms "
        f"error=0x{state.error_code:X}",
        flush=True,
    )


def interactive_loop(arm, effort_nm: float, settle: float) -> None:
    print("\nInput target opening angle. Examples: 0, 30deg, 0.52rad")
    print("Commands: s/status = read state, q/quit = exit")
    print_state(arm, "current")
    while True:
        try:
            command = input("TypeNex angle> ").strip()
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
            angle = parse_angle(command)
            print(
                f"[command] angle={angle:.6f} rad ({math.degrees(angle):.2f} deg), "
                f"effort={effort_nm:.3f} Nm",
                flush=True,
            )
            # Native pybind API: no startouchclass/SingleArm wrapper is used.
            arm.setGripperAngleEffort(angle, effort_nm)
            time.sleep(settle)
            print_state(arm, "feedback")
        except ValueError as exc:
            print(f"Invalid input: {exc}", file=sys.stderr, flush=True)
        except RuntimeError as exc:
            print(f"Command failed: {exc}", file=sys.stderr, flush=True)


def main() -> int:
    args = parse_args()
    require_type_nex_config()
    print(f"TypeNex native angle-effort test on {args.can}")
    print(f"Allowed opening: 0-{TYPE_NEX_MAX_ANGLE_RAD:.6f} rad (0-118.35 deg)")
    print(f"Motor output-shaft torque target/limit: {args.effort:.3f} Nm")
    print("This initializes the arm controller and enables motors; it will move the gripper.")
    print("Stop other programs using the same CAN interface before continuing.")
    if not args.yes and input("Type YES to continue: ").strip() != "YES":
        print("Cancelled.")
        return 2

    os.chdir(SDK_ROOT)
    sys.path.insert(0, str(SDK_ROOT / "interface_py"))
    import startouch

    arm = None
    try:
        arm = startouch.ArmController(
            can_interface=args.can,
            enable_fd=False,
            gripper_exist=True,
            dry_run=False,
        )
        time.sleep(0.5)
        interactive_loop(arm, args.effort, args.settle)
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130
    finally:
        if arm is not None:
            arm.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
