#!/usr/bin/env python3
"""Interactively move a TypeNex gripper using timed linear angle interpolation."""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import sys
import time


SDK_ROOT = Path(__file__).resolve().parents[1]
TYPE_NEX_MAX_ANGLE_RAD = 2.065597169735


def parse_angle(value: str) -> float:
    """Parse radians by default; a 'deg' suffix selects degrees."""
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Interactively move TypeNex to target angles in timed linear steps."
    )
    parser.add_argument("--duration", type=float, default=1.0,
                        help="interpolation time per command in seconds (default: 1.0)")
    parser.add_argument("--steps", type=int, default=20,
                        help="linear steps per command (default: 20)")
    parser.add_argument("--can", default="can0", help="SocketCAN interface (default: can0)")
    parser.add_argument("--yes", action="store_true", help="skip startup confirmation")
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration < 0.0:
        parser.error("--duration must be finite and non-negative")
    if args.steps < 1:
        parser.error("--steps must be at least 1")
    return args


def print_state(arm, label: str) -> None:
    angle = float(arm.get_gripper_angle())
    state = arm.get_gripper_state()
    print(
        f"[{label}] angle={angle:.6f} rad ({math.degrees(angle):.2f} deg) "
        f"distance={state.distance_m:.5f} m position={state.position:.4f} "
        f"feedback_valid={state.feedback_valid} "
        f"feedback_age={state.feedback_age_ms:.1f} ms error=0x{state.error_code:X}",
        flush=True,
    )


def interactive_loop(arm, args: argparse.Namespace) -> None:
    print("\nInput target opening angle. Examples: 0, 30deg, 0.52rad")
    print("Commands: s/status = read state, q/quit = exit")
    print(f"Interpolation: duration={args.duration:.3f} s, steps={args.steps}")
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
        except ValueError as exc:
            print(f"Invalid input: {exc}", file=sys.stderr, flush=True)
            continue

        try:
            start_angle = float(arm.get_gripper_angle())
            print(
                f"[command] start={start_angle:.6f} rad ({math.degrees(start_angle):.2f} deg) "
                f"target={angle:.6f} rad ({math.degrees(angle):.2f} deg)",
                flush=True,
            )
            started = time.monotonic()
            arm.setGripperAngleInterpolated(
                angle, duration=args.duration, steps=args.steps
            )
            elapsed = time.monotonic() - started
            print_state(arm, f"feedback elapsed={elapsed:.3f}s")
        except RuntimeError as exc:
            print(f"Command failed: {exc}", file=sys.stderr, flush=True)


def main() -> int:
    args = parse_args()
    print(f"TypeNex interpolated angle test on {args.can}")
    print(f"Allowed opening: 0-{TYPE_NEX_MAX_ANGLE_RAD:.6f} rad (0-118.35 deg)")
    print("This initializes the controller and enables motors; it will move the gripper.")
    print("Stop other programs using the same CAN interface before continuing.")
    if not args.yes and input("Type YES to continue: ").strip() != "YES":
        print("Cancelled.")
        return 2

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
