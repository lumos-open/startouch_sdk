#!/usr/bin/env python3
"""TypeNex DM4310 gripper force-position smoke test.

Dry-run is the default. Pass --execute only after the arm is stationary, no
other controller is using the CAN interface, and the gripper area is clear.
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import sys
import time


SDK_ROOT = Path(__file__).resolve().parents[1]


def _prepare_runtime() -> None:
    """Make the source-tree SDK runnable without requiring env.sh."""
    os.environ.setdefault("STARTOUCH_SDK_ROOT", str(SDK_ROOT))
    os.environ.setdefault("STARTOUCH_CONFIG_DIR", str(SDK_ROOT / "src" / "config"))
    os.environ.setdefault(
        "STARTOUCH_ROBOT_KINEMATICS_CONFIG",
        str(SDK_ROOT / "src" / "config" / "robot_kinematics.yaml"),
    )
    # The normal controller startup publishes closeGripper(). A force test must
    # issue its current-limited command first, so disable only that startup action.
    os.environ["STARTOUCH_GRIPPER_STARTUP_CLOSE"] = "0"

    lib_dir = str(SDK_ROOT / "src")
    current = os.environ.get("LD_LIBRARY_PATH", "")
    entries = [entry for entry in current.split(":") if entry]
    if lib_dir not in entries and os.environ.get("STARTOUCH_FORCE_TEST_REEXEC") != "1":
        os.environ["LD_LIBRARY_PATH"] = ":".join([lib_dir, *entries])
        os.environ["STARTOUCH_FORCE_TEST_REEXEC"] = "1"
        os.execv(sys.executable, [sys.executable, *sys.argv])


def _finite_nonnegative(value: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result < 0.0:
        raise argparse.ArgumentTypeError("value must be finite and non-negative")
    return result


def _positive(value: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise argparse.ArgumentTypeError("value must be finite and positive")
    return result


def _distance(value: str) -> float:
    result = float(value)
    if not math.isfinite(result) or not 0.0 <= result <= 0.24073:
        raise argparse.ArgumentTypeError("TypeNex distance must be in [0, 0.24073] m")
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="connect to real CAN hardware")
    parser.add_argument("--yes", action="store_true", help="skip the interactive confirmation")
    parser.add_argument("--can", default="can0", help="SocketCAN interface (default: can0)")
    parser.add_argument(
        "--target-distance",
        type=_distance,
        default=0.0,
        help="force-position target opening in metres (default: 0.0)",
    )
    parser.add_argument(
        "--effort-nm",
        type=_finite_nonnegative,
        default=0.3,
        help="motor output-shaft torque target/limit in Nm (default: 0.3)",
    )
    parser.add_argument(
        "--hold-seconds",
        type=_positive,
        default=3.0,
        help="force command observation time (default: 3.0)",
    )
    parser.add_argument(
        "--sample-hz",
        type=_positive,
        default=20.0,
        help="status print rate (default: 20)",
    )
    parser.add_argument(
        "--activation-timeout",
        type=_positive,
        default=1.0,
        help="maximum time to enter force mode (default: 1.0)",
    )
    parser.add_argument(
        "--release-distance",
        type=_distance,
        default=0.24073,
        help="MIT position target after the test (default: fully open)",
    )
    parser.add_argument(
        "--release-seconds",
        type=_finite_nonnegative,
        default=1.5,
        help="time allowed for release before cleanup (default: 1.5)",
    )
    return parser.parse_args()


def confirm_hardware_test(args: argparse.Namespace) -> None:
    if not args.execute or args.yes:
        return
    print("\nREAL HARDWARE TEST")
    print("- Stop all other arm/teleop processes using this CAN interface.")
    print("- Keep the six-axis arm stationary and keep people clear.")
    print("- Place only the intended test object between the fingers.")
    print(f"- Command: {args.target_distance:.5f} m, {args.effort_nm:.3f} Nm")
    answer = input("Type FORCE to continue: ").strip()
    if answer != "FORCE":
        raise SystemExit("cancelled")


def print_state(elapsed: float, state) -> None:
    print(
        f"t={elapsed:6.2f}s "
        f"distance={state.distance_m:8.5f}m "
        f"tau={state.effort_nm:+7.3f}Nm "
        f"cmd={state.commanded_effort_nm:6.3f}Nm "
        f"vel={state.motor_velocity_rad_s:+7.3f}rad/s "
        f"temp={state.mos_temperature_c:3d}/{state.rotor_temperature_c:3d}C "
        f"err=0x{state.error_code:X} "
        f"age={state.feedback_age_ms:6.1f}ms "
        f"valid={int(state.feedback_valid)} active={int(state.force_control_active)}"
    )


def main() -> int:
    _prepare_runtime()
    args = parse_args()
    confirm_hardware_test(args)

    from startouchclass import SingleArm, __version__

    dry_run = not args.execute
    print(f"StarTouch SDK {__version__}; mode={'DRY-RUN' if dry_run else 'HARDWARE'}")
    arm = None
    force_command_sent = False
    try:
        arm = SingleArm(
            can_interface_=args.can,
            gripper=True,
            enable_fd_=False,
            dry_run=dry_run,
        )
        arm.setGripperDistanceEffort(args.target_distance, args.effort_nm)
        force_command_sent = True

        started = time.monotonic()
        activation_deadline = started + args.activation_timeout
        force_mode_seen = dry_run
        period = 1.0 / args.sample_hz
        next_sample = started
        while True:
            now = time.monotonic()
            if now >= next_sample:
                state = arm.get_gripper_state()
                print_state(now - started, state)
                if args.execute:
                    if not state.feedback_valid:
                        raise RuntimeError("gripper feedback became stale")
                    if state.error_code >= 0x8 or not state.enabled:
                        raise RuntimeError(
                            f"gripper motor fault/disabled: enabled={state.enabled}, "
                            f"error=0x{state.error_code:X}"
                        )
                    if state.force_control_active:
                        force_mode_seen = True
                    elif force_mode_seen:
                        raise RuntimeError("force control stopped unexpectedly")
                    elif now >= activation_deadline:
                        raise RuntimeError(
                            "force control did not activate before timeout"
                        )
                next_sample += period
            if now - started >= args.hold_seconds:
                break
            time.sleep(min(0.01, max(0.0, next_sample - now)))

        print("Switching back to MIT position mode for release.")
        arm.setGripperDistance(args.release_distance, 2.0, 0.1)
        force_command_sent = False
        if args.release_seconds > 0.0:
            time.sleep(args.release_seconds)
        final_state = arm.get_gripper_state()
        print_state(time.monotonic() - started, final_state)
        if final_state.force_control_active:
            raise RuntimeError("failed to leave force-position mode")
        print("PASS")
        return 0
    except KeyboardInterrupt:
        print("Interrupted by operator.", file=sys.stderr)
        return 130
    finally:
        if arm is not None:
            if force_command_sent:
                try:
                    # Any ordinary position command switches the motor back to MIT.
                    current = arm.get_gripper_state().distance_m
                    arm.setGripperDistance(current, 0.1, 0.1)
                    time.sleep(0.05)
                except Exception as exc:
                    print(f"Failed to leave force mode cleanly: {exc}", file=sys.stderr)
            arm.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
