import ast
import math
import re
import signal
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from startouchclass import SingleArm, euler_to_quaternion
from tcp_compensation import tcp_position_to_flange


CAN_INTERFACE = "can0"
ENABLE_FD = False

TRAJ_FILE = None
DEFAULT_TRAJ_DIR = Path("/home/lumos/code/FastTouchV2/fnl/fnl/vlareplay")
TRAJ_GLOB = "inferenceTraj_20260509_170209.txt"
# TRAJ_GLOB = "*209.txt"

# "cmd_flange" replays the cmd_pos/cmd_euler_rad fields written by
# pi0_rollout_single_startouch_lxh.py. "action_tcp" replays raw action xyz/rpy/g
# and applies the same TCP-to-flange conversion used by that rollout script.
REPLAY_POSE_SOURCE = "cmd_flange"
TCP_OFFSET_XYZ = [0.0, 0.0, 0.0]

# "cartesian" uses move_p/move_p_with_gripper and checks Cartesian interpolation
# reachability. "joint_waypoints" matches the older vla_replay_waypoints path:
# solve IK only at logged frames, then send one joint waypoint trajectory.
EXECUTION_MODE = "joint_waypoints"
USE_TIME_MODE = True
ENABLE_GRIPPER_REPLAY = False
GROUP_SIZE = 30
GROUP_BY_STEP_IDX_RESET = False
DEFAULT_GROUP_TIME_SEC = 10.0
GROUP_TIME_SECS = [
    10.0,
]
SPEED_PERCENT = 0.2
GROUP_PAUSE_SEC = 0.1

MOVE_TO_FIRST_POSE_USE_TIME_MODE = True
MOVE_TO_FIRST_POSE_TIME_SEC =5.0
MOVE_TO_FIRST_POSE_SPEED_PERCENT = 1

ZERO_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
RETURN_HOME_TIME_SEC = 3.0
MAX_FRAMES = 0
LOW_Z_WARN_M = 0.06
LARGE_BOUNDARY_JUMP_WARN_M = 0.02


@dataclass
class MotionResult:
    duration: float = 0.0
    error: Optional[BaseException] = None


def action_to_flange_frame(action_values, tcp_offset_xyz):
    if len(action_values) < 7:
        raise ValueError(f"action must contain 7 values, got {len(action_values)}")
    x, y, z, roll_deg, pitch_deg, yaw_deg, gripper = [float(v) for v in action_values[:7]]
    euler_rad = [
        math.radians(roll_deg),
        math.radians(pitch_deg),
        math.radians(yaw_deg),
    ]
    flange_pos = tcp_position_to_flange([x, y, z], euler_rad, tcp_offset_xyz).tolist()
    return [
        flange_pos[0],
        flange_pos[1],
        flange_pos[2],
        euler_rad[0],
        euler_rad[1],
        euler_rad[2],
        float(min(1.0, max(0.0, gripper))),
    ]


def parse_traj(path: Path):
    cmd_pattern = re.compile(
        r"(?:cmdpos|cmd_pos)=(\[[^\]]+\])\s+"
        r"(?:cmdeulerrad|cmd_euler_rad)=(\[[^\]]+\])\s+"
        r"(?:cmdgripper|cmd_gripper)=([-\d.eE]+)"
    )
    action_pattern = re.compile(r"action=(\[[^\]]+\])")
    step_pattern = re.compile(r"(?:stepidx|step_idx)=(-?\d+)")
    frames = []
    for line_no, line in enumerate(path.read_text().splitlines(), 1):
        step_match = step_pattern.search(line)
        step_idx = int(step_match.group(1)) if step_match else None
        if REPLAY_POSE_SOURCE == "cmd_flange":
            match = cmd_pattern.search(line)
            if not match:
                continue
            pos = ast.literal_eval(match.group(1))
            euler = ast.literal_eval(match.group(2))
            gripper = float(match.group(3))
            if len(pos) != 3 or len(euler) != 3:
                raise RuntimeError(f"bad frame at line {line_no}: pos/euler must be length 3")
            frame = [
                float(pos[0]),
                float(pos[1]),
                float(pos[2]),
                float(euler[0]),
                float(euler[1]),
                float(euler[2]),
                float(gripper),
            ]
        elif REPLAY_POSE_SOURCE == "action_tcp":
            match = action_pattern.search(line)
            if not match:
                continue
            frame = action_to_flange_frame(ast.literal_eval(match.group(1)), TCP_OFFSET_XYZ)
        else:
            raise ValueError("REPLAY_POSE_SOURCE must be 'cmd_flange' or 'action_tcp'")
        frames.append((step_idx, frame))
    return frames


def resolve_traj_file() -> Path:
    if TRAJ_FILE is not None:
        return Path(TRAJ_FILE)
    candidates = sorted(DEFAULT_TRAJ_DIR.glob(TRAJ_GLOB))
    if not candidates:
        raise FileNotFoundError(f"no trajectory files matched {DEFAULT_TRAJ_DIR / TRAJ_GLOB}")
    return candidates[-1]


def frame_groups(entries, group_size):
    if GROUP_BY_STEP_IDX_RESET and any(step_idx is not None for step_idx, _frame in entries):
        group_start = 0
        group = []
        for row_idx, (step_idx, frame) in enumerate(entries):
            if step_idx == 0 and group:
                yield group_start, group
                group_start = row_idx
                group = []
            group.append(frame)
        if group:
            yield group_start, group
        return

    frames = [frame for _step_idx, frame in entries]
    for start in range(0, len(frames), group_size):
        yield start, frames[start:start + group_size]


def print_data_diagnostics(entries, groups):
    frames = [frame for _step_idx, frame in entries]
    if not frames:
        return
    z_values = [frame[2] for frame in frames]
    print(f"z range: min={min(z_values):.4f}m, max={max(z_values):.4f}m")
    if min(z_values) < LOW_Z_WARN_M:
        print(
            "WARNING: trajectory contains low-z Cartesian targets; "
            "move_p may reject or shake if these poses are outside the reachable workspace."
        )

    for group_idx, (start, group) in enumerate(groups, 1):
        group_z = [frame[2] for frame in group]
        print(
            f"group {group_idx}: rows {start}..{start + len(group) - 1}, "
            f"points={len(group)}, z_min={min(group_z):.4f}, z_max={max(group_z):.4f}"
        )

    for i in range(1, len(frames)):
        prev_frame = frames[i - 1]
        frame = frames[i]
        jump = math.sqrt(sum((frame[j] - prev_frame[j]) ** 2 for j in range(3)))
        if jump > LARGE_BOUNDARY_JUMP_WARN_M:
            print(
                f"WARNING: large Cartesian jump at rows {i - 1}->{i}: "
                f"{jump:.4f}m, z {prev_frame[2]:.4f}->{frame[2]:.4f}"
            )


def solve_joint_waypoints(arm, frames):
    q_seed = list(arm.get_joint_positions())
    joint_points = []
    for idx, frame in enumerate(frames):
        quat_wxyz = euler_to_quaternion(frame[3], frame[4], frame[5])
        q, ok = arm.solve_ik(frame[:3], quat_wxyz, q_seed=q_seed)
        if not ok:
            raise RuntimeError(f"IK failed at frame {idx}: pos={frame[:3]}, euler={frame[3:6]}")
        q_seed = list(q)
        joint_points.append(q_seed)
    return joint_points


def run_gripper_timeline_thread(arm, gripper_positions, duration_sec, stop_requested):
    def worker():
        values = [float(min(1.0, max(0.0, value))) for value in gripper_positions]
        if not values:
            return
        if len(values) == 1 or duration_sec <= 0.0:
            arm.setGripperPosition(values[-1])
            return
        start_time = time.monotonic()
        last_idx = -1
        while not stop_requested.is_set():
            elapsed = time.monotonic() - start_time
            ratio = min(1.0, elapsed / duration_sec)
            idx = min(len(values) - 1, int(round(ratio * (len(values) - 1))))
            if idx != last_idx:
                arm.setGripperPosition(values[idx])
                last_idx = idx
            if ratio >= 1.0:
                break
            time.sleep(0.005)
        arm.setGripperPosition(values[-1])

    thread = threading.Thread(target=worker, daemon=True)
    thread.start()
    return thread


def run_joint_waypoints_with_optional_gripper(arm, group, time_sec, speed_percent, stop_requested):
    joint_points = solve_joint_waypoints(arm, group)
    gripper_thread = None
    if ENABLE_GRIPPER_REPLAY and time_sec is not None:
        gripper_thread = run_gripper_timeline_thread(
            arm,
            [frame[6] for frame in group],
            float(time_sec),
            stop_requested,
        )
    try:
        duration = arm.set_joint_waypoints(
            joint_points,
            time_sec=time_sec,
            speed_percent=speed_percent,
        )
    finally:
        if gripper_thread is not None:
            gripper_thread.join(timeout=1.0)
    if ENABLE_GRIPPER_REPLAY:
        arm.setGripperPosition(group[-1][6])
    return duration


def run_motion_thread(call_fn, *args, **kwargs):
    result = MotionResult()

    def worker():
        try:
            result.duration = call_fn(*args, **kwargs)
        except BaseException as exc:  # noqa: BLE001
            result.error = exc

    thread = threading.Thread(target=worker, daemon=True)
    thread.start()
    return thread, result


def wait_motion(thread, result, stop_requested):
    while thread.is_alive():
        thread.join(timeout=0.05)
        if stop_requested.is_set():
            thread.join(timeout=2.0)
            raise KeyboardInterrupt
    if result.error is not None:
        raise result.error
    return result.duration


def interruptible_sleep(seconds, stop_requested):
    deadline = time.monotonic() + max(0.0, float(seconds))
    while not stop_requested.is_set():
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return
        time.sleep(min(0.05, remaining))
    raise KeyboardInterrupt


def return_home_after_interrupt(arm):
    print("Returning to zero joint position before exit.")
    arm.set_joint_waypoints([ZERO_JOINTS], time_sec=RETURN_HOME_TIME_SEC)


def group_time_sec(group_idx):
    idx = group_idx - 1
    if 0 <= idx < len(GROUP_TIME_SECS):
        return float(GROUP_TIME_SECS[idx])
    return float(DEFAULT_GROUP_TIME_SEC)


def main():
    if GROUP_SIZE <= 0:
        raise ValueError("GROUP_SIZE must be positive")
    if USE_TIME_MODE and any(group_time_sec(i + 1) <= 0.0 for i in range(max(1, len(GROUP_TIME_SECS)))):
        raise ValueError("all group time values must be positive in time mode")
    if MOVE_TO_FIRST_POSE_USE_TIME_MODE and MOVE_TO_FIRST_POSE_TIME_SEC <= 0.0:
        raise ValueError("MOVE_TO_FIRST_POSE_TIME_SEC must be positive")
    if not MOVE_TO_FIRST_POSE_USE_TIME_MODE and not (0.0 < MOVE_TO_FIRST_POSE_SPEED_PERCENT <= 1.0):
        raise ValueError("MOVE_TO_FIRST_POSE_SPEED_PERCENT must be in (0, 1] in speed mode")
    if not USE_TIME_MODE and not (0.0 < SPEED_PERCENT <= 1.0):
        raise ValueError("SPEED_PERCENT must be in (0, 1] in speed mode")
    if EXECUTION_MODE not in ("cartesian", "joint_waypoints"):
        raise ValueError("EXECUTION_MODE must be 'cartesian' or 'joint_waypoints'")

    stop_requested = threading.Event()
    cleanup_started = threading.Event()

    def request_stop(signum, frame):  # noqa: ARG001
        stop_requested.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    traj_file = resolve_traj_file()
    frames = parse_traj(traj_file)
    if MAX_FRAMES > 0:
        frames = frames[:MAX_FRAMES]
    if not frames:
        raise RuntimeError(f"no cmdpos/cmdeulerrad/cmdgripper frames found in {traj_file}")

    print(f"using trajectory: {traj_file}")
    print(f"frames: {len(frames)}, group_size={GROUP_SIZE}, pause={GROUP_PAUSE_SEC}s")
    print(f"grouping: {'step_idx reset' if GROUP_BY_STEP_IDX_RESET else 'fixed row count'}")
    print(f"pose source: {REPLAY_POSE_SOURCE}, tcp_offset_xyz={TCP_OFFSET_XYZ}")
    if USE_TIME_MODE:
        print(f"mode: time, default_group_time_sec={DEFAULT_GROUP_TIME_SEC}, group_time_secs={GROUP_TIME_SECS}")
    else:
        print(f"mode: speed, speed_percent={SPEED_PERCENT}")
    print(f"execution mode: {EXECUTION_MODE}")
    print(f"gripper replay: {'enabled' if ENABLE_GRIPPER_REPLAY else 'disabled'}")

    arm = SingleArm(can_interface_=CAN_INTERFACE, enable_fd_=ENABLE_FD)

    def cleanup_once():
        if cleanup_started.is_set():
            return
        cleanup_started.set()
        arm.cleanup()

    try:
        groups = list(frame_groups(frames, GROUP_SIZE))
        print_data_diagnostics(frames, groups)
        total_groups = len(groups)
        first_pose = groups[0][1][0][:6]
        current_pos, current_euler = arm.get_ee_pose_euler()
        current_pose = list(current_pos) + list(current_euler)
        print(f"current pose: {current_pose}")
        if MOVE_TO_FIRST_POSE_USE_TIME_MODE:
            print(f"move current pose -> first trajectory pose, time_sec={MOVE_TO_FIRST_POSE_TIME_SEC}")
            thread, result = run_motion_thread(
                arm.move_p,
                [current_pose, first_pose],
                time_sec=MOVE_TO_FIRST_POSE_TIME_SEC,
                blend_radius_m=0.0,
            )
        else:
            print(
                "move current pose -> first trajectory pose, "
                f"speed_percent={MOVE_TO_FIRST_POSE_SPEED_PERCENT}"
            )
            thread, result = run_motion_thread(
                arm.move_p,
                [current_pose, first_pose],
                speed_percent=MOVE_TO_FIRST_POSE_SPEED_PERCENT,
                blend_radius_m=0.0,
            )
        wait_motion(thread, result, stop_requested)

        for group_idx, (start, group) in enumerate(groups, 1):
            if stop_requested.is_set():
                raise KeyboardInterrupt
            end = start + len(group) - 1
            print(f"execute group {group_idx}/{total_groups}: rows {start}..{end}, points={len(group)}")
            if USE_TIME_MODE:
                this_group_time_sec = group_time_sec(group_idx)
                print(f"group {group_idx} time_sec={this_group_time_sec}")
                if EXECUTION_MODE == "joint_waypoints":
                    thread, result = run_motion_thread(
                        run_joint_waypoints_with_optional_gripper,
                        arm,
                        group,
                        this_group_time_sec,
                        None,
                        stop_requested,
                    )
                else:
                    call_fn = arm.move_p_with_gripper if ENABLE_GRIPPER_REPLAY else arm.move_p
                    call_data = group if ENABLE_GRIPPER_REPLAY else [frame[:6] for frame in group]
                    thread, result = run_motion_thread(
                        call_fn,
                        call_data,
                        time_sec=this_group_time_sec,
                    )
            else:
                if EXECUTION_MODE == "joint_waypoints":
                    thread, result = run_motion_thread(
                        run_joint_waypoints_with_optional_gripper,
                        arm,
                        group,
                        None,
                        SPEED_PERCENT,
                        stop_requested,
                    )
                else:
                    call_fn = arm.move_p_with_gripper if ENABLE_GRIPPER_REPLAY else arm.move_p
                    call_data = group if ENABLE_GRIPPER_REPLAY else [frame[:6] for frame in group]
                    thread, result = run_motion_thread(
                        call_fn,
                        call_data,
                        speed_percent=SPEED_PERCENT,
                    )
            duration = wait_motion(thread, result, stop_requested)
            print(f"group {group_idx} done, planned_duration_s={duration:.3f}")
            interruptible_sleep(GROUP_PAUSE_SEC, stop_requested)
    except KeyboardInterrupt:
        print("Interrupted. Overriding current motion and returning to zero position.")
        return_home_after_interrupt(arm)
    finally:
        cleanup_once()


if __name__ == "__main__":
    main()
