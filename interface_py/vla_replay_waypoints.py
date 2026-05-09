import argparse
import ast
import math
import re
import time
from pathlib import Path

from startouchclass import SingleArm


DEFAULT_TRAJ_DIR = Path("/home/lumos/code/FastTouchV2/fnl/fnl/vlareplay")
CONFIG_PATH = Path("/home/lumos/code/FastTouchV2/fnl/fnl/startouch_sdk/src/config/robot_kinematics.yaml")
HOME = [0.0, 0, 0.0, 0, 0.0, 0.0]
CONTROL_HZ = 400.0


def parse_traj(path: Path):
    pattern = re.compile(
        r"cmd_pos=(\[[^\]]+\])\s+cmd_euler_rad=(\[[^\]]+\])\s+cmd_gripper=([-\d.]+)"
    )
    points = []
    for line in path.read_text().splitlines():
        match = pattern.search(line)
        if not match:
            continue
        pos = ast.literal_eval(match.group(1))
        euler = ast.literal_eval(match.group(2))
        gripper = float(match.group(3))
        points.append((pos, euler, gripper))
    return points


def solve_joint_waypoints(arm: SingleArm, cartesian_points):
    q_seed = list(arm.get_joint_positions())
    joint_points = []
    for idx, (pos, euler, _gripper) in enumerate(cartesian_points):
        q, ok = arm.arm.solve_ik(list(pos), list(euler), q_seed)
        if not ok:
            raise RuntimeError(f"IK failed at point {idx}: pos={pos}, euler={euler}")
        q_seed = list(q)
        joint_points.append(q_seed)
    return joint_points


def load_joint_trajectory_limits(config_path: Path):
    text = config_path.read_text()

    def read_array(key):
        match = re.search(rf"{key}:\s*(\[[^\]]+\])", text)
        if not match:
            raise RuntimeError(f"missing {key} in {config_path}")
        values = ast.literal_eval(match.group(1))
        if len(values) != 6:
            raise RuntimeError(f"{key} must contain 6 values")
        return [float(v) for v in values]

    match = re.search(r"default_speed_percent:\s*([-\d.]+)", text)
    default_speed = float(match.group(1)) if match else 0.1
    return {
        "max_vel": read_array("max_vel_limits"),
        "max_acc": read_array("max_acc_limits"),
        "max_jerk": read_array("max_jerk_limits"),
        "default_speed_percent": default_speed,
    }


def estimate_waypoint_duration(q_start, waypoints, limits, speed_percent, control_hz=CONTROL_HZ):
    if speed_percent <= 0.0:
        speed_percent = limits["default_speed_percent"]
    if speed_percent <= 0.0 or speed_percent > 1.0:
        raise RuntimeError("speed_percent must be in (0, 1]")

    max_vel = [v * speed_percent for v in limits["max_vel"]]
    max_acc = limits["max_acc"]
    max_jerk = limits["max_jerk"]

    points = [list(q_start)] + [list(q) for q in waypoints]
    compact = [points[0]]
    for p in points[1:]:
        if max(abs(a - b) for a, b in zip(p, compact[-1])) > 1e-9:
            compact.append(p)
    points = compact
    if len(points) < 2:
        return 0.0

    tangents = [[0.0] * 6 for _ in points]
    for i in range(1, len(points) - 1):
        for j in range(6):
            d_prev = points[i][j] - points[i - 1][j]
            d_next = points[i + 1][j] - points[i][j]
            tangents[i][j] = 0.0 if d_prev * d_next <= 0.0 else 0.5 * (d_prev + d_next)

    total = 0.0
    for i in range(len(points) - 1):
        p0 = points[i]
        p1 = points[i + 1]
        m0 = tangents[i]
        m1 = tangents[i + 1]
        t_min = 1.0 / control_hz

        for j in range(6):
            d = abs(p1[j] - p0[j])
            m = max(abs(m0[j]), abs(m1[j]))
            span = max(d, m)
            t_min = max(t_min, span / max_vel[j])
            t_min = max(t_min, math.sqrt(span / max_acc[j]))
            t_min = max(t_min, (span / max_jerk[j]) ** (1.0 / 3.0))

        max_dq_du = [0.0] * 6
        max_ddq_du2 = [0.0] * 6
        max_dddq_du3 = [0.0] * 6
        for k in range(65):
            u = k / 64.0
            u2 = u * u
            u3 = u2 * u
            u4 = u3 * u
            for j in range(6):
                d = p1[j] - p0[j] - m0[j]
                v = m1[j] - m0[j]
                c3 = 10.0 * d - 4.0 * v
                c4 = 7.0 * v - 15.0 * d
                c5 = 6.0 * d - 3.0 * v
                dq_du = m0[j] + 3.0 * c3 * u2 + 4.0 * c4 * u3 + 5.0 * c5 * u4
                ddq_du2 = 6.0 * c3 * u + 12.0 * c4 * u2 + 20.0 * c5 * u3
                dddq_du3 = 6.0 * c3 + 24.0 * c4 * u + 60.0 * c5 * u2
                max_dq_du[j] = max(max_dq_du[j], abs(dq_du))
                max_ddq_du2[j] = max(max_ddq_du2[j], abs(ddq_du2))
                max_dddq_du3[j] = max(max_dddq_du3[j], abs(dddq_du3))

        for j in range(6):
            if max_dq_du[j] > 0.0:
                t_min = max(t_min, max_dq_du[j] / max_vel[j])
            if max_ddq_du2[j] > 0.0:
                t_min = max(t_min, math.sqrt(max_ddq_du2[j] / max_acc[j]))
            if max_dddq_du3[j] > 0.0:
                t_min = max(t_min, (max_dddq_du3[j] / max_jerk[j]) ** (1.0 / 3.0))

        ticks = max(1, math.ceil(t_min * control_hz))
        total += ticks / control_hz
    return total


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("can_interface", nargs="?", default="can0")
    parser.add_argument("--file", type=Path, default=None)
    parser.add_argument("--group-size", type=int, default=30)
    parser.add_argument("--pause", type=float, default=0.1)
    parser.add_argument("--speed-percent", type=float, default=0.1)
    parser.add_argument("--wait-margin", type=float, default=0.05)
    parser.add_argument("--max-points", type=int, default=0)
    parser.add_argument("--single-trajectory", action="store_true")
    args = parser.parse_args()

    traj_file = args.file
    if traj_file is None:
        traj_file = sorted(DEFAULT_TRAJ_DIR.glob("inferenceTraj_*.txt"))[0]

    cartesian_points = parse_traj(traj_file)
    if args.max_points > 0:
        cartesian_points = cartesian_points[:args.max_points]
    if not cartesian_points:
        raise RuntimeError(f"no cmd_pos/cmd_euler_rad points found in {traj_file}")

    print(f"using trajectory: {traj_file}")
    print(f"cartesian points: {len(cartesian_points)}")
    limits = load_joint_trajectory_limits(CONFIG_PATH)
    print(
        f"speed mode: speed_percent={args.speed_percent}, "
        f"pause={args.pause}s, wait_margin={args.wait_margin}s"
    )

    arm = SingleArm(can_interface_=args.can_interface, enable_fd_=False)
    try:
        print("go home")
        arm.set_joint(HOME, tf=3.0)
        time.sleep(3.2)

        print("solve IK")
        joint_points = solve_joint_waypoints(arm, cartesian_points)
        print(f"joint waypoints: {len(joint_points)}")
        current_q = list(arm.get_joint_positions())

        if args.single_trajectory:
            total_time = estimate_waypoint_duration(current_q, joint_points, limits, args.speed_percent)
            print(f"execute all points as one speed trajectory, estimated_time={total_time:.3f}s")
            arm.move_joint_waypoints(
                joint_points,
                time_sec=0.0,
                speed_percent=args.speed_percent,
                ctrl_hz=CONTROL_HZ,
            )
            time.sleep(total_time + args.wait_margin)
            return

        group_size = max(1, args.group_size)
        for start in range(0, len(joint_points), group_size):
            group = joint_points[start:start + group_size]
            duration = estimate_waypoint_duration(current_q, group, limits, args.speed_percent)
            print(f"execute points {start}..{start + len(group) - 1}, estimated_time={duration:.3f}s")
            arm.move_joint_waypoints(
                group,
                time_sec=0.0,
                speed_percent=args.speed_percent,
                ctrl_hz=CONTROL_HZ,
            )
            time.sleep(duration + args.pause + args.wait_margin)
            current_q = list(group[-1])

        print("return home")
        arm.set_joint(HOME, tf=3.0)
        time.sleep(3.2)
    finally:
        arm.cleanup()


if __name__ == "__main__":
    main()
