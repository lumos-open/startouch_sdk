#!/usr/bin/env python3
"""
基于 startouch 控制的单臂轨迹复现脚本（支持按 Q 安全退出，退出后仍会绘图并归位）

轨迹格式：每行 timestamp x y z qx qy qz qw（xyzw）
夹爪格式：TUM 每行 timestamp value（0~clamp_max），映射为 startouch 的 0..1
"""
import argparse
import time
import sys
import threading
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
from tqdm import tqdm
import os

from startouchclass import SingleArm, euler_to_quaternion

# ---------- 跨平台键盘检测（非阻塞，单字符）----------
_quit_flag = False
_quit_lock = threading.Lock()

def _set_quit_flag():
    global _quit_flag
    with _quit_lock:
        _quit_flag = True

def _clear_quit_flag():
    global _quit_flag
    with _quit_lock:
        _quit_flag = False

def is_quit_requested():
    with _quit_lock:
        return _quit_flag

def _kbhit():
    """检测是否有键盘输入（非阻塞）"""
    if sys.platform == 'win32':
        import msvcrt
        return msvcrt.kbhit()
    else:
        import termios, fcntl
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            new = termios.tcgetattr(fd)
            new[3] &= ~termios.ICANON
            new[3] &= ~termios.ECHO
            termios.tcsetattr(fd, termios.TCSANOW, new)
            fcntl.fcntl(fd, fcntl.F_SETFL, os.O_NONBLOCK)
            ch = sys.stdin.read(1)
            return bool(ch)
        except:
            return False
        finally:
            termios.tcsetattr(fd, termios.TCSAFLUSH, old)

def _getch():
    """读取单个字符（阻塞）"""
    if sys.platform == 'win32':
        import msvcrt
        return msvcrt.getch().decode('utf-8', errors='ignore')
    else:
        import termios, tty
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSAFLUSH, old)
        return ch

def start_quit_listener():
    """启动一个后台线程监听 'q' 键，不阻塞主程序"""
    def listener():
        while not is_quit_requested():
            if _kbhit():
                ch = _getch().lower()
                if ch == 'q':
                    _set_quit_flag()
                    print("\n[退出] 检测到 Q 键，将结束回放并继续绘图、归位...")
                    break
            time.sleep(0.05)  # 降低 CPU 占用
    t = threading.Thread(target=listener, daemon=True)
    t.start()
# ------------------------------------------------

def plot_replay_xyz_rpy_single(
    cmd_xyz: np.ndarray,
    act_xyz: np.ndarray,
    cmd_rpy: np.ndarray,
    act_rpy: np.ndarray,
    save_path: str | None = None,
    show: bool = False,
) -> None:
    """误差图：XYZ(cm) + RPY(rad)，仅显示不保存"""
    import matplotlib.pyplot as plt

    # ✅ 位置误差（m → cm）
    err_xyz = (cmd_xyz - act_xyz) * 100.0

    # ✅ 姿态误差（rad）
    err_rpy = cmd_rpy - act_rpy

    n = cmd_xyz.shape[0]
    steps = np.arange(n)

    labels = (
        "X Error (cm)",
        "Y Error (cm)",
        "Z Error (cm)",
        "Roll Error (rad)",
        "Pitch Error (rad)",
        "Yaw Error (rad)",
    )

    fig, axes = plt.subplots(6, 1, figsize=(10, 16), sharex=True)

    data_list = [
        err_xyz[:, 0],
        err_xyz[:, 1],
        err_xyz[:, 2],
        err_rpy[:, 0],
        err_rpy[:, 1],
        err_rpy[:, 2],
    ]

    for ax, data, ylab in zip(axes, data_list, labels):
        ax.plot(steps, data, linestyle="-", linewidth=1.2)
        ax.set_ylabel(ylab)
        ax.grid(True, alpha=0.3)

        # ✅ 加一条0参考线（很关键）
        ax.axhline(0.0, linestyle="--", linewidth=1)

    axes[-1].set_xlabel("轨迹步索引")
    fig.suptitle("末端位姿误差（cmd - act）")
    fig.tight_layout()

    # ✅ 保存（使用原参数）
    if save_path:
        fig.savefig(save_path, dpi=150)
        print(f"已保存误差图: {save_path}")

    # ✅ 显示（保持原逻辑）
    if show:
        plt.show()

    plt.close(fig)

def load_trajectory(path):
    data = []
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) >= 8:
                row = [float(parts[i]) for i in range(8)]
                data.append(row)
    return np.array(data) if data else None


def load_clamp_tum(path):
    ts, vals = [], []
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) >= 2:
                ts.append(float(parts[0]))
                vals.append(float(parts[1]))
    return np.array(ts), np.array(vals) if ts else (np.array([]), np.array([]))


def align_clamp_to_trajectory(traj_timestamps, clamp_ts, clamp_vals, clamp_max=90.0):
    if len(clamp_ts) == 0 or len(clamp_vals) == 0:
        return None
    idx = np.argmin(np.abs(clamp_ts[:, None] - traj_timestamps[None, :]), axis=0)
    vals = np.clip(clamp_vals[idx].astype(np.float64), 0, clamp_max)
    frac = (clamp_max - vals) / clamp_max
    return np.clip(frac, 0.0, 1.0)


def build_T_base_to_local(base_x, base_y, base_z, base_roll_deg, base_pitch_deg, base_yaw_deg):
    base_roll, base_pitch, base_yaw = np.deg2rad([base_roll_deg, base_pitch_deg, base_yaw_deg])
    rotation_base_to_local = R.from_euler('xyz', [base_roll, base_pitch, base_yaw]).as_matrix()
    T = np.eye(4)
    T[:3, :3] = rotation_base_to_local
    T[:3, 3] = [base_x, base_y, base_z]
    return T


def transform_to_base_quat(x, y, z, qx, qy, qz, qw, T_base_to_local):
    rotation_local = R.from_quat([qx, qy, qz, qw]).as_matrix()
    T_local = np.eye(4)
    T_local[:3, :3] = rotation_local
    T_local[:3, 3] = [x, y, z]
    T_base_r = np.matmul(T_local[:3, :3], T_base_to_local[:3, :3])
    x_base, y_base, z_base = T_base_to_local[:3, 3] + T_local[:3, 3]
    rotation_base = R.from_matrix(T_base_r)
    qx_b, qy_b, qz_b, qw_b = rotation_base.as_quat()  # scipy xyzw
    return x_base, y_base, z_base, qx_b, qy_b, qz_b, qw_b


def trajectory_to_startouch(traj, T_base_to_local=None):
    out = []
    for row in traj:
        t, x, y, z, qx, qy, qz, qw = row[:8]
        if T_base_to_local is not None:
            xb, yb, zb, qx_b, qy_b, qz_b, qw_b = transform_to_base_quat(x, y, z, qx, qy, qz, qw, T_base_to_local)
        else:
            xb, yb, zb = x, y, z
            qx_b, qy_b, qz_b, qw_b = qx, qy, qz, qw
        quat_wxyz = [qw_b, qx_b, qy_b, qz_b]
        pos = [xb, yb, zb]
        out.append((pos, quat_wxyz))
    return out


def interpolate_and_move(arm: SingleArm, start_pos, start_quat_wxyz, target_pos, target_quat_wxyz, step_size=0.01, dt=0.04):
    """返回 True 表示正常完成，False 表示被用户中断"""
    p0 = np.asarray(start_pos, dtype=float)
    p1 = np.asarray(target_pos, dtype=float)
    dist = np.linalg.norm(p1 - p0)
    steps = max(int(np.ceil(dist / max(step_size, 1e-6))), 1)
    print(f'插值移动: 距离 {dist:.3f} m, 步数 {steps}, 每步平移约 {dist/steps:.3f} m')

    q0_w, q0_x, q0_y, q0_z = start_quat_wxyz
    q1_w, q1_x, q1_y, q1_z = target_quat_wxyz
    rot0 = R.from_quat([q0_x, q0_y, q0_z, q0_w])
    rot1 = R.from_quat([q1_x, q1_y, q1_z, q1_w])
    slerp = Slerp([0, 1], R.concatenate([rot0, rot1]))
    times = np.linspace(0.0, 1.0, steps + 1)
    rots = slerp(times)
    quats_xyzw = rots.as_quat()

    positions = np.linspace(p0, p1, steps + 1)
    for i in range(steps + 1):
        if is_quit_requested():
            print("插值移动被用户中断")
            return False
        pos = positions[i].tolist()
        q_xyzw = quats_xyzw[i]
        q_wxyz = [q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]]
        arm.set_end_effector_pose_quat_raw(pos=pos, quat=q_wxyz)
        time.sleep(dt)
    return True


def move_to_first_with_interp(arm: SingleArm, target_pos, target_quat_wxyz, step_size=0.01, dt=0.04):
    cur_pos, cur_quat = arm.get_ee_pose_quat()
    return interpolate_and_move(arm, cur_pos.tolist(), cur_quat.tolist(), target_pos, target_quat_wxyz, step_size=step_size, dt=dt)


def return_to_home_with_interp(arm: SingleArm, home_pos, home_quat_wxyz, step_size=0.01, dt=0.04):
    cur_pos, cur_quat = arm.get_ee_pose_quat()
    return interpolate_and_move(arm, cur_pos.tolist(), cur_quat.tolist(), home_pos, home_quat_wxyz, step_size=step_size, dt=dt)


def main():
    parser = argparse.ArgumentParser(description='单臂轨迹复现（startouch，按Q结束回放但仍绘图归位）')
    parser.add_argument('--left_file', type=str, default='/home/lumos/code/starnew/session_091619/right_hand_250801DR48FP26001130/Merged_Trajectory/merged_trajectory.txt')
    parser.add_argument('--replay_dir', type=str, default='')
    parser.add_argument('--left_clamp_file', type=str, default='/home/lumos/code/starnew/session_091619/right_hand_250801DR48FP26001130/Merged_Trajectory/Clamp_Data/clamp_data_tum.txt')
    parser.add_argument('--dt', type=float, default=0.02)
    parser.add_argument('--interp_step_size', type=float, default=0.001)
    parser.add_argument('--gripper', type=float, default=0.0)
    parser.add_argument('--clamp_max', type=float, default=90.0)
    parser.add_argument('--no_gripper', action='store_true')
    parser.add_argument('--dry_run', action='store_true')
    parser.add_argument('--debug_interp', action='store_true')
    parser.add_argument('--sim_current_pos', type=str, default="0.16,0.18,0.146,0.0,0.0,0.0,1.0")
    parser.add_argument('--left_can', type=str, default='can0')
    parser.add_argument('--no_transform', action='store_true')
    parser.add_argument('--base_x', type=float, default=0.35)
    parser.add_argument('--base_y', type=float, default=-0.0)
    parser.add_argument('--base_z', type=float, default=0.16)
    parser.add_argument('--base_roll', type=float, default=0.0)
    parser.add_argument('--base_pitch', type=float, default=0.0)
    parser.add_argument('--base_yaw', type=float, default=0.0)
    parser.add_argument('--no_plot', action='store_true')
    parser.add_argument('--plot_out', type=str, default='replay_xyz_compare.png')
    parser.add_argument('--plot_show', action='store_true')
    args = parser.parse_args()

    left_path = os.path.join(args.replay_dir, args.left_file)
    print(f"左臂轨迹文件: {left_path}")
    left_clamp_path = os.path.join(args.replay_dir, args.left_clamp_file)

    if not os.path.isfile(left_path):
        raise FileNotFoundError(f'轨迹文件不存在: {left_path}')

    left_traj = load_trajectory(left_path)
    if left_traj is None:
        raise ValueError('轨迹为空')

    T_base_to_local = None
    if not args.no_transform:
        T_base_to_local = build_T_base_to_local(args.base_x, args.base_y, args.base_z, args.base_roll, args.base_pitch, args.base_yaw)
        print(f'使用 local->base 变换: base=({args.base_x},{args.base_y},{args.base_z}), rpy=({args.base_roll},{args.base_pitch},{args.base_yaw})')
    else:
        print('未使用坐标变换')

    poses_left = trajectory_to_startouch(left_traj, T_base_to_local)
    n_steps = len(poses_left)
    print(f'轨迹步数: {n_steps}')

    gripper_left_arr = None
    if os.path.isfile(left_clamp_path):
        ts_l, vals_l = load_clamp_tum(left_clamp_path)
        gripper_left_arr = align_clamp_to_trajectory(left_traj[:, 0], ts_l, vals_l, clamp_max=args.clamp_max)
        if gripper_left_arr is not None:
            print(f'加载左夹爪: {left_clamp_path} points={len(ts_l)}')
    else:
        print(f'未找到夹爪文件: {left_clamp_path}')

    default_gripper = float(np.clip(args.gripper, 0.0, 1.0))

    if args.dry_run:
        if args.debug_interp:
            def parse_sim(s):
                parts = [float(x) for x in s.split(',')]
                if len(parts) != 7:
                    raise ValueError('sim current must be 7 floats: x,y,z,w,x,y,z')
                pos = parts[0:3]
                quat = parts[3:7]
                return pos, quat

            p0, q0 = poses_left[0]
            step_size = args.interp_step_size
            def calc_steps(p_cur, p_target):
                return max(int(np.ceil(np.linalg.norm(np.asarray(p_target) - np.asarray(p_cur)) / max(step_size, 1e-6))), 1)

            if args.sim_current_pos:
                cur_pos, cur_quat = parse_sim(args.sim_current_pos)
            else:
                cur_pos, cur_quat = p0, q0

            steps = calc_steps(cur_pos, p0)
            print(f'[debug_interp] 到首点插值步数: {steps} (step_size={step_size} m)')
            home_pos = [0.158, 0.28, 0.145]
            steps_home = calc_steps(p0, home_pos)
            print(f'[debug_interp] 首点->home 插值步数: {steps_home}')
            def sample_positions(p_cur, p_target, steps):
                pos_arr = np.linspace(np.asarray(p_cur), np.asarray(p_target), steps+1)
                idxs = [0, steps//2, steps]
                return [pos_arr[i].tolist() for i in idxs]
            print('[debug_interp] 示例插值点 (start, mid, end):', sample_positions(cur_pos, p0, steps))
            return
        else:
            for i in range(min(5, n_steps)):
                print(f'Step {i} pos={poses_left[i][0]} quat={poses_left[i][1]} G={gripper_left_arr[i] if gripper_left_arr is not None else default_gripper}')
            return

    # 启动退出监听线程
    start_quit_listener()
    print("提示：按 'Q' 键将立即结束轨迹回放，但会继续绘图并归位\n")

    left_arm = SingleArm(can_interface_=args.left_can)
    time.sleep(2)

    # 初始化记录数组
    cmd_xyz = np.zeros((n_steps, 3), dtype=float)
    cmd_rpy = np.zeros((n_steps, 3), dtype=float)
    act_xyz = np.zeros((n_steps, 3), dtype=float)
    act_rpy = np.zeros((n_steps, 3), dtype=float)
    executed_steps = 0  # 实际执行的步数

    try:
        # 平滑移动到第一个轨迹点
        p0, q0 = poses_left[0]
        print('平滑移动到首点...')
        if not move_to_first_with_interp(left_arm, p0, q0, step_size=args.interp_step_size, dt=args.dt):
            print("首点移动被用户中断，将跳过轨迹复现，直接绘图并归位")
            # 没有轨迹数据，直接跳转到归位
        else:
            print('开始复现轨迹...')
            with tqdm(total=n_steps, desc='复现轨迹', unit='步') as pbar:
                for i in range(n_steps):
                    if is_quit_requested():
                        print("\n用户请求退出，停止复现轨迹")
                        break

                    pl, ql = poses_left[i]
                    left_arm.set_end_effector_pose_quat_raw(pos=pl, quat=ql)

                    qL_w, qL_x, qL_y, qL_z = ql
                    cmd_rpy[i] = R.from_quat([qL_x, qL_y, qL_z, qL_w]).as_euler('xyz')
                    cmd_xyz[i] = pl

                    if not args.no_gripper:
                        gl = float(gripper_left_arr[i]) if gripper_left_arr is not None else default_gripper
                        left_arm.setGripperPosition(gl)

                    time.sleep(args.dt)

                    pos_l, rpy_l = left_arm.get_ee_pose_euler()
                    act_xyz[i] = np.asarray(pos_l, dtype=float).reshape(3)
                    act_rpy[i] = np.asarray(rpy_l, dtype=float).reshape(3)
                    executed_steps = i + 1
                    pbar.update(1)

        # 绘图（使用实际记录的步数）
        if not args.no_plot and executed_steps > 0:
            print(f"正在绘制位姿对比图（已执行 {executed_steps} 步）...")
            plot_replay_xyz_rpy_single(
                cmd_xyz=cmd_xyz[:executed_steps],
                act_xyz=act_xyz[:executed_steps],
                cmd_rpy=cmd_rpy[:executed_steps],
                act_rpy=act_rpy[:executed_steps],
                save_path=args.plot_out,
                show=args.plot_show,
            )
        elif not args.no_plot and executed_steps == 0:
            print("没有执行任何轨迹步，跳过绘图")

        # 归位到 home（同样支持 Q 中断）
        left_arm.go_home()
        print('归位到 home...')
        time.sleep(2)
        # if not return_to_home_with_interp(left_arm, home_pos, home_quat, step_size=args.interp_step_size, dt=args.dt):
        #     print("归位移动被用户中断")
        # else:
        print('归位完成')
    finally:
        left_arm.cleanup()
        _clear_quit_flag()


if __name__ == '__main__':
    main()
