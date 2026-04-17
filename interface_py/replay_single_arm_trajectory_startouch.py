#!/usr/bin/env python3
"""
基于 startouch 控制的单臂轨迹复现脚本（改编自 replay_dual_arm_trajectory_startouch2.py）

轨迹格式：每行 timestamp x y z qx qy qz qw（xyzw）
夹爪格式：TUM 每行 timestamp value（0~clamp_max），映射为 startouch 的 0..1
"""
import argparse
import time
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
from tqdm import tqdm
import os

from startouchclass import SingleArm, euler_to_quaternion


def plot_replay_xyz_rpy_single(
    cmd_xyz: np.ndarray,
    act_xyz: np.ndarray,
    cmd_rpy: np.ndarray,
    act_rpy: np.ndarray,
    save_path: str | None = None,
    show: bool = False,
) -> None:
    """六张折线图：X、Y、Z、R、P、Y，每条图内为指令与实际反馈（单臂）。"""
    import matplotlib

    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    n = cmd_xyz.shape[0]
    steps = np.arange(n)
    labels = ("X (m)", "Y (m)", "Z (m)", "Roll (rad)", "Pitch (rad)", "Yaw (rad)")
    fig, axes = plt.subplots(6, 1, figsize=(10, 16), sharex=True)
    
    data_list = [
        (cmd_xyz[:, 0], act_xyz[:, 0]),
        (cmd_xyz[:, 1], act_xyz[:, 1]),
        (cmd_xyz[:, 2], act_xyz[:, 2]),
        (cmd_rpy[:, 0], act_rpy[:, 0]),
        (cmd_rpy[:, 1], act_rpy[:, 1]),
        (cmd_rpy[:, 2], act_rpy[:, 2]),
    ]
    
    for ax, (cmd, act), ylab in zip(axes, data_list, labels):
        ax.plot(steps, cmd, color="tab:blue", linestyle="-", label="traj", linewidth=1.2)
        ax.plot(steps, act, color="tab:red", linestyle="--", label="state", linewidth=1.0, alpha=0.9)
        ax.set_ylabel(ylab)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", fontsize=8)
    axes[-1].set_xlabel("轨迹步索引")
    fig.suptitle("末端位姿：指令 vs get_ee_pose_euler() 反馈")
    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=150)
        print(f"已保存位姿折线图: {save_path}")
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
    # map 0~clamp_max -> startouch 1.0~0.0 (原逻辑为 255*(clamp_max-val)/clamp_max)
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
    """返回列表，每项 (pos3, quat_wxyz)
    输入轨迹为 timestamp,x,y,z,qx,qy,qz,qw (xyzw)
    """
    out = []
    for row in traj:
        t, x, y, z, qx, qy, qz, qw = row[:8]
        if T_base_to_local is not None:
            xb, yb, zb, qx_b, qy_b, qz_b, qw_b = transform_to_base_quat(x, y, z, qx, qy, qz, qw, T_base_to_local)
        else:
            xb, yb, zb = x, y, z
            qx_b, qy_b, qz_b, qw_b = qx, qy, qz, qw
        # startouch expects quaternion as [w,x,y,z]
        quat_wxyz = [qw_b, qx_b, qy_b, qz_b]
        pos = [xb, yb, zb]
        out.append((pos, quat_wxyz))
    return out


def interpolate_and_move(arm: SingleArm, start_pos, start_quat_wxyz, target_pos, target_quat_wxyz, step_size=0.01, dt=0.04):
    """按线性位移 + 球面线性插值旋转插值，分步发送位姿到 arm。
    - start_pos/target_pos: [x,y,z] (米)
    - quaternions in w,x,y,z order
    - step_size: 每步最大平移 (米)，实际步数 = ceil(dist/step_size)
    - dt: 每步之间的睡眠秒数
    """
    p0 = np.asarray(start_pos, dtype=float)
    p1 = np.asarray(target_pos, dtype=float)
    dist = np.linalg.norm(p1 - p0)
    steps = max(int(np.ceil(dist / max(step_size, 1e-6))), 1)
    print(f'插值移动: 距离 {dist:.3f} m, 步数 {steps}, 每步平移约 {dist/steps:.3f} m')

    # prepare rotation slerp: scipy expects quaternions as [x,y,z,w]
    q0_w, q0_x, q0_y, q0_z = start_quat_wxyz
    q1_w, q1_x, q1_y, q1_z = target_quat_wxyz
    rot0 = R.from_quat([q0_x, q0_y, q0_z, q0_w])
    rot1 = R.from_quat([q1_x, q1_y, q1_z, q1_w])
    slerp = Slerp([0, 1], R.concatenate([rot0, rot1]))
    times = np.linspace(0.0, 1.0, steps + 1)
    rots = slerp(times)
    quats_xyzw = rots.as_quat()  # returns [x,y,z,w]

    positions = np.linspace(p0, p1, steps + 1)
    for i in range(steps + 1):
        pos = positions[i].tolist()
        q_xyzw = quats_xyzw[i]
        # convert back to w,x,y,z
        q_wxyz = [q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]]
        arm.set_end_effector_pose_quat_raw(pos=pos, quat=q_wxyz)
        time.sleep(dt)


def move_to_first_with_interp(arm: SingleArm, target_pos, target_quat_wxyz, step_size=0.01, dt=0.04):
    """从当前末端位姿平滑移动到目标位姿（插值）。"""
    cur_pos, cur_quat = arm.get_ee_pose_quat()
    # cur_quat is returned as [w,x,y,z]
    interpolate_and_move(arm, cur_pos.tolist(), cur_quat.tolist(), target_pos, target_quat_wxyz, step_size=step_size, dt=dt)


def return_to_home_with_interp(arm: SingleArm, home_pos, home_quat_wxyz, step_size=0.01, dt=0.04):
    interpolate_and_move(arm, arm.get_ee_pose_quat()[0].tolist(), arm.get_ee_pose_quat()[1].tolist(), home_pos, home_quat_wxyz, step_size=step_size, dt=dt)


def main():
    parser = argparse.ArgumentParser(description='单臂轨迹复现（startouch）')
    parser.add_argument('--left_file', type=str, default='/home/zgy/FastUMI_Touch_test/data_trajectory/bottle_handover/left_hand_250801DR48FP26001303/Merged_Trajectory/merged_trajectory.txt',
                        help='左臂轨迹文件路径')
    parser.add_argument('--replay_dir', type=str, default='', help='轨迹目录（与文件拼接）')
    parser.add_argument('--left_clamp_file', type=str, default='/home/zgy/FastUMI_Touch_test/data_trajectory/bottle_handover/left_hand_250801DR48FP26001303/Clamp_Data/clamp_data_tum.txt',
                        help='左臂夹爪数据文件路径（TUM格式）')
    parser.add_argument('--dt', type=float, default=0.04)
    parser.add_argument('--interp_step_size', type=float, default=0.001, help='插值的最大平移步长(m)')
    parser.add_argument('--gripper', type=float, default=0.0, help='无夹爪文件时的 gripper 开度 0..1')
    parser.add_argument('--clamp_max', type=float, default=90.0)
    parser.add_argument('--no_gripper', action='store_true')
    parser.add_argument('--dry_run', action='store_true')
    parser.add_argument('--debug_interp', action='store_true', help='dry_run 下打印插值步数和示例点')
    parser.add_argument('--sim_current_pos', type=str, default="0.16,0.18,0.146,0.0,0.0,0.0,1.0",
                        help='模拟当前位姿: x,y,z,w,x,y,z')
    parser.add_argument('--left_can', type=str, default='can0', help='左臂CAN接口')
    parser.add_argument('--no_transform', action='store_true')
    parser.add_argument('--base_x', type=float, default=0.3)
    parser.add_argument('--base_y', type=float, default=-0.0)
    parser.add_argument('--base_z', type=float, default=0.16)
    parser.add_argument('--base_roll', type=float, default=0.0)
    parser.add_argument('--base_pitch', type=float, default=0.0)
    parser.add_argument('--base_yaw', type=float, default=0.0)
    parser.add_argument('--no_plot', action='store_true', help='不复现结束后绘制 XYZ 折线图')
    parser.add_argument('--plot_out', type=str, default='replay_xyz_compare.png', help='折线图保存路径')
    parser.add_argument('--plot_show', action='store_true', help='保存后尝试弹出 matplotlib 窗口')
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

            # first pose
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

            # 返回 home 的步数
            home_pos = [0.158, 0.28, 0.145]
            steps_home = calc_steps(p0, home_pos)
            print(f'[debug_interp] 首点->home 插值步数: {steps_home}')

            # show example few interpolated positions (first/last/med)
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

    # connect startouch arm
    left_arm = SingleArm(can_interface_=args.left_can)
    time.sleep(2)

    try:
        # 平滑移动到第一个轨迹点（插值）
        p0, q0 = poses_left[0]
        print('平滑移动到首点...')
        move_to_first_with_interp(left_arm, p0, q0, step_size=args.interp_step_size, dt=args.dt)
        print('开始复现轨迹...')
        cmd_xyz = np.array([poses_left[i][0] for i in range(n_steps)], dtype=float)
        cmd_rpy = np.zeros((n_steps, 3), dtype=float)
        act_xyz = np.zeros((n_steps, 3), dtype=float)
        act_rpy = np.zeros((n_steps, 3), dtype=float)

        for i in tqdm(range(n_steps), desc='复现轨迹', unit='步'):
            pl, ql = poses_left[i]
            left_arm.set_end_effector_pose_quat_raw(pos=pl, quat=ql)
            
            # quat is [w, x, y, z], scipy R.from_quat expects [x, y, z, w]
            qL_w, qL_x, qL_y, qL_z = ql
            cmd_rpy[i] = R.from_quat([qL_x, qL_y, qL_z, qL_w]).as_euler('xyz')
            
            if not args.no_gripper:
                gl = float(gripper_left_arr[i]) if gripper_left_arr is not None else default_gripper
                left_arm.setGripperPosition(gl)
            time.sleep(args.dt)
            pos_l, rpy_l = left_arm.get_ee_pose_euler()
            act_xyz[i] = np.asarray(pos_l, dtype=float).reshape(3)
            act_rpy[i] = np.asarray(rpy_l, dtype=float).reshape(3)

        print('轨迹复现结束')
        if not args.no_plot:
            plot_replay_xyz_rpy_single(
                cmd_xyz=cmd_xyz,
                act_xyz=act_xyz,
                cmd_rpy=cmd_rpy,
                act_rpy=act_rpy,
                save_path=args.plot_out,
                show=args.plot_show,
            )
        # 复现结束后归位到 home（平滑）
        home_pos = [0.2, 0.0, 0.17]
        home_euler_rad = np.deg2rad([0.0, 0.0, 0.0])
        home_quat = euler_to_quaternion(home_euler_rad[0], home_euler_rad[1], home_euler_rad[2]).tolist()
        print('归位到 home...')
        return_to_home_with_interp(left_arm, home_pos, home_quat, step_size=args.interp_step_size, dt=args.dt)
        print('归位完成')
    finally:
        left_arm.cleanup()


if __name__ == '__main__':
    main()