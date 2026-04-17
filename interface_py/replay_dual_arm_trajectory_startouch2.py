#!/usr/bin/env python3
"""
基于 startouch 控制的双臂轨迹复现脚本（改编自 replay_dual_arm_trajectory.py）

轨迹格式与原脚本一致：每行 timestamp x y z qx qy qz qw（xyzw）
夹爪格式：TUM 每行 timestamp value（0~clamp_max），映射为 startouch 的 0..1
"""
import argparse
import time
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
from tqdm import tqdm
import os

from startouchclass import SingleArm, euler_to_quaternion


def plot_replay_xyz_rpy(
    cmd_L: np.ndarray,
    cmd_R: np.ndarray,
    act_L: np.ndarray,
    act_R: np.ndarray,
    cmd_L_rpy: np.ndarray,
    cmd_R_rpy: np.ndarray,
    act_L_rpy: np.ndarray,
    act_R_rpy: np.ndarray,
    save_path: str | None = None,
    show: bool = False,
) -> None:
    """六张折线图：X、Y、Z、R、P、Y，每条图内为左右臂指令与实际反馈。"""
    import matplotlib

    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    n = cmd_L.shape[0]
    steps = np.arange(n)
    labels = ("X (m)", "Y (m)", "Z (m)", "Roll (rad)", "Pitch (rad)", "Yaw (rad)")
    fig, axes = plt.subplots(6, 1, figsize=(10, 16), sharex=True)
    
    data_list = [
        (cmd_L[:, 0], cmd_R[:, 0], act_L[:, 0], act_R[:, 0]),
        (cmd_L[:, 1], cmd_R[:, 1], act_L[:, 1], act_R[:, 1]),
        (cmd_L[:, 2], cmd_R[:, 2], act_L[:, 2], act_R[:, 2]),
        (cmd_L_rpy[:, 0], cmd_R_rpy[:, 0], act_L_rpy[:, 0], act_R_rpy[:, 0]),
        (cmd_L_rpy[:, 1], cmd_R_rpy[:, 1], act_L_rpy[:, 1], act_R_rpy[:, 1]),
        (cmd_L_rpy[:, 2], cmd_R_rpy[:, 2], act_L_rpy[:, 2], act_R_rpy[:, 2]),
    ]
    
    for ax, data_tup, ylab in zip(axes, data_list, labels):
        cL, cR, aL, aR = data_tup
        ax.plot(steps, cL, color="tab:blue", linestyle="-", label="left traj", linewidth=1.2)
        ax.plot(steps, cR, color="tab:orange", linestyle="-", label="right traj", linewidth=1.2)
        ax.plot(steps, aL, color="tab:blue", linestyle="--", label="left state", linewidth=1.0, alpha=0.9)
        ax.plot(steps, aR, color="tab:orange", linestyle="--", label="right state", linewidth=1.0, alpha=0.9)
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


def interpolate_and_move_both(armL: SingleArm, armR: SingleArm,
                             start_pos_L, start_quat_wxyz_L, target_pos_L, target_quat_wxyz_L,
                             start_pos_R, start_quat_wxyz_R, target_pos_R, target_quat_wxyz_R,
                             step_size=0.01, dt=0.04):
    """同步插值并同时发送左右臂位姿，步数由两臂最大距离决定。"""
    p0L = np.asarray(start_pos_L, dtype=float)
    p1L = np.asarray(target_pos_L, dtype=float)
    p0R = np.asarray(start_pos_R, dtype=float)
    p1R = np.asarray(target_pos_R, dtype=float)
    distL = np.linalg.norm(p1L - p0L)
    distR = np.linalg.norm(p1R - p0R)
    steps = max(int(np.ceil(distL / max(step_size, 1e-6))), int(np.ceil(distR / max(step_size, 1e-6))), 1)
    print(f'同步插值移动: 左距 {distL:.3f} m, 右距 {distR:.3f} m, 步数 {steps}, 每步平移约 左{distL/steps:.3f} 右{distR/steps:.3f} m')

    # prepare rotations
    q0_wL, q0_xL, q0_yL, q0_zL = start_quat_wxyz_L
    q1_wL, q1_xL, q1_yL, q1_zL = target_quat_wxyz_L
    rot0L = R.from_quat([q0_xL, q0_yL, q0_zL, q0_wL])
    rot1L = R.from_quat([q1_xL, q1_yL, q1_zL, q1_wL])
    slerpL = Slerp([0, 1], R.concatenate([rot0L, rot1L]))

    q0_wR, q0_xR, q0_yR, q0_zR = start_quat_wxyz_R
    q1_wR, q1_xR, q1_yR, q1_zR = target_quat_wxyz_R
    rot0R = R.from_quat([q0_xR, q0_yR, q0_zR, q0_wR])
    rot1R = R.from_quat([q1_xR, q1_yR, q1_zR, q1_wR])
    slerpR = Slerp([0, 1], R.concatenate([rot0R, rot1R]))

    times = np.linspace(0.0, 1.0, steps + 1)
    rotsL = slerpL(times).as_quat()
    rotsR = slerpR(times).as_quat()
    posL_arr = np.linspace(p0L, p1L, steps + 1)
    posR_arr = np.linspace(p0R, p1R, steps + 1)

    for i in range(steps + 1):
        posL = posL_arr[i].tolist()
        posR = posR_arr[i].tolist()
        qL_xyzw = rotsL[i]
        qR_xyzw = rotsR[i]
        qL_wxyz = [qL_xyzw[3], qL_xyzw[0], qL_xyzw[1], qL_xyzw[2]]
        qR_wxyz = [qR_xyzw[3], qR_xyzw[0], qR_xyzw[1], qR_xyzw[2]]
        armL.set_end_effector_pose_quat_raw(pos=posL, quat=qL_wxyz)
        armR.set_end_effector_pose_quat_raw(pos=posR, quat=qR_wxyz)
        time.sleep(dt)


def move_both_to_first_with_interp(armL: SingleArm, armR: SingleArm, target_pos_L, target_quat_wxyz_L, target_pos_R, target_quat_wxyz_R, step_size=0.01, dt=0.04):
    curL_pos, curL_quat = armL.get_ee_pose_quat()
    curR_pos, curR_quat = armR.get_ee_pose_quat()
    interpolate_and_move_both(armL, armR, curL_pos.tolist(), curL_quat.tolist(), target_pos_L, target_quat_wxyz_L,
                             curR_pos.tolist(), curR_quat.tolist(), target_pos_R, target_quat_wxyz_R,
                             step_size=step_size, dt=dt)


def return_both_to_home_with_interp(armL: SingleArm, armR: SingleArm, home_pos, home_quat_wxyz, step_size=0.01, dt=0.04):
    curL_pos, curL_quat = armL.get_ee_pose_quat()
    curR_pos, curR_quat = armR.get_ee_pose_quat()
    interpolate_and_move_both(armL, armR, curL_pos.tolist(), curL_quat.tolist(), home_pos, home_quat_wxyz,
                             curR_pos.tolist(), curR_quat.tolist(), home_pos, home_quat_wxyz,
                             step_size=step_size, dt=dt)


def main():
    parser = argparse.ArgumentParser(description='双臂轨迹复现（startouch）')
    parser.add_argument('--left_file', type=str, default='/home/zgy/FastUMI_Touch_test/data_trajectory/bottle_handover/left_hand_250801DR48FP26001303/Merged_Trajectory/merged_trajectory.txt')
    parser.add_argument('--right_file', type=str, default='/home/zgy/FastUMI_Touch_test/data_trajectory/bottle_handover/right_hand_250801DR48FP25002350/Merged_Trajectory/merged_trajectory.txt')
    parser.add_argument('--replay_dir', type=str, default='', help='轨迹目录')
    parser.add_argument('--left_clamp_file', type=str, default='/home/lumos/openpi/session_091619/left_hand_250801DR48FP25002794/Clamp_Data/clamp_data_tum.txt')
    parser.add_argument('--right_clamp_file', type=str, default='/home/lumos/openpi/session_091619/right_hand_250801DR48FP26001130/Clamp_Data/clamp_data_tum.txt')
    parser.add_argument('--dt', type=float, default=0.04)
    parser.add_argument('--interp_step_size', type=float, default=0.001, help='插值的最大平移步长(m)')
    parser.add_argument('--gripper', type=float, default=0.0, help='无夹爪文件时的 gripper 开度 0..1')
    parser.add_argument('--clamp_max', type=float, default=90.0)
    parser.add_argument('--no_gripper', action='store_true')
    parser.add_argument('--dry_run', action='store_true')
    parser.add_argument('--debug_interp', action='store_true', help='dry_run 下打印插值步数和示例点')
    parser.add_argument('--sim_current_left', type=str, default="0.16,0.18,0.146,0.0,0.0,0.0,1.0", help='模拟左臂当前位姿: x,y,z,w,x,y,z')
    parser.add_argument('--sim_current_right', type=str, default="0.16,0.18,0.146,0.0,0.0,0.0,1.0", help='模拟右臂当前位姿: x,y,z,w,x,y,z')
    parser.add_argument('--left_can', type=str, default='can0')
    parser.add_argument('--right_can', type=str, default='can1')
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
    right_path = os.path.join(args.replay_dir, args.right_file)
    print(right_path)
    left_clamp_path = os.path.join(args.replay_dir, args.left_clamp_file)
    right_clamp_path = os.path.join(args.replay_dir, args.right_clamp_file)

    if not os.path.isfile(left_path) or not os.path.isfile(right_path):
        raise FileNotFoundError('轨迹文件不存在')

    left_traj = load_trajectory(left_path)
    right_traj = load_trajectory(right_path)
    if left_traj is None or right_traj is None:
        raise ValueError('轨迹为空')

    T_base_to_local = None
    if not args.no_transform:
        T_base_to_local = build_T_base_to_local(args.base_x, args.base_y, args.base_z, args.base_roll, args.base_pitch, args.base_yaw)
        print(f'使用 local->base 变换: base=({args.base_x},{args.base_y},{args.base_z}), rpy=({args.base_roll},{args.base_pitch},{args.base_yaw})')
    else:
        print('未使用坐标变换')

    poses_left = trajectory_to_startouch(left_traj, T_base_to_local)
    poses_right = trajectory_to_startouch(right_traj, T_base_to_local)
    n_steps = min(len(poses_left), len(poses_right))
    print(f'左 {len(poses_left)} 右 {len(poses_right)} 步数 {n_steps}')

    gripper_left_arr = None
    gripper_right_arr = None
    if os.path.isfile(left_clamp_path):
        ts_l, vals_l = load_clamp_tum(left_clamp_path)
        gripper_left_arr = align_clamp_to_trajectory(left_traj[:, 0], ts_l, vals_l, clamp_max=args.clamp_max)
        if gripper_left_arr is not None:
            print(f'加载左夹爪: {left_clamp_path} points={len(ts_l)}')
    if os.path.isfile(right_clamp_path):
        ts_r, vals_r = load_clamp_tum(right_clamp_path)
        gripper_right_arr = align_clamp_to_trajectory(right_traj[:, 0], ts_r, vals_r, clamp_max=args.clamp_max)
        if gripper_right_arr is not None:
            print(f'加载右夹爪: {right_clamp_path} points={len(ts_r)}')

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

            # first poses
            pL, qL = poses_left[0]
            pR, qR = poses_right[0]
            step_size = args.interp_step_size
            def calc_steps(p_cur, p_target):
                return max(int(np.ceil(np.linalg.norm(np.asarray(p_target) - np.asarray(p_cur)) / max(step_size, 1e-6))), 1)

            if args.sim_current_left:
                curL_pos, curL_quat = parse_sim(args.sim_current_left)
            else:
                curL_pos, curL_quat = pL, qL
            if args.sim_current_right:
                curR_pos, curR_quat = parse_sim(args.sim_current_right)
            else:
                curR_pos, curR_quat = pR, qR

            steps_L = calc_steps(curL_pos, pL)
            steps_R = calc_steps(curR_pos, pR)
            print(f'[debug_interp] 左臂到首点插值步数: {steps_L} (step_size={step_size} m)')
            print(f'[debug_interp] 右臂到首点插值步数: {steps_R} (step_size={step_size} m)')

            # 返回 home 的步数
            home_pos = [0.158, 0.28, 0.145]
            steps_L_home = calc_steps(pL, home_pos)
            steps_R_home = calc_steps(pR, home_pos)
            print(f'[debug_interp] 左臂首点->home 插值步数: {steps_L_home}')
            print(f'[debug_interp] 右臂首点->home 插值步数: {steps_R_home}')

            # show example few interpolated positions (first/last/med)
            def sample_positions(p_cur, p_target, steps):
                pos_arr = np.linspace(np.asarray(p_cur), np.asarray(p_target), steps+1)
                idxs = [0, steps//2, steps]
                return [pos_arr[i].tolist() for i in idxs]

            print('[debug_interp] 左臂示例插值点 (start, mid, end):', sample_positions(curL_pos, pL, steps_L))
            print('[debug_interp] 右臂示例插值点 (start, mid, end):', sample_positions(curR_pos, pR, steps_R))
            return
        else:
            for i in range(min(5, n_steps)):
                print(f'Step {i} L pos={poses_left[i][0]} quat={poses_left[i][1]} G={gripper_left_arr[i] if gripper_left_arr is not None else default_gripper}')
            return

    # connect startouch arms
    left_arm = SingleArm(can_interface_=args.left_can)
    right_arm = SingleArm(can_interface_=args.right_can)
    time.sleep(2)

    try:
        # 平滑移动到第一个轨迹点（插值）
        p0, q0 = poses_left[0]
        p0r, q0r = poses_right[0]
        print('平滑移动到首点 (双臂同步)...')
        move_both_to_first_with_interp(left_arm, right_arm, p0, q0, p0r, q0r, step_size=args.interp_step_size, dt=args.dt)
        print('开始复现轨迹...')
        cmd_L_xyz = np.array([poses_left[i][0] for i in range(n_steps)], dtype=float)
        cmd_R_xyz = np.array([poses_right[i][0] for i in range(n_steps)], dtype=float)
        cmd_L_rpy = np.zeros((n_steps, 3), dtype=float)
        cmd_R_rpy = np.zeros((n_steps, 3), dtype=float)
        act_L_xyz = np.zeros((n_steps, 3), dtype=float)
        act_R_xyz = np.zeros((n_steps, 3), dtype=float)
        act_L_rpy = np.zeros((n_steps, 3), dtype=float)
        act_R_rpy = np.zeros((n_steps, 3), dtype=float)

        for i in tqdm(range(n_steps), desc='复现轨迹', unit='步'):
            pl, ql = poses_left[i]
            pr, qr = poses_right[i]
            left_arm.set_end_effector_pose_quat_raw(pos=pl, quat=ql)
            right_arm.set_end_effector_pose_quat_raw(pos=pr, quat=qr)
            
            # quat is [w, x, y, z], scipy R.from_quat expects [x, y, z, w]
            qL_w, qL_x, qL_y, qL_z = ql
            cmd_L_rpy[i] = R.from_quat([qL_x, qL_y, qL_z, qL_w]).as_euler('xyz')
            qR_w, qR_x, qR_y, qR_z = qr
            cmd_R_rpy[i] = R.from_quat([qR_x, qR_y, qR_z, qR_w]).as_euler('xyz')
            
            if not args.no_gripper:
                gl = float(gripper_left_arr[i]) if gripper_left_arr is not None else default_gripper
                gr = float(gripper_right_arr[i]) if gripper_right_arr is not None else default_gripper
                left_arm.setGripperPosition(gl)
                right_arm.setGripperPosition(gr)
            time.sleep(args.dt)
            pos_l, rpy_l = left_arm.get_ee_pose_euler()
            pos_r, rpy_r = right_arm.get_ee_pose_euler()
            act_L_xyz[i] = np.asarray(pos_l, dtype=float).reshape(3)
            act_R_xyz[i] = np.asarray(pos_r, dtype=float).reshape(3)
            act_L_rpy[i] = np.asarray(rpy_l, dtype=float).reshape(3)
            act_R_rpy[i] = np.asarray(rpy_r, dtype=float).reshape(3)

        print('轨迹复现结束')
        if not args.no_plot:
            plot_replay_xyz_rpy(
                cmd_L=cmd_L_xyz,
                cmd_R=cmd_R_xyz,
                act_L=act_L_xyz,
                act_R=act_R_xyz,
                cmd_L_rpy=cmd_L_rpy,
                cmd_R_rpy=cmd_R_rpy,
                act_L_rpy=act_L_rpy,
                act_R_rpy=act_R_rpy,
                save_path=args.plot_out,
                show=args.plot_show,
            )
        # 复现结束后归位到 home（平滑）
        home_pos = [0.2, 0.0, 0.17]
        home_euler_rad = np.deg2rad([0.0, 0.0, 0.0])
        home_quat = euler_to_quaternion(home_euler_rad[0], home_euler_rad[1], home_euler_rad[2]).tolist()
        print('归位到 home (双臂同步)...')
        return_both_to_home_with_interp(left_arm, right_arm, home_pos, home_quat, step_size=args.interp_step_size, dt=args.dt)
        print('归位完成')
    finally:
        left_arm.cleanup()
        right_arm.cleanup()


if __name__ == '__main__':
    main()
