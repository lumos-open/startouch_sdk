#!/usr/bin/env python3
"""
基于 startouch 控制的单臂轨迹复现脚本（优化版）
- 关节1-6: 400Hz (dt=0.0025s)
- 只控前6个关节，不控夹爪
- 保证位置、速度、加速度平滑
- 中间轨迹保持原始数据
- 头尾到HOME位平滑插值
- 支持按Q安全退出
"""
import argparse
import time
import sys
import threading
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from tqdm import tqdm
import os

from startouchclass import SingleArm

# ---------- 全局变量 ----------
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

# ---------- 键盘监听 ----------
def start_quit_listener():
    def listener():
        while not is_quit_requested():
            try:
                if sys.stdin.read(1).lower() == 'q':
                    _set_quit_flag()
                    print("\n[退出] 检测到 Q 键，将结束回放...")
                    break
            except:
                pass
            time.sleep(0.05)
    t = threading.Thread(target=listener, daemon=True)
    t.start()

# ---------- 数据加载 ----------
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

# ---------- 坐标变换 ----------
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
    qx_b, qy_b, qz_b, qw_b = rotation_base.as_quat()
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

# ---------- 平滑插值函数 ----------
def smooth_interpolate_positions(start_pos, end_pos, start_quat, end_quat, total_time, dt=0.0025, accel_time=0.2):
    """
    生成平滑的轨迹点，确保位置、速度、加速度连续
    使用S曲线加速度规划
    """
    p0 = np.array(start_pos)
    p1 = np.array(end_pos)
    dist = np.linalg.norm(p1 - p0)
    
    # 计算最小时间（基于最大速度和加速度）
    max_vel = 0.5  # m/s
    max_accel = 1.0  # m/s²
    min_time = max(2 * dist / max_vel, np.sqrt(4 * dist / max_accel))
    actual_time = max(total_time, min_time)
    
    # S曲线规划
    accel_phase = min(accel_time, actual_time / 2)
    const_vel_time = actual_time - 2 * accel_phase
    
    # 时间点
    t_accel = np.linspace(0, accel_phase, int(accel_phase / dt) + 1)
    t_const = np.linspace(accel_phase, accel_phase + const_vel_time, int(const_vel_time / dt) + 1)
    t_decel = np.linspace(accel_phase + const_vel_time, actual_time, int(accel_phase / dt) + 1)
    t_total = np.concatenate([t_accel, t_const[1:], t_decel[1:]])
    
    # 位置规划
    def s_curve_pos(t, t_total):
        if t < accel_phase:
            # 加速阶段
            s = (t / accel_phase) ** 2 * (3 - 2 * (t / accel_phase))
        elif t < accel_phase + const_vel_time:
            # 匀速阶段
            s = 1.0
        else:
            # 减速阶段
            u = (actual_time - t) / accel_phase
            s = u ** 2 * (3 - 2 * u)
        return s
    
    s_vals = np.array([s_curve_pos(t, actual_time) for t in t_total])
    positions = p0 + s_vals[:, np.newaxis] * (p1 - p0)
    
    # 姿态插值
    rot0 = R.from_quat([start_quat[1], start_quat[2], start_quat[3], start_quat[0]])  # xyzw to wxyz
    rot1 = R.from_quat([end_quat[1], end_quat[2], end_quat[3], end_quat[0]])
    slerp = Slerp([0, 1], R.concatenate([rot0, rot1]))
    s_norm = s_vals / s_vals[-1] if s_vals[-1] > 0 else np.zeros_like(s_vals)
    rots = slerp(s_norm)
    quats_xyzw = rots.as_quat()
    quats_wxyz = np.column_stack([quats_xyzw[:, 3], quats_xyzw[:, 0], quats_xyzw[:, 1], quats_xyzw[:, 2]])
    
    return positions, quats_wxyz, t_total

# ---------- 移动函数 ----------
def move_smooth(arm, target_pos, target_quat, total_time=2.0, dt=0.0025):
    """平滑移动到目标位置"""
    current_pos, current_quat = arm.get_ee_pose_quat()
    positions, quats, times = smooth_interpolate_positions(
        current_pos, target_pos, current_quat, target_quat, total_time, dt
    )
    
    for pos, quat in zip(positions, quats):
        if is_quit_requested():
            return False
        arm.set_end_effector_pose_quat_raw(pos=pos.tolist(), quat=quat.tolist())
        time.sleep(dt)
    return True

# ---------- 轨迹复现 ----------
def replay_trajectory_smooth(arm, poses, dt=0.0025):
    """复现轨迹，保持原始数据但确保时间同步"""
    n_steps = len(poses)
    cmd_xyz = np.zeros((n_steps, 3))
    cmd_rpy = np.zeros((n_steps, 3))
    act_xyz = np.zeros((n_steps, 3))
    act_rpy = np.zeros((n_steps, 3))
    
    executed_steps = 0
    
    with tqdm(total=n_steps, desc='复现轨迹', unit='步') as pbar:
        for i in range(n_steps):
            if is_quit_requested():
                break
            
            pos, quat = poses[i]
            arm.set_end_effector_pose_quat_raw(pos=pos, quat=quat)
            
            # 记录命令值
            q_w, q_x, q_y, q_z = quat
            cmd_rpy[i] = R.from_quat([q_x, q_y, q_z, q_w]).as_euler('xyz')
            cmd_xyz[i] = pos
            
            time.sleep(dt)
            
            # 记录实际值
            act_pos, act_rpy_deg = arm.get_ee_pose_euler()
            act_xyz[i] = np.array(act_pos)
            act_rpy[i] = np.radians(act_rpy_deg)  # 转换为弧度
            
            executed_steps = i + 1
            pbar.update(1)
    
    return cmd_xyz[:executed_steps], cmd_rpy[:executed_steps], act_xyz[:executed_steps], act_rpy[:executed_steps]

# ---------- 绘图 ----------
def plot_results(cmd_xyz, act_xyz, cmd_rpy, act_rpy, save_path=None, show=False):
    """绘制误差图"""
    err_xyz = (cmd_xyz - act_xyz) * 100.0  # m to cm
    err_rpy = cmd_rpy - act_rpy  # rad
    
    n = cmd_xyz.shape[0]
    steps = np.arange(n)
    
    labels = [
        "X Error (cm)", "Y Error (cm)", "Z Error (cm)",
        "Roll Error (rad)", "Pitch Error (rad)", "Yaw Error (rad)"
    ]
    
    fig, axes = plt.subplots(6, 1, figsize=(12, 16), sharex=True)
    
    data_list = [
        err_xyz[:, 0], err_xyz[:, 1], err_xyz[:, 2],
        err_rpy[:, 0], err_rpy[:, 1], err_rpy[:, 2]
    ]
    
    for ax, data, label in zip(axes, data_list, labels):
        ax.plot(steps, data, 'b-', linewidth=1.5)
        ax.axhline(0, color='r', linestyle='--', linewidth=1)
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3)
    
    axes[-1].set_xlabel("Step")
    fig.suptitle("End-Effector Pose Error (Command - Actual)")
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved plot to: {save_path}")
    
    if show:
        plt.show()
    
    plt.close()

# ---------- 主函数 ----------
def main():
    parser = argparse.ArgumentParser(description='单臂轨迹复现（优化版，400Hz关节控制）')
    parser.add_argument('--trajectory_file', type=str, default='/home/lumos/code/FastTouchV2/cccc/starnew/session_091619/right_hand_250801DR48FP26001130/Merged_Trajectory/merged_trajectory.txt', help='轨迹文件路径')
    parser.add_argument('--replay_dir', type=str, default='')
    parser.add_argument('--left_clamp_file', type=str, default='/home/lumos/code/FastTouchV2/cccc/starnew/session_091619/right_hand_250801DR48FP26001130/Merged_Trajectory/Clamp_Data/clamp_data_tum.txt')
    parser.add_argument('--can_interface', type=str, default='can0', help='CAN接口')
    parser.add_argument('--no_transform', action='store_true', help='不使用坐标变换')
    parser.add_argument('--base_x', type=float, default=0.35)
    parser.add_argument('--base_y', type=float, default=0.0)
    parser.add_argument('--base_z', type=float, default=0.16)
    parser.add_argument('--base_roll', type=float, default=0.0)
    parser.add_argument('--base_pitch', type=float, default=0.0)
    parser.add_argument('--base_yaw', type=float, default=0.0)
    parser.add_argument('--home_pos', type=str, default='0.158,0.28,0.145', help='HOME位置 (x,y,z)')
    parser.add_argument('--home_quat', type=str, default='0.0,0.0,0.0,1.0', help='HOME姿态 (w,x,y,z)')
    parser.add_argument('--move_time', type=float, default=3.0, help='移动到首点的时间(s)')
    parser.add_argument('--return_time', type=float, default=3.0, help='返回HOME的时间(s)')
    parser.add_argument('--no_plot', action='store_true', help='不绘制图表')
    parser.add_argument('--plot_out', type=str, default='replay_error.png')
    parser.add_argument('--plot_show', action='store_true')
    parser.add_argument('--dry_run', action='store_true', help='仅打印轨迹，不执行')
    
    args = parser.parse_args()
    
    # 解析HOME位置和姿态
    home_pos = [float(x) for x in args.home_pos.split(',')]
    home_quat = [float(x) for x in args.home_quat.split(',')]
    
    # 加载轨迹
    if not os.path.isfile(args.trajectory_file):
        raise FileNotFoundError(f'轨迹文件不存在: {args.trajectory_file}')
    
    traj = load_trajectory(args.trajectory_file)
    if traj is None or len(traj) == 0:
        raise ValueError('轨迹为空')
    
    # 坐标变换
    T_base_to_local = None
    if not args.no_transform:
        T_base_to_local = build_T_base_to_local(
            args.base_x, args.base_y, args.base_z,
            args.base_roll, args.base_pitch, args.base_yaw
        )
        print(f'使用坐标变换: base=({args.base_x},{args.base_y},{args.base_z}), rpy=({args.base_roll},{args.base_pitch},{args.base_yaw})')
    
    # 转换轨迹
    poses = trajectory_to_startouch(traj, T_base_to_local)
    print(f'轨迹点数: {len(poses)}')
    
    if args.dry_run:
        print("Dry run - 仅显示前5个点:")
        for i in range(min(5, len(poses))):
            pos, quat = poses[i]
            print(f"Step {i}: pos={pos}, quat={quat}")
        return
    
    # 初始化手臂
    start_quit_listener()
    print("提示：按 'Q' 键将立即结束轨迹回放\n")
    
    arm = SingleArm(can_interface_=args.can_interface)
    time.sleep(2)
    
    try:
        # 1. 平滑移动到轨迹起点
        start_pos, start_quat = poses[0]
        print('平滑移动到轨迹起点...')
        if not move_smooth(arm, start_pos, start_quat, args.move_time):
            print("移动被中断，跳过轨迹复现")
        else:
            # 2. 复现轨迹（400Hz）
            print('开始复现轨迹...')
            cmd_xyz, cmd_rpy, act_xyz, act_rpy = replay_trajectory_smooth(arm, poses)
            
            # 3. 绘制结果
            if not args.no_plot and len(cmd_xyz) > 0:
                print("绘制误差图...")
                plot_results(cmd_xyz, act_xyz, cmd_rpy, act_rpy, 
                           args.plot_out, args.plot_show)
        
        # 4. 平滑返回HOME
        print('平滑返回HOME位置...')
        move_smooth(arm, home_pos, home_quat, args.return_time)
        print('完成')
        
    finally:
        arm.cleanup()
        _clear_quit_flag()

if __name__ == '__main__':
    main()