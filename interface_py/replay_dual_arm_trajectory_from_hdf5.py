#!/usr/bin/env python3
"""
双臂轨迹复现（HDF5 数据源）：从 HDF5 文件读取 robot_0/action、robot_1/action，
转换为 xarm TCP 位姿（xyz 毫米 + 欧拉角）及夹爪控制，仿照 replay_dual_arm_trajectory.py 进行复现。

HDF5 数据格式：
  /robot_0/action          Dataset {N, 8}  # [x_m, y_m, z_m, qx, qy, qz, qw, gripper_0_1]
  /robot_1/action          Dataset {N, 8}
  /robot_0/observations/qpos Dataset {N, 8}
  /robot_1/observations/qpos Dataset {N, 8}
"""
import argparse
import os
import time
import h5py
import numpy as np
from scipy.spatial.transform import Rotation as R
from tqdm import tqdm
from xarm.wrapper import XArmAPI


def action_8_to_pose_and_gripper(action_8):
    """
    将单臂 8 维 action [x_m, y_m, z_m, qx, qy, qz, qw, gripper] 转为 xarm 位姿与夹爪控制。
    - x,y,z 米 -> 毫米
    - 四元数 -> 欧拉角 (度)
    - gripper 0~1 (1=开) -> robotiq 0~255 (0=开, 255=闭): a = int((1-gripper)*255)
    返回: (x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg), gripper_0_255
    """
    x, y, z = float(action_8[0]), float(action_8[1]), float(action_8[2])
    qx, qy, qz, qw = float(action_8[3]), float(action_8[4]), float(action_8[5]), float(action_8[6])
    gripper_01 = float(action_8[7])
    r = R.from_quat([qx, qy, qz, qw])
    roll, pitch, yaw = r.as_euler('xyz', degrees=True)
    x_mm = x * 1000.0
    y_mm = y * 1000.0
    z_mm = z * 1000.0
    gripper_255 = int(np.clip((1.0 - gripper_01) * 255.0, 0, 255))
    return [x_mm, y_mm, z_mm, roll, pitch, yaw], gripper_255


def load_actions_from_hdf5(hdf5_path, use_qpos=False):
    """
    从 HDF5 加载双臂 action（或 qpos）。
    use_qpos: 若 True 则用 observations/qpos 代替 action（用于无 action 的数据集）。
    返回: (actions_0 (N,8), actions_1 (N,8)), N 为步数
    """
    if not os.path.isfile(hdf5_path):
        raise FileNotFoundError(f'HDF5 文件不存在: {hdf5_path}')
    with h5py.File(hdf5_path, 'r') as f:
        if use_qpos:
            a0 = np.array(f['/robot_0/observations/qpos'])
            a1 = np.array(f['/robot_1/observations/qpos'])
        else:
            a0 = np.array(f['/robot_0/action'])
            a1 = np.array(f['/robot_1/action'])
    return a0, a1


def main():
    parser = argparse.ArgumentParser(description='双臂轨迹复现（HDF5 数据源）')
    parser.add_argument('--robot0_ip', type=str, default='192.168.1.217', help='左臂 IP')
    parser.add_argument('--robot1_ip', type=str, default='192.168.1.240', help='右臂 IP')
    parser.add_argument('--hdf5', type=str,
                        default='/home/lumos/Data/K001/ACT/hdf5_step1/episode_000006.hdf5',
                        help='HDF5 文件路径')
    parser.add_argument('--use_qpos', action='store_true',
                        help='使用 observations/qpos 代替 action 作为轨迹（无 action 时用）')
    parser.add_argument('--dt', type=float, default=0.04, help='每步间隔(秒)')
    parser.add_argument('--no_gripper', action='store_true', help='不发送夹爪指令')
    parser.add_argument('--dry_run', action='store_true', help='只打印位姿，不连机器人')
    parser.add_argument('--max_steps', type=int, default=None,
                        help='最大复现步数，默认全部')
    args = parser.parse_args()

    actions_0, actions_1 = load_actions_from_hdf5(args.hdf5, use_qpos=args.use_qpos)
    n0, n1 = len(actions_0), len(actions_1)
    n_steps = min(n0, n1)
    if args.max_steps is not None:
        n_steps = min(n_steps, args.max_steps)
    print(f'左臂步数: {n0}, 右臂步数: {n1}, 复现步数: {n_steps}')
    print(f'数据源: {"qpos" if args.use_qpos else "action"}')

    # 转换为位姿列表
    poses_left = []
    poses_right = []
    grippers_left = []
    grippers_right = []
    for i in range(n_steps):
        pose_l, gl = action_8_to_pose_and_gripper(actions_0[i])
        pose_r, gr = action_8_to_pose_and_gripper(actions_1[i])
        poses_left.append(pose_l)
        poses_right.append(pose_r)
        grippers_left.append(gl)
        grippers_right.append(gr)

    if args.dry_run:
        for i in range(min(5, n_steps)):
            print(f'Step {i} Left: {poses_left[i]}, gripper={grippers_left[i]}')
            print(f'Step {i} Right: {poses_right[i]}, gripper={grippers_right[i]}')
        print('... (dry_run 仅打印前 5 步)')
        return

    # 连接双臂
    arm_left = XArmAPI(args.robot0_ip)
    time.sleep(0.5)
    arm_left.set_mode(1)
    arm_left.set_state(0)
    arm_left.motion_enable(enable=True)

    arm_right = XArmAPI(args.robot1_ip)
    time.sleep(0.5)
    arm_right.set_mode(1)
    arm_right.set_state(0)
    arm_right.motion_enable(enable=True)

    # 初始位姿（与 rollout 一致）
    arm_left.set_position(*[400,0,360,180,-90,0], speed=10000, mvacc=500000, wait=False)
    arm_right.set_position(*[400,0,360,180,-90,0], speed=10000, mvacc=500000, wait=False)
    arm_left.robotiq_set_position(0)
    arm_right.robotiq_set_position(0)
    time.sleep(0.3)
    print('双臂初始化完成，开始复现轨迹...')

    for i in tqdm(range(n_steps), desc='复现轨迹', unit='步'):
        pl = poses_left[i]
        pr = poses_right[i]
        arm_left.set_servo_cartesian([pl[0], pl[1], pl[2], pl[3], pl[4], pl[5]])
        arm_right.set_servo_cartesian([pr[0], pr[1], pr[2], pr[3], pr[4], pr[5]])
        if not args.no_gripper:
            arm_left.robotiq_set_position(grippers_left[i], wait=False, wait_motion=False)
            arm_right.robotiq_set_position(grippers_right[i], wait=False, wait_motion=False)
        time.sleep(args.dt)

    print('轨迹复现结束。')


if __name__ == '__main__':
    main()
