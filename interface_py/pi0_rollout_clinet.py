#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件用途：
    本文件是双臂 Startouch 真机测试版 rollout 客户端，用于连接远端 OpenPI 策略服务、
    采集左右腕部相机图像和双臂末端状态，并使用 SDK 的 move_joint_waypoints 关节路点接口执行策略动作。
    当前脚本只负责在线推理与真机执行。

主要功能：
    1. 通过 WebSocket 连接远端策略服务器，获取任务 metadata 并发送观测进行推理。
    2. 初始化左右两条 Startouch 机械臂、夹爪保护器和两路腕部相机。
    3. 构造 16 维双臂观测 state：左臂 TCP xyz + quat xyzw + gripper，右臂同样格式。
    4. 接收 14 维双臂 action：左臂 TCP xyz + RPY(deg) + gripper，右臂同样格式。
    5. 将策略输出的 TCP 目标转换为法兰位姿，再通过 solve_ik 求解关节路点。
    6. 使用 move_joint_waypoints 同步执行双臂关节轨迹，并同步下发左右夹爪目标。
    7. 支持 IK 重试、夹爪接触保持保护、相机预览窗口和运行时暂停复位。

适用场景：
    适用于验证新版 Startouch SDK 的 solve_ik 与 move_joint_waypoints 执行链路。
    适用于只需要在线测试策略执行效果、不需要采集数据集或保存调试文件的真机实验。

核心逻辑：
    脚本启动后解析命令行参数，初始化双臂和腕部相机；
    按 Enter 后连接策略服务器，随后循环读取双臂状态和双腕图像，发送给策略服务器推理；
    截取指定 action 区间，随后转换为法兰位姿并求解 IK；
    最后调用 move_joint_waypoints 执行双臂关节路点轨迹。运行中按 s 复位并暂停，按 c 继续。

运行方式：
    python pi0_rollout_serve_pro_v1_remote_lxh_0318_startouch_wrist2_rpy_test.py [参数]

运行示例：
    # 示例一：使用默认策略服务、默认 CAN 和默认相机运行
    python pi0_rollout_serve_pro_v1_remote_lxh_0318_startouch_wrist2_rpy_test.py

    # 示例二：指定任务描述、策略服务和相机节点
    python pi0_rollout_serve_pro_v1_remote_lxh_0318_startouch_wrist2_rpy_test.py \
        --description "pick up the object and place it into the bowl" \
        --host x.x.x.x --port 8000 \
        --left_cam 0 --right_cam 2

    # 示例三：调整 IK 重试次数并关闭相机预览窗口
    python pi0_rollout_serve_pro_v1_remote_lxh_0318_startouch_wrist2_rpy_test.py \
        --ik_retries 3 \
        --ik_retry_sleep_s 0.05 \
        --no_camera_window

运行前提：
    - 运行环境需安装 OpenCV、NumPy、websockets 以及本项目 openpi_client 相关依赖。
    - 需可导入 fastumi_startouch_client，并能访问 DEFAULT_STARTOUCH_INTERFACE_DIR 指向的 Startouch SDK。
    - 机器人控制器、左右 CAN 接口和夹爪需上电并可正常通信。
    - 两路腕部相机需在 Linux 下暴露为 /dev/videoN，并可采集 YU12 图像。
    - 远端策略服务需提前启动 serve_policy，并保证 --host/--port 指向 WebSocket 策略端口。
    - 若需要相机预览窗口，Linux 图形环境需设置 DISPLAY 或 WAYLAND_DISPLAY。

参数说明：
    --description：任务自然语言指令；字符串；选填，默认从服务器 metadata 读取；示例：--description "fold the towel"
    --left_can：左臂 CAN 接口；字符串；默认 can0；示例：--left_can can0
    --right_can：右臂 CAN 接口；字符串；默认 can1；示例：--right_can can1
    --left_cam：左腕相机 /dev/videoN 的 N；整数；默认 0；示例：--left_cam 0
    --right_cam：右腕相机 /dev/videoN 的 N；整数；默认 2；示例：--right_cam 2
    --cam_width / --cam_height / --cam_fps：相机采集宽、高、帧率；整数；默认 1280、1280、100。
    --no_camera_window：关闭双腕相机预览窗口；布尔标志；默认 False。
    --host / --port：策略服务器地址和端口。
    --trajectory_dt：每个 action 点对应执行时长；浮点数且必须 > 0；默认 0.05；示例：--trajectory_dt 0.05
    --joint_waypoint_speed_percent：move_joint_waypoints 的 speed_percent；>0 时优先使用速度百分比，默认 -1.0 表示使用 time_sec。
    --ik_retries：每个 IK waypoint 首次失败后的额外重试次数；整数且不能为负；默认 2。
    --ik_retry_sleep_s：IK 失败后每次重试前等待秒数；浮点数且不能为负；默认 0.0。
    --reset_time_sec：初始化和按 s 复位时的运动时长；浮点数且必须 > 0；默认 2.0。
    --gripper_open_position：初始化和复位时夹爪打开目标；浮点数；默认 GRIPPER_OPEN_MAX。
    --disable_gripper_hold：关闭夹爪接触/卡滞 holding 保护；布尔标志；默认 False。
    --gripper_contact_dwell：夹爪接触判定持续时间；浮点数秒；默认 0.2。
    --gripper_contact_epsilon：接触判定窗口内夹爪位置最大变化量；浮点数；默认 0.005。
    --gripper_hold_squeeze：检测到接触后额外收紧并保持的幅度；浮点数；默认 0.1。
    --init_pose_left / --init_pose_right：左右臂初始/复位位姿，格式 x,y,z,roll,pitch,yaw，角度单位为弧度。
    --tcp_offset：法兰坐标系下法兰原点到夹爪尖 TCP 的平移 x,y,z，单位米；默认 0.0,0.0,0.0。
    --tcp_debug：打印 TCP/法兰转换调试信息；布尔标志；默认 False。
    --action_start / --action_end：策略 action 序列执行区间闭区间下标；整数；默认 0、50。

注意事项：
    - 本脚本不会创建 recordings 目录，也不会保存 steps、state samples、manifest、视频、IK trace 或轨迹图。
    - 策略输入图像固定使用 image["robot_0"] 和 image["robot_1"]，由左右腕部相机 resize 到 224x224。
    - 终端进入运行循环后，按 s 会复位并暂停，按 c 会继续；相机窗口内按 q 或 ESC 会退出。
    - 如果未显式传入 --description，远端策略服务必须在 metadata 中提供可用任务描述。
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

import cv2
import numpy as np

from fastumi_startouch_client import (
    DEFAULT_STARTOUCH_INTERFACE_DIR,
    DirectGripperHoldProtector,
    DirectGripperPassthrough,
    DualCameraFrameSampler,
    DualCameraPreviewWindow,
    GRIPPER_OPEN_MAX,
    bootstrap_startouch_python,
    build_dual_pose_trajectories,
    connect_policy_client_or_raise,
    enable_stdin_cbreak,
    execute_dual_move_joint_waypoints_with_grippers,
    flange_position_to_tcp,
    import_single_arm,
    init_yu12_camera,
    parse_init_pose,
    parse_tool_offset_xyz,
    reset_arms_to_init,
    resolve_description,
    resolve_dual_camera_devices,
    restore_stdin,
    stdin_read_char_nonblocking,
    wait_for_enter_with_dual_camera_preview,
)


STARTOUCH_INTERFACE_DIR = Path(DEFAULT_STARTOUCH_INTERFACE_DIR).expanduser().resolve()
bootstrap_startouch_python(STARTOUCH_INTERFACE_DIR, entrypoint=Path(__file__).resolve())
if str(STARTOUCH_INTERFACE_DIR) not in sys.path:
    sys.path.insert(0, str(STARTOUCH_INTERFACE_DIR))

from openpi_client import image_tools


SingleArm = import_single_arm(STARTOUCH_INTERFACE_DIR)

SERVER_IP = "x.x.x.x"
PORT = 8000
LEFT_CAM_DEV = 0
RIGHT_CAM_DEV = 2
DEFAULT_CAM_WIDTH = 1280
DEFAULT_CAM_HEIGHT = 1280
DEFAULT_CAM_FPS = 100
CAMERA_WINDOW_NAME = "Dual Startouch Wrist Camera Preview"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--description",
        default=None,
        help="任务自然语言指令；不传时会从策略服务器 metadata 的 description/prompt/task/default_prompt/task_name 自动读取",
    )
    parser.add_argument("--left_can", default="can0", help="左臂（arm0）CAN 接口")
    parser.add_argument("--right_can", default="can1", help="右臂（arm1）CAN 接口")
    parser.add_argument("--left_cam", type=int, default=LEFT_CAM_DEV, help="左腕相机设备号 /dev/videoN 中的 N")
    parser.add_argument("--right_cam", type=int, default=RIGHT_CAM_DEV, help="右腕相机设备号 /dev/videoN 中的 N")
    parser.add_argument("--cam_width", type=int, default=DEFAULT_CAM_WIDTH, help="两路相机采集宽度")
    parser.add_argument("--cam_height", type=int, default=DEFAULT_CAM_HEIGHT, help="两路相机采集高度")
    parser.add_argument("--cam_fps", type=int, default=DEFAULT_CAM_FPS, help="两路相机采集帧率")
    parser.add_argument("--no_camera_window", action="store_true", help="不新开窗口显示双腕摄像头画面")
    parser.add_argument("--host", default=SERVER_IP, help="策略服务器 host")
    parser.add_argument("--port", type=int, default=PORT, help="策略服务器端口")
    parser.add_argument("--trajectory_dt", type=float, default=0.05, help="整段 move_joint_waypoints 轨迹中每个 action 点对应的时间（秒）")
    parser.add_argument("--joint_waypoint_speed_percent", type=float, default=-1.0, help="传给 SDK move_joint_waypoints 的 speed_percent；>0 时优先用速度百分比，否则使用 trajectory_dt 算 time_sec")
    parser.add_argument("--ik_retries", type=int, default=5, help="每个 IK waypoint 首次失败后的额外重试次数；默认 5，即总共尝试 6 次")
    parser.add_argument("--ik_retry_sleep_s", type=float, default=0.0, help="IK 失败后每次重试前等待多少秒；默认 0 表示立刻重试")
    parser.add_argument("--reset_time_sec", type=float, default=2.0, help="初始位姿/复位运动时长（秒）")
    parser.add_argument("--gripper_open_position", type=float, default=GRIPPER_OPEN_MAX, help="复位/初始化时使用的夹爪打开目标")
    parser.add_argument("--disable_gripper_hold", action="store_true", help="关闭夹爪接触/卡滞 holding 保护，直接下发模型 gripper 连续值")
    parser.add_argument("--gripper_contact_dwell", type=float, default=0.2, help="闭合时夹爪位置近乎不变持续多久判定为接触/卡滞")
    parser.add_argument("--gripper_contact_epsilon", type=float, default=0.005, help="接触判定窗口内允许的夹爪位置最大变化量")
    parser.add_argument("--gripper_hold_squeeze", type=float, default=0.1, help="检测到夹住物体后，从当前读数再轻微收紧多少并保持")
    parser.add_argument("--init_pose_left", type=str, default="0.3,0.04,0.16,0.0,0.0,0.0", help="左臂初始位姿 x,y,z,roll,pitch,yaw（弧度）")
    parser.add_argument("--init_pose_right", type=str, default="0.3,-0.04,0.16,0.0,0.0,0.0", help="右臂初始位姿 x,y,z,roll,pitch,yaw（弧度）")
    parser.add_argument(
        "--tcp_offset",
        type=str,
        default="0.0,0.0,0.0",
        help=(
            "双臂共用：法兰系下「法兰原点→夹爪尖 TCP」的平移 x,y,z（米）。"
            "训练 state/action 为 TCP、真机读法兰时填标定；训练为法兰则 0,0,0。"
        ),
    )
    parser.add_argument("--tcp_debug", action="store_true", help="打印观测侧 法兰->TCP 与下发侧 TCP->法兰 调试信息")
    parser.add_argument("--action_start", type=int, default=0, help="本次推理返回的 action 序列起始下标（含）")
    parser.add_argument("--action_end", type=int, default=50, help="本次推理返回的 action 序列结束下标（含）")
    args = parser.parse_args()

    if args.trajectory_dt <= 0.0:
        raise ValueError(f"--trajectory_dt 必须为正数，实际得到 {args.trajectory_dt}")
    if args.joint_waypoint_speed_percent > 1.0:
        raise ValueError(f"--joint_waypoint_speed_percent 必须 <= 1.0，实际得到 {args.joint_waypoint_speed_percent}")
    if args.ik_retries < 0:
        raise ValueError(f"--ik_retries 不能为负数，实际得到 {args.ik_retries}")
    if args.ik_retry_sleep_s < 0.0:
        raise ValueError(f"--ik_retry_sleep_s 不能为负数，实际得到 {args.ik_retry_sleep_s}")
    if args.reset_time_sec <= 0.0:
        raise ValueError(f"--reset_time_sec 必须为正数，实际得到 {args.reset_time_sec}")

    tcp_off = parse_tool_offset_xyz(args.tcp_offset)
    if np.any(tcp_off != 0.0):
        print(f"[INFO] TCP 工具偏移（法兰系，米，双臂共用）={tcp_off.tolist()}")

    resolved_left_cam, resolved_right_cam = resolve_dual_camera_devices(args.left_cam, args.right_cam)

    init_pos0, init_euler0 = parse_init_pose(args.init_pose_left)
    init_pos1, init_euler1 = parse_init_pose(args.init_pose_right)

    arm0 = None
    arm1 = None
    cam0 = None
    cam1 = None
    policy_client = None
    camera_sampler = None
    camera_preview = None
    stdin_fd = None
    old_term = None
    try:
        arm0 = SingleArm(can_interface_=args.left_can)
        arm1 = SingleArm(can_interface_=args.right_can)
        if not hasattr(arm0, "move_joint_waypoints") or not hasattr(arm1, "move_joint_waypoints"):
            raise RuntimeError(
                "当前加载的 Startouch SDK 没有 move_joint_waypoints()，不能使用关节路点执行路径。"
                f"当前 SingleArm 模块: {getattr(sys.modules.get(SingleArm.__module__), '__file__', '<unknown>')}。"
                "请使用新版 startouch SDK，或设置 STARTOUCH_INTERFACE_DIR=/home/benchmark/pi0_client/startouch_sdk/interface_py 后重试。"
            )
        time.sleep(2)
        if args.disable_gripper_hold:
            print("[INFO] 已关闭 gripper hold/contact 保护：夹爪将直接执行模型输出")
            gripper0 = DirectGripperPassthrough(arm0)
            gripper1 = DirectGripperPassthrough(arm1)
        else:
            gripper0 = DirectGripperHoldProtector(
                arm0,
                contact_dwell=args.gripper_contact_dwell,
                contact_epsilon=args.gripper_contact_epsilon,
                hold_squeeze=args.gripper_hold_squeeze,
            )
            gripper1 = DirectGripperHoldProtector(
                arm1,
                contact_dwell=args.gripper_contact_dwell,
                contact_epsilon=args.gripper_contact_epsilon,
                hold_squeeze=args.gripper_hold_squeeze,
            )

        print(f"[INFO] 移动到初始位姿: left={args.init_pose_left}  right={args.init_pose_right}")
        reset_arms_to_init(
            arm0,
            arm1,
            init_pos0,
            init_euler0,
            init_pos1,
            init_euler1,
            gripper0=gripper0,
            gripper1=gripper1,
            open_position=args.gripper_open_position,
            reset_time_sec=args.reset_time_sec,
        )
        print("[INFO] 已到达初始位姿")

        print(
            f"[INFO] 打开两路腕部摄像头: left=/dev/video{resolved_left_cam} "
            f"right=/dev/video{resolved_right_cam} "
            f"@ {args.cam_width}x{args.cam_height} {args.cam_fps}fps"
        )
        cam0 = init_yu12_camera(resolved_left_cam, args.cam_width, args.cam_height, args.cam_fps)
        cam1 = init_yu12_camera(resolved_right_cam, args.cam_width, args.cam_height, args.cam_fps)
        for _ in range(50):
            _ = cam0.read()
            _ = cam1.read()
        print("[INFO] 两路腕部摄像头预热完成，开始循环")

        camera_sampler = DualCameraFrameSampler(
            cam0,
            cam1,
            width=args.cam_width,
            height=args.cam_height,
            sample_fps=args.cam_fps,
        )
        camera_sampler.start()
        camera_preview = DualCameraPreviewWindow(
            enabled=not args.no_camera_window,
            window_name=CAMERA_WINDOW_NAME,
        )

        def reset_and_pause() -> None:
            reset_arms_to_init(
                arm0,
                arm1,
                init_pos0,
                init_euler0,
                init_pos1,
                init_euler1,
                gripper0=gripper0,
                gripper1=gripper1,
                open_position=args.gripper_open_position,
                reset_time_sec=args.reset_time_sec,
            )

        preview_quit_requested = False
        preview_warned = False

        def refresh_camera_preview() -> bool:
            nonlocal preview_quit_requested, preview_warned
            if preview_quit_requested:
                return True
            if camera_preview is None or not camera_preview.enabled:
                return False
            try:
                left_preview, right_preview = camera_sampler.get_latest(timeout=0.02)
            except Exception as exc:
                if not preview_warned:
                    print(f"[WARN] 暂时无法刷新双腕摄像头预览窗口: {exc}")
                    preview_warned = True
                return False
            if camera_preview.update(left_preview, right_preview, resolved_left_cam, resolved_right_cam):
                preview_quit_requested = True
                print("[INFO] 双腕摄像头窗口收到退出指令，准备结束。")
            return preview_quit_requested

        if wait_for_enter_with_dual_camera_preview(
            camera_sampler.get_latest,
            camera_preview,
            resolved_left_cam,
            resolved_right_cam,
        ):
            return

        print(args.host, args.port)
        policy_client = connect_policy_client_or_raise(args.host, args.port)
        print(f"[INFO] 已连接策略服务器：ws://{args.host}:{args.port}")
        try:
            server_metadata = policy_client.get_server_metadata() or {}
        except Exception as exc:
            server_metadata = {}
            print(f"[WARN] 无法读取策略服务器 metadata，将使用 unknown fallback: {exc}")
        print(f"[INFO] 策略服务器 metadata: {server_metadata}")
        args.description = resolve_description(args.description, server_metadata)

        stdin_fd, old_term = enable_stdin_cbreak()
        print("[INFO] 按 s：复位双臂并暂停；按 c：继续推理")

        paused = False
        while True:
            if refresh_camera_preview():
                break

            ch = stdin_read_char_nonblocking()
            if ch in ("s", "S"):
                reset_and_pause()
                paused = True
                print("[INFO] 已复位并暂停；按 c 继续推理")
                continue
            if ch in ("c", "C"):
                paused = False
                print("[INFO] 继续推理")

            if paused:
                time.sleep(0.02)
                continue

            pos0, quat_wxyz0 = arm0.get_ee_pose_quat()
            qw0, qx0, qy0, qz0 = quat_wxyz0
            quat0 = np.array([qx0, qy0, qz0, qw0], dtype=np.float32)
            p_tcp0 = flange_position_to_tcp(pos0, quat_wxyz0, tcp_off)
            x0, y0, z0 = p_tcp0

            pos1, quat_wxyz1 = arm1.get_ee_pose_quat()
            qw1, qx1, qy1, qz1 = quat_wxyz1
            quat1 = np.array([qx1, qy1, qz1, qw1], dtype=np.float32)
            p_tcp1 = flange_position_to_tcp(pos1, quat_wxyz1, tcp_off)
            x1, y1, z1 = p_tcp1

            if args.tcp_debug:
                print(
                    "[TCP DEBUG][obs] flange -> state(TCP) | "
                    f"L {np.asarray(pos0).round(4)} -> {np.asarray(p_tcp0).round(4)} | "
                    f"R {np.asarray(pos1).round(4)} -> {np.asarray(p_tcp1).round(4)} | "
                    f"off {tcp_off.tolist()}"
                )

            gripper_open0 = float(arm0.get_gripper_position())
            gripper_open1 = float(arm1.get_gripper_position())

            state_vec = np.array(
                [x0, y0, z0, *quat0, gripper_open0, x1, y1, z1, *quat1, gripper_open1],
                dtype=np.float32,
            )
            if state_vec.shape != (16,):
                raise ValueError(f"双臂 state 期望 16 维，实际得到 {state_vec.shape}")

            img_latest0, img_latest1 = camera_sampler.get_latest()
            if refresh_camera_preview():
                break
            img_rgb0 = cv2.resize(img_latest0, (224, 224), interpolation=cv2.INTER_AREA)
            img_rgb1 = cv2.resize(img_latest1, (224, 224), interpolation=cv2.INTER_AREA)
            obs = {
                "state": state_vec,
                "image": {
                    "robot_0": image_tools.convert_to_uint8(img_rgb0),
                    "robot_1": image_tools.convert_to_uint8(img_rgb1),
                },
                "prompt": args.description,
            }
            print("[INFO] 即将发送 obs: state_shape=%s, image_keys=%s" % (state_vec.shape, list(obs["image"].keys())))
            resp = policy_client.infer(obs)

            actions_all = resp["actions"] if "actions" in resp else resp["action"]
            actions_all = np.array(actions_all, dtype=np.float64, copy=True)
            if actions_all.ndim == 1:
                actions_all = actions_all.reshape(1, -1)
            if actions_all.shape[-1] != 14:
                raise ValueError(f"双臂脚本期望 14 维 action，实际收到 shape={actions_all.shape}")

            #actions_all[..., 6] = np.clip(actions_all[..., 6], 0.0, 1.0)
            #actions_all[..., 13] = np.clip(actions_all[..., 13], 0.0, 1.0)
            for gripper_idx in (6, 13):
                g = actions_all[..., gripper_idx]
                g = np.where(g < 0.8, g - 0.1, g)
                actions_all[..., gripper_idx] = np.clip(g, 0.0, 1.0)


            a0, a1 = args.action_start, args.action_end
            n_act = len(actions_all)
            if a0 < 0 or a1 < 0:
                print(f"[WARN] 忽略非法 action 区间: action_start={a0} action_end={a1}")
                continue
            if a0 > a1:
                print(f"[WARN] action_start 不能大于 action_end: {a0} > {a1}")
                continue
            if n_act == 0:
                print("[WARN] 策略返回的 action 序列为空")
                continue
            lo = a0
            hi = min(a1, n_act - 1)
            if lo >= n_act:
                print(f"[WARN] action_start 越界: start={a0} 序列长度={n_act}")
                continue
            if lo > hi:
                print(f"[WARN] 无 action 可执行: 区间 [{a0},{a1}] 与长度 {n_act} 无交集")
                continue

            action_slice = actions_all[lo : hi + 1]
            step_key = stdin_read_char_nonblocking()
            if step_key in ("s", "S"):
                reset_and_pause()
                paused = True
                print("[INFO] 已复位并暂停；按 c 继续推理")
                continue

            left_poses, right_poses, left_grippers, right_grippers = build_dual_pose_trajectories(action_slice, tcp_off)
            trajectory_time_sec = len(action_slice) * args.trajectory_dt
            if args.tcp_debug:
                print(
                    "[TCP DEBUG][cmd] action_slice=%d..%d duration=%.3fs | "
                    "L first=%s last=%s | R first=%s last=%s"
                    % (
                        lo,
                        hi,
                        trajectory_time_sec,
                        np.asarray(left_poses[0]).round(4).tolist(),
                        np.asarray(left_poses[-1]).round(4).tolist(),
                        np.asarray(right_poses[0]).round(4).tolist(),
                        np.asarray(right_poses[-1]).round(4).tolist(),
                    )
                )

            print(
                "[INFO] 执行 move_joint_waypoints 轨迹: steps=%d, duration=%.3fs, "
                "left_gripper %.3f->%.3f, right_gripper %.3f->%.3f"
                % (
                    len(action_slice),
                    trajectory_time_sec,
                    left_grippers[0],
                    left_grippers[-1],
                    right_grippers[0],
                    right_grippers[-1],
                )
            )
            durations = execute_dual_move_joint_waypoints_with_grippers(
                arm0,
                arm1,
                left_poses,
                right_poses,
                left_grippers,
                right_grippers,
                trajectory_time_sec=trajectory_time_sec,
                joint_waypoint_speed_percent=args.joint_waypoint_speed_percent,
                gripper0=gripper0,
                gripper1=gripper1,
                ik_retries=args.ik_retries,
                ik_retry_sleep_s=args.ik_retry_sleep_s,
            )
            print(f"[INFO] move_joint_waypoints 轨迹完成: {durations}")

    finally:
        restore_stdin(stdin_fd, old_term)
        if camera_sampler is not None:
            camera_sampler.stop()
        if cam0 is not None:
            cam0.release()
        if cam1 is not None:
            cam1.release()
        if arm0 is not None:
            arm0.cleanup()
        if arm1 is not None:
            arm1.cleanup()
        if camera_preview is not None:
            camera_preview.close()
        print("[INFO] 结束，摄像头与机械臂已释放。")


if __name__ == "__main__":
    main()
