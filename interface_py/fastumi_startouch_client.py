#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Startouch helper utilities used by the slim dual-arm OpenPI clients.

This module intentionally keeps only the small shared surface needed by the
rollout scripts:
    - Startouch SDK path/bootstrap helpers
    - SingleArm import helper
    - YU12 camera open/read helpers
    - TCP/flange position conversion helpers
"""

from __future__ import annotations

import os
from pathlib import Path
import select
import shutil
import subprocess
import sys
import termios
import threading
import time
import tty

import cv2
import numpy as np
import websockets.exceptions

from openpi_client import websocket_client_policy


_REPO_ROOT = Path(__file__).resolve().parent
_WORKSPACE_ROOT = _REPO_ROOT.parent
_OPENPI_CLIENT_SRC_CANDIDATES = (
    _REPO_ROOT / "openpi" / "packages" / "openpi-client" / "src",
    _WORKSPACE_ROOT / "openpi_0318" / "packages" / "openpi-client" / "src",
)
DEFAULT_STARTOUCH_INTERFACE_DIR = _REPO_ROOT / "startouch-v1" / "interface_py"

for path in (_REPO_ROOT, *_OPENPI_CLIENT_SRC_CANDIDATES):
    if path.exists() and str(path) not in sys.path:
        sys.path.insert(0, str(path))


GRIPPER_OPEN_MAX = 1.0
_SYS_VIDEO_ROOT = Path("/sys/class/video4linux")
_DUAL_CAMERA_WINDOW_W = 640
_DUAL_CAMERA_WINDOW_H = 640

__all__ = [
    "DEFAULT_STARTOUCH_INTERFACE_DIR",
    "DirectGripperHoldProtector",
    "DirectGripperPassthrough",
    "DualCameraFrameSampler",
    "DualCameraPreviewWindow",
    "GRIPPER_OPEN_MAX",
    "bootstrap_startouch_python",
    "build_dual_pose_trajectories",
    "connect_policy_client_or_raise",
    "enable_stdin_cbreak",
    "execute_dual_move_joint_waypoints_with_grippers",
    "flange_position_to_tcp",
    "grab_yu12_rgb",
    "import_single_arm",
    "init_yu12_camera",
    "open_gripper_max",
    "parse_init_pose",
    "parse_tool_offset_xyz",
    "reset_arms_to_init",
    "resolve_description",
    "resolve_dual_camera_devices",
    "restore_stdin",
    "solve_joint_waypoints_from_poses",
    "stdin_read_char_nonblocking",
    "tcp_position_to_flange",
    "wait_for_enter_with_dual_camera_preview",
]


def _python_minor_version(executable: Path) -> str | None:
    try:
        return subprocess.check_output(
            [
                str(executable),
                "-c",
                "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')",
            ],
            text=True,
            timeout=3,
        ).strip()
    except Exception:
        return None


def bootstrap_startouch_python(interface_dir: Path, entrypoint: Path | None = None) -> None:
    """Restart with Python 3.10 when only the cp310 Startouch extension exists."""
    if os.environ.get("FASTUMI_STARTOUCH_BOOTSTRAPPED") == "1":
        return
    if sys.version_info[:2] == (3, 10):
        return

    interface_dir = interface_dir.expanduser().resolve()
    current_suffix = f"startouch.cpython-{sys.version_info.major}{sys.version_info.minor}-*.so"
    has_current_extension = any(interface_dir.glob(current_suffix))
    has_cp310_extension = any(interface_dir.glob("startouch.cpython-310-*.so"))
    if has_current_extension or not has_cp310_extension:
        return

    candidate_paths: list[Path] = []
    override = os.environ.get("PI0_STARTOUCH_PYTHON")
    if override:
        candidate_paths.append(Path(override).expanduser())
    for env_var in ("CONDA_PREFIX", "VIRTUAL_ENV"):
        env_prefix = os.environ.get(env_var)
        if env_prefix:
            candidate_paths.append(Path(env_prefix) / "bin" / "python")
    candidate_paths.extend(
        [
            Path("/home/benchmark/miniconda3/envs/umi_bench/bin/python"),
            Path("/home/benchmark/miniconda3/envs/bestman/bin/python"),
        ]
    )
    python310_on_path = shutil.which("python3.10")
    if python310_on_path:
        candidate_paths.append(Path(python310_on_path))

    entrypoint = Path(entrypoint or Path(__file__).resolve()).expanduser().resolve()

    seen: set[Path] = set()
    for candidate in candidate_paths:
        if not candidate.exists():
            continue
        candidate_resolved = candidate.resolve()
        if candidate_resolved in seen or candidate_resolved == Path(sys.executable).resolve():
            continue
        seen.add(candidate_resolved)
        if _python_minor_version(candidate_resolved) != "3.10":
            continue

        env = os.environ.copy()
        env["FASTUMI_STARTOUCH_BOOTSTRAPPED"] = "1"
        bootstrap_pythonpath = [
            str(path)
            for path in (interface_dir, _REPO_ROOT, *_OPENPI_CLIENT_SRC_CANDIDATES)
            if path.exists()
        ]
        existing_pythonpath = env.get("PYTHONPATH")
        if existing_pythonpath:
            bootstrap_pythonpath.append(existing_pythonpath)
        env["PYTHONPATH"] = os.pathsep.join(bootstrap_pythonpath)

        print(
            "[INFO] startouch 当前只有 Python 3.10 扩展，"
            f"检测到正在使用 Python {sys.version_info.major}.{sys.version_info.minor}；"
            f"自动切换到 {candidate_resolved}",
            flush=True,
        )
        os.execve(
            str(candidate_resolved),
            [str(candidate_resolved), str(entrypoint), *sys.argv[1:]],
            env,
        )

    print(
        "[WARN] startouch 当前只有 Python 3.10 扩展，但没有找到可用的 Python 3.10 解释器；"
        "后续导入可能失败。可设置 PI0_STARTOUCH_PYTHON=/path/to/python3.10 后重试。",
        flush=True,
    )


def import_single_arm(interface_dir: Path):
    interface_path = interface_dir.expanduser().resolve()
    if str(interface_path) not in sys.path:
        sys.path.insert(0, str(interface_path))
    from startouchclass import SingleArm

    return SingleArm


def init_yu12_camera(device: int, width: int, height: int, fps: int):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open /dev/video{device}")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YU12"))
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass
    fcc_int = int(cap.get(cv2.CAP_PROP_FOURCC))
    fcc = "".join(chr((fcc_int >> (8 * i)) & 0xFF) for i in range(4))
    print(f"[INFO] FOURCC from driver /dev/video{device}: {fcc}")
    return cap


def grab_yu12_rgb(cap, width: int, height: int, flush_n: int = 4) -> np.ndarray:
    for _ in range(flush_n):
        cap.grab()
    ok, raw = cap.read()
    if not ok:
        raise RuntimeError("Camera read failed")
    yuv = np.ascontiguousarray(raw).reshape(height * 3 // 2, width)
    bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def _normalize_quat_wxyz(quat_wxyz) -> np.ndarray:
    quat = np.asarray(quat_wxyz, dtype=np.float64)
    if quat.shape != (4,):
        raise ValueError(f"quat_wxyz 需要 4 个值，实际得到 shape={quat.shape}")
    norm = float(np.linalg.norm(quat))
    if norm <= 0.0:
        raise ValueError("quat_wxyz 不能是零四元数")
    return quat / norm


def _quat_wxyz_to_matrix(quat_wxyz) -> np.ndarray:
    qw, qx, qy, qz = _normalize_quat_wxyz(quat_wxyz)
    return np.asarray(
        [
            [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)],
            [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)],
            [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)],
        ],
        dtype=np.float64,
    )


def _euler_xyz_to_quat_wxyz(euler_rad) -> np.ndarray:
    roll, pitch, yaw = np.asarray(euler_rad, dtype=np.float64)
    cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
    cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
    cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
    return np.asarray(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ],
        dtype=np.float64,
    )


def flange_position_to_tcp(flange_pos, quat_wxyz, tcp_offset: np.ndarray) -> np.ndarray:
    rot = _quat_wxyz_to_matrix(quat_wxyz)
    return np.asarray(flange_pos, dtype=np.float64) + rot @ np.asarray(tcp_offset, dtype=np.float64)


def tcp_position_to_flange(tcp_pos, euler_rad, tcp_offset: np.ndarray) -> np.ndarray:
    quat_wxyz = _euler_xyz_to_quat_wxyz(euler_rad)
    rot = _quat_wxyz_to_matrix(quat_wxyz)
    return np.asarray(tcp_pos, dtype=np.float64) - rot @ np.asarray(tcp_offset, dtype=np.float64)


def parse_tool_offset_xyz(offset_str) -> np.ndarray:
    if isinstance(offset_str, (list, tuple, np.ndarray)):
        arr = np.asarray(offset_str, dtype=np.float64)
    else:
        arr = np.asarray([float(v) for v in str(offset_str).split(",")], dtype=np.float64)
    if arr.shape != (3,):
        raise ValueError(f"tcp_offset 需要 3 个值 x,y,z，实际得到: {offset_str}")
    return arr


def parse_init_pose(text: str) -> tuple[list[float], list[float]]:
    parts = [float(v) for v in text.split(",")]
    if len(parts) != 6:
        raise ValueError(f"init_pose 需要 6 个值 x,y,z,r,p,y，实际得到: {text}")
    x, y, z, roll, pitch, yaw = parts
    return [x, y, z], [roll, pitch, yaw]


def resolve_description(cli_description: str | None, server_metadata: dict) -> str:
    if cli_description and cli_description.strip():
        return cli_description.strip()

    for key in ("description", "prompt", "task", "default_prompt", "task_name"):
        value = server_metadata.get(key)
        if value and str(value).strip():
            resolved = str(value).strip()
            print(f"[INFO] 未传 --description，使用 server metadata['{key}'] 作为 prompt: {resolved}")
            return resolved

    raise ValueError(
        "未传 --description，且策略服务器 metadata 中没有 description/prompt/task/default_prompt/task_name。"
        "请在远端 metadata 中透传任务描述，或在客户端显式传 --description。"
    )


def connect_policy_client_or_raise(host: str, port: int):
    try:
        return websocket_client_policy.WebsocketClientPolicy(host=host, port=port)
    except websockets.exceptions.InvalidMessage as exc:
        raise RuntimeError(
            f"无法连接策略服务器 ws://{host}:{port}：未收到合法的 WebSocket HTTP 响应。"
            "这通常表示公网转发口返回了 502/404，或该端口上不是 serve_policy 的 websocket 服务。"
            "请优先检查：1) GPU 侧 serve_policy 是否已启动；2) 该公网端口当前是否被其他 HTTP 服务占用；"
        ) from exc
    except OSError as exc:
        raise RuntimeError(
            f"无法连接策略服务器 ws://{host}:{port}：{exc}。"
            "请检查公网地址、端口和 frp 转发状态。"
        ) from exc


def _video_sysfs_dir(device_id: int) -> Path:
    return _SYS_VIDEO_ROOT / f"video{device_id}"


def _read_int_file(path: Path) -> int | None:
    try:
        return int(path.read_text().strip())
    except Exception:
        return None


def _video_group_key(device_id: int) -> str | None:
    sysfs_dir = _video_sysfs_dir(device_id)
    if not sysfs_dir.exists():
        return None
    try:
        return str((sysfs_dir / "device").resolve())
    except Exception:
        return None


def _scan_video_nodes() -> list[dict]:
    nodes = []
    if not _SYS_VIDEO_ROOT.exists():
        return nodes
    for entry in sorted(_SYS_VIDEO_ROOT.glob("video*")):
        name = entry.name
        if not name.startswith("video"):
            continue
        try:
            device_id = int(name[5:])
        except ValueError:
            continue
        nodes.append(
            {
                "device_id": device_id,
                "index": _read_int_file(entry / "index"),
                "group": _video_group_key(device_id),
            }
        )
    return nodes


def _resolve_capture_camera(requested_device_id: int, role: str) -> dict:
    sysfs_dir = _video_sysfs_dir(requested_device_id)
    if not sysfs_dir.exists():
        raise RuntimeError(
            f"{role} 相机 /dev/video{requested_device_id} 不存在。"
            "请先用 `v4l2-ctl --list-devices` 确认可用节点。"
        )

    requested_index = _read_int_file(sysfs_dir / "index")
    requested_group = _video_group_key(requested_device_id)
    resolved_device_id = requested_device_id
    auto_resolved = False

    if requested_index not in (None, 0) and requested_group is not None:
        for node in _scan_video_nodes():
            if node["group"] == requested_group and node["index"] == 0:
                resolved_device_id = node["device_id"]
                auto_resolved = resolved_device_id != requested_device_id
                break

    if auto_resolved:
        print(
            f"[INFO] {role} 相机 /dev/video{requested_device_id} 看起来是 metadata 节点"
            f"（index={requested_index}），自动改用同一设备的图像节点 /dev/video{resolved_device_id}"
        )

    return {
        "requested": requested_device_id,
        "resolved": resolved_device_id,
        "requested_index": requested_index,
        "group": requested_group,
        "auto_resolved": auto_resolved,
    }


def resolve_dual_camera_devices(left_device_id: int, right_device_id: int) -> tuple[int, int]:
    left = _resolve_capture_camera(left_device_id, "左腕")
    right = _resolve_capture_camera(right_device_id, "右腕")

    if left["resolved"] == right["resolved"]:
        raise RuntimeError(
            "左右相机最终解析到了同一个图像节点 "
            f"/dev/video{left['resolved']}。"
            f"当前传参 left_cam={left_device_id}, right_cam={right_device_id}。"
            "这通常表示你把某一路 metadata 节点和同一物理相机的图像节点一起传进来了。"
            "请从两台不同 XVisio 设备各选一个 `Video Capture` 节点。"
        )

    return left["resolved"], right["resolved"]


def _combine_wrist_frames_bgr(left_rgb: np.ndarray, right_rgb: np.ndarray, left_dev: int, right_dev: int) -> np.ndarray:
    left = cv2.resize(left_rgb, (_DUAL_CAMERA_WINDOW_W, _DUAL_CAMERA_WINDOW_H), interpolation=cv2.INTER_AREA)
    right = cv2.resize(right_rgb, (_DUAL_CAMERA_WINDOW_W, _DUAL_CAMERA_WINDOW_H), interpolation=cv2.INTER_AREA)
    cv2.putText(left, f"robot_0 /dev/video{left_dev}", (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
    cv2.putText(right, f"robot_1 /dev/video{right_dev}", (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
    combined_rgb = np.concatenate([left, right], axis=1)
    return cv2.cvtColor(combined_rgb, cv2.COLOR_RGB2BGR)


class DualCameraPreviewWindow:
    def __init__(self, enabled: bool, window_name: str):
        self.enabled = False
        self.window_name = window_name
        self.closed = False
        if not enabled:
            return
        if sys.platform.startswith("linux") and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            print("[WARN] 未检测到图形显示环境，跳过双腕摄像头预览窗口。")
            return
        try:
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window_name, _DUAL_CAMERA_WINDOW_W * 2, _DUAL_CAMERA_WINDOW_H)
            self.enabled = True
            print(f"[INFO] 双腕摄像头预览窗口已打开：{window_name}，窗口内按 q 或 ESC 可退出")
        except cv2.error as exc:
            print(f"[WARN] OpenCV 无法创建双腕摄像头预览窗口，已跳过显示。错误: {exc}")

    def update(self, left_rgb: np.ndarray, right_rgb: np.ndarray, left_dev: int, right_dev: int) -> bool:
        if not self.enabled:
            return False
        if self.closed:
            return True
        frame_bgr = _combine_wrist_frames_bgr(left_rgb, right_rgb, left_dev, right_dev)
        cv2.imshow(self.window_name, frame_bgr)
        key = cv2.waitKey(1) & 0xFF
        self.closed = key in (ord("q"), ord("Q"), 27)
        return self.closed

    def close(self) -> None:
        if not self.enabled:
            return
        try:
            cv2.destroyWindow(self.window_name)
        except cv2.error:
            pass
        self.enabled = False


class DualCameraFrameSampler:
    def __init__(
        self,
        cam0,
        cam1,
        width: int,
        height: int,
        sample_fps: float,
    ):
        self.cam0 = cam0
        self.cam1 = cam1
        self.width = width
        self.height = height
        self.sample_fps = max(float(sample_fps), 1.0)
        self.period = 1.0 / self.sample_fps
        self._stop_event = threading.Event()
        self._condition = threading.Condition()
        self._thread = threading.Thread(target=self._run, name="dual-camera-sampler", daemon=True)
        self._latest_left_rgb = None
        self._latest_right_rgb = None
        self._error = None
        self.samples = 0

    def start(self) -> None:
        self._thread.start()

    def get_latest(self, timeout: float = 2.0) -> tuple[np.ndarray, np.ndarray]:
        deadline = time.monotonic() + timeout if timeout is not None else None
        with self._condition:
            while self._latest_left_rgb is None and self._error is None:
                if deadline is None:
                    self._condition.wait()
                    continue
                remaining = deadline - time.monotonic()
                if remaining <= 0 or not self._condition.wait(remaining):
                    break
            if self._error is not None:
                raise RuntimeError("双腕摄像头采样线程失败") from self._error
            if self._latest_left_rgb is None or self._latest_right_rgb is None:
                raise RuntimeError("等待双腕摄像头首帧超时")
            return self._latest_left_rgb.copy(), self._latest_right_rgb.copy()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread.is_alive():
            self._thread.join(timeout=3.0)

    def _run(self) -> None:
        next_sample_ts = time.monotonic()
        print(f"[INFO] 双腕摄像头采样线程已启动，目标频率 {self.sample_fps:g} Hz")
        while not self._stop_event.is_set():
            try:
                left_rgb = grab_yu12_rgb(self.cam0, self.width, self.height, flush_n=0)
                right_rgb = grab_yu12_rgb(self.cam1, self.width, self.height, flush_n=0)
                with self._condition:
                    self._latest_left_rgb = left_rgb
                    self._latest_right_rgb = right_rgb
                    self.samples += 1
                    self._condition.notify_all()
                next_sample_ts += self.period
                sleep_s = next_sample_ts - time.monotonic()
                if sleep_s < -self.period:
                    next_sample_ts = time.monotonic()
                    sleep_s = 0.0
                if sleep_s > 0:
                    self._stop_event.wait(sleep_s)
            except Exception as exc:
                with self._condition:
                    self._error = exc
                    self._condition.notify_all()
                print(f"[ERROR] 双腕摄像头采样线程失败: {exc}")
                self._stop_event.set()


def stdin_read_char_nonblocking():
    if not sys.stdin.isatty():
        return None
    rlist, _, _ = select.select([sys.stdin], [], [], 0)
    if not rlist:
        return None
    try:
        return sys.stdin.read(1)
    except Exception:
        return None


def enable_stdin_cbreak() -> tuple[int | None, object | None]:
    if not sys.stdin.isatty():
        return None, None
    stdin_fd = sys.stdin.fileno()
    old_term = termios.tcgetattr(stdin_fd)
    tty.setcbreak(stdin_fd)
    return stdin_fd, old_term


def restore_stdin(stdin_fd, old_term) -> None:
    if stdin_fd is None or old_term is None:
        return
    termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_term)


def wait_for_enter_with_dual_camera_preview(
    get_frame,
    preview: DualCameraPreviewWindow,
    left_dev: int,
    right_dev: int,
) -> bool:
    if not preview.enabled:
        input("按 Enter 开始")
        return False
    if not sys.stdin.isatty():
        input("按 Enter 开始")
        return False

    stdin_fd = sys.stdin.fileno()
    old_term = termios.tcgetattr(stdin_fd)
    try:
        tty.setcbreak(stdin_fd)
        print("[INFO] 双腕摄像头预览中：终端按 Enter 开始推理；窗口内按 q 或 ESC 退出")
        while True:
            left_rgb, right_rgb = get_frame()
            if preview.update(left_rgb, right_rgb, left_dev, right_dev):
                print("[INFO] 双腕摄像头窗口收到退出指令，准备结束。")
                return True
            ch = stdin_read_char_nonblocking()
            if ch in ("\n", "\r"):
                return False
            time.sleep(0.01)
    finally:
        termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_term)


def open_gripper_max(arm, open_position: float = GRIPPER_OPEN_MAX) -> None:
    if open_position >= GRIPPER_OPEN_MAX and hasattr(arm, "openGripper"):
        arm.openGripper()
    else:
        arm.setGripperPosition(float(open_position))


class DirectGripperHoldProtector:
    """Pass policy gripper targets through, but stop over-closing after contact."""

    def __init__(
        self,
        arm,
        contact_dwell: float,
        contact_epsilon: float,
        hold_squeeze: float,
    ):
        self.arm = arm
        self.contact_dwell = max(float(contact_dwell), 0.0)
        self.contact_epsilon = max(float(contact_epsilon), 0.0)
        self.hold_squeeze = max(float(hold_squeeze), 0.0)
        self.mode = "direct"
        self.hold_target: float | None = None
        self.contact_samples: list[tuple[float, float]] = []

    def reset_open(self, open_position: float = GRIPPER_OPEN_MAX) -> None:
        self.mode = "direct"
        self.hold_target = None
        self.contact_samples.clear()
        open_gripper_max(self.arm, open_position=open_position)

    def command_policy(self, policy_target: float) -> float:
        target = float(np.clip(policy_target, 0.0, 1.0))
        current = float(self.arm.get_gripper_position())

        if self.mode == "holding" and self.hold_target is not None:
            if target >= self.hold_target:
                self.mode = "direct"
                self.hold_target = None
                self.contact_samples.clear()
            else:
                self.arm.setGripperPosition(self.hold_target)
                return self.hold_target

        if target < current:
            now = time.monotonic()
            self.contact_samples.append((now, current))
            if self.contact_dwell > 0.0:
                cutoff = now - self.contact_dwell
                self.contact_samples = [(t, p) for t, p in self.contact_samples if t >= cutoff]

            if self.contact_dwell <= 0.0:
                enough_time = True
            else:
                enough_time = (
                    len(self.contact_samples) >= 2
                    and now - self.contact_samples[0][0] >= self.contact_dwell - 1e-6
                )
            if len(self.contact_samples) >= 2:
                positions = [p for _, p in self.contact_samples]
                if enough_time and max(positions) - min(positions) <= self.contact_epsilon:
                    self.mode = "holding"
                    self.hold_target = float(np.clip(current - self.hold_squeeze, 0.0, 1.0))
                    self.arm.setGripperPosition(self.hold_target)
                    print(
                        "[INFO] gripper contact detected; "
                        f"holding at {self.hold_target:.3f} instead of chasing {target:.3f}"
                    )
                    return self.hold_target
        else:
            self.contact_samples.clear()

        self.arm.setGripperPosition(target)
        return target

    def start_sync(self, targets, duration_sec: float):
        stop_event = threading.Event()
        commanded_values: list[float] = []
        values = np.clip(np.asarray(targets, dtype=np.float64).reshape(-1), 0.0, 1.0)
        if len(values) == 0:
            return stop_event, None, commanded_values

        if len(values) == 1 or duration_sec <= 0.0:
            command_times = np.array([0.0], dtype=np.float64)
            command_values = np.array([values[-1]], dtype=np.float64)
        else:
            command_times = np.linspace(0.0, float(duration_sec), len(values))
            command_values = values

        def sync_loop():
            start_time = time.monotonic()
            for command_time, value in zip(command_times, command_values):
                if stop_event.is_set():
                    break
                wait_sec = start_time + float(command_time) - time.monotonic()
                if wait_sec > 0.0 and stop_event.wait(wait_sec):
                    break
                if stop_event.is_set():
                    break
                commanded_values.append(self.command_policy(float(value)))

        thread = threading.Thread(target=sync_loop, daemon=True)
        thread.start()
        return stop_event, thread, commanded_values


class DirectGripperPassthrough(DirectGripperHoldProtector):
    """Pass policy gripper targets directly to the SDK with no hold/contact logic."""

    def __init__(self, arm):
        self.arm = arm

    def reset_open(self, open_position: float = GRIPPER_OPEN_MAX) -> None:
        open_gripper_max(self.arm, open_position=open_position)

    def command_policy(self, policy_target: float) -> float:
        target = float(np.clip(policy_target, 0.0, 1.0))
        self.arm.setGripperPosition(target)
        return target


def _run_blocking_calls_concurrently(calls: list[tuple[str, object, tuple, dict]]) -> dict[str, object]:
    results: dict[str, object] = {}
    errors: list[tuple[str, BaseException]] = []
    lock = threading.Lock()

    def worker(name, fn, args, kwargs):
        try:
            value = fn(*args, **kwargs)
        except BaseException as exc:
            with lock:
                errors.append((name, exc))
            return
        with lock:
            results[name] = value

    threads = [
        threading.Thread(target=worker, args=(name, fn, fn_args, kwargs), daemon=True)
        for name, fn, fn_args, kwargs in calls
    ]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    if errors:
        name, exc = errors[0]
        raise RuntimeError(f"{name} 执行失败: {exc}") from exc
    return results


def reset_arms_to_init(
    arm0,
    arm1,
    init_pos0,
    init_euler0,
    init_pos1,
    init_euler1,
    gripper0: DirectGripperHoldProtector | None = None,
    gripper1: DirectGripperHoldProtector | None = None,
    open_position: float = GRIPPER_OPEN_MAX,
    reset_time_sec: float = 2.0,
) -> None:
    _run_blocking_calls_concurrently(
        [
            (
                "left reset",
                arm0.set_end_effector_pose_euler,
                (),
                {"pos": init_pos0, "euler": init_euler0, "tf": reset_time_sec},
            ),
            (
                "right reset",
                arm1.set_end_effector_pose_euler,
                (),
                {"pos": init_pos1, "euler": init_euler1, "tf": reset_time_sec},
            ),
        ]
    )
    if gripper0 is None:
        open_gripper_max(arm0, open_position=open_position)
    else:
        gripper0.reset_open(open_position=open_position)
    if gripper1 is None:
        open_gripper_max(arm1, open_position=open_position)
    else:
        gripper1.reset_open(open_position=open_position)


def build_dual_pose_trajectories(actions: np.ndarray, tcp_offset: np.ndarray):
    left_poses = []
    right_poses = []
    left_grippers = []
    right_grippers = []
    for action in actions:
        x_a0, y_a0, z_a0, roll0, pitch0, yaw0, g_open0, x_a1, y_a1, z_a1, roll1, pitch1, yaw1, g_open1 = action
        euler0 = np.deg2rad([roll0, pitch0, yaw0])
        euler1 = np.deg2rad([roll1, pitch1, yaw1])
        pos0 = tcp_position_to_flange([x_a0, y_a0, z_a0], euler0, tcp_offset)
        pos1 = tcp_position_to_flange([x_a1, y_a1, z_a1], euler1, tcp_offset)
        left_poses.append([*pos0.tolist(), *euler0.tolist()])
        right_poses.append([*pos1.tolist(), *euler1.tolist()])
        left_grippers.append(float(np.clip(g_open0, 0.0, 1.0)))
        right_grippers.append(float(np.clip(g_open1, 0.0, 1.0)))
    return (
        np.asarray(left_poses, dtype=np.float64),
        np.asarray(right_poses, dtype=np.float64),
        np.asarray(left_grippers, dtype=np.float64),
        np.asarray(right_grippers, dtype=np.float64),
    )


def solve_joint_waypoints_from_poses(
    arm,
    poses: np.ndarray,
    arm_name: str,
    ik_retries: int = 0,
    ik_retry_sleep_s: float = 0.0,
) -> np.ndarray:
    if not hasattr(arm, "solve_ik"):
        raise RuntimeError(f"{arm_name} arm 当前 SDK 没有 solve_ik()，无法使用 move_joint_waypoints 执行路径")
    if not hasattr(arm, "move_joint_waypoints"):
        raise RuntimeError(
            f"{arm_name} arm 当前 SDK 没有 move_joint_waypoints()，无法使用关节路点执行路径。"
            "请使用新版 startouch SDK，或设置 STARTOUCH_INTERFACE_DIR 指向 startouch_sdk/interface_py。"
        )

    poses = np.asarray(poses, dtype=np.float64)
    q_seed = list(np.asarray(arm.get_joint_positions(), dtype=np.float64))
    joint_waypoints = []
    max_attempts = 1 + max(0, int(ik_retries))
    retry_sleep_s = max(0.0, float(ik_retry_sleep_s))
    for i, pose in enumerate(poses):
        pos = pose[:3]
        euler = pose[3:6]
        quat = _euler_xyz_to_quat_wxyz(euler)
        input_seed = list(q_seed)
        q_output = None
        ok = False
        attempt_count = 0
        for attempt_index in range(max_attempts):
            attempt_count = attempt_index + 1
            try:
                q_sol, ok = arm.solve_ik(pos.tolist(), quat.tolist(), q_seed=input_seed)
            except Exception:
                if attempt_index + 1 < max_attempts:
                    if retry_sleep_s > 0.0:
                        time.sleep(retry_sleep_s)
                    continue
                raise
            q_output = list(np.asarray(q_sol, dtype=np.float64)) if q_sol is not None else None
            if ok:
                break
            if attempt_index + 1 < max_attempts and retry_sleep_s > 0.0:
                time.sleep(retry_sleep_s)
        if not ok:
            raise RuntimeError(
                f"{arm_name} IK failed at waypoint {i} after {attempt_count} attempts: "
                f"pos={np.round(pos, 6).tolist()}, euler={np.round(euler, 6).tolist()}, "
                f"seed={np.round(input_seed, 6).tolist()}"
            )
        q_seed = q_output
        joint_waypoints.append(q_seed)
    return np.asarray(joint_waypoints, dtype=np.float64)


def _joint_waypoints_motion_kwargs(trajectory_time_sec: float, speed_percent: float) -> dict:
    if speed_percent > 0.0:
        return {"speed_percent": float(speed_percent)}
    return {"time_sec": float(trajectory_time_sec)}


def execute_dual_move_joint_waypoints_with_grippers(
    arm0,
    arm1,
    left_poses,
    right_poses,
    left_grippers,
    right_grippers,
    trajectory_time_sec: float,
    joint_waypoint_speed_percent: float,
    gripper0: DirectGripperHoldProtector,
    gripper1: DirectGripperHoldProtector,
    ik_retries: int = 0,
    ik_retry_sleep_s: float = 0.0,
) -> dict[str, object]:
    left_joint_waypoints = solve_joint_waypoints_from_poses(
        arm0,
        left_poses,
        "left",
        ik_retries=ik_retries,
        ik_retry_sleep_s=ik_retry_sleep_s,
    )
    right_joint_waypoints = solve_joint_waypoints_from_poses(
        arm1,
        right_poses,
        "right",
        ik_retries=ik_retries,
        ik_retry_sleep_s=ik_retry_sleep_s,
    )
    stop0, thread0, commanded0 = gripper0.start_sync(left_grippers, duration_sec=trajectory_time_sec)
    stop1, thread1, commanded1 = gripper1.start_sync(right_grippers, duration_sec=trajectory_time_sec)
    results = {}
    try:
        motion_kwargs = _joint_waypoints_motion_kwargs(trajectory_time_sec, joint_waypoint_speed_percent)
        results = _run_blocking_calls_concurrently(
            [
                (
                    "left move_joint_waypoints",
                    arm0.move_joint_waypoints,
                    (left_joint_waypoints.tolist(),),
                    motion_kwargs,
                ),
                (
                    "right move_joint_waypoints",
                    arm1.move_joint_waypoints,
                    (right_joint_waypoints.tolist(),),
                    motion_kwargs,
                ),
            ]
        )
        return results
    finally:
        stop0.set()
        stop1.set()
        if thread0 is not None:
            thread0.join(timeout=1.0)
        if thread1 is not None:
            thread1.join(timeout=1.0)
        if len(left_grippers) > 0:
            final0 = gripper0.command_policy(float(left_grippers[-1]))
            if len(commanded0) < len(left_grippers):
                commanded0.append(final0)
        if len(right_grippers) > 0:
            final1 = gripper1.command_policy(float(right_grippers[-1]))
            if len(commanded1) < len(right_grippers):
                commanded1.append(final1)
        results["commanded_left_grippers"] = commanded0
        results["commanded_right_grippers"] = commanded1
