"""
Dual Touch robot implementation（fastouch_sdk / pip 安装版）。

与 dual_touch.py 的 BaseRobot 接口保持一致，但机械臂驱动改为通过
``pip install fastouch_sdk``（或在 startouch_sdk 目录 ``pip install -e .``）
安装到环境中的顶层模块 ``startouchclass`` / ``startouch``，
不再从 sys.path 挂载本地 ``interface_py``；**关节/电机 CSV 标定默认优先使用**
``~/startouch-v1/param_csv_gripper``（与旧 ``dual_touch.py`` 一致），仍可用环境变量覆盖。

``libstartouch.so`` 会在 **扩展所在目录** 与 **进程当前工作目录** 下查找
``robot_kinematics.yaml``。若 pip 未把该文件装进 site-packages，请任选其一：

- 设置 ``STARTOUCH_SDK_ROOT`` 为 startouch_sdk 源码根目录（本模块会在其下查找
  ``src/config`` 或 ``config`` 中的 yaml）；
- 或设置 ``STARTOUCH_CONFIG_DIR`` 为直接包含 ``robot_kinematics.yaml`` 的目录；
- 或将 ``robot_kinematics.yaml`` 复制/软链到与 ``libstartouch.so`` 相同的目录。

**标定 CSV**：原版 ``startouchclass`` 用 ``../param_csv_gripper`` 相对 ``interface_py``。
pip 版把 ``startouchclass.py`` 放在 ``site-packages`` 时该相对路径会**指错目录**（常见症状：
机械臂持续朝某一方向漂移）。本模块的 ``SingleArm`` 会显式解析 ``param_csv_gripper``（见
``_find_param_csv_gripper_dir``）。查找顺序见该函数注释；若要用新 SDK 自带 CSV，请设置
``STARTOUCH_PARAM_DIR`` 为 ``…/startouch_sdk/src/param_csv_gripper``。
"""

import inspect
import os
import time
import threading
import queue
from contextlib import contextmanager
from enum import IntEnum
from pathlib import Path
from typing import Iterator, Optional, Union

import numpy as np
from loguru import logger
from scipy.spatial.transform import Rotation as R

try:
    from startouchclass import SingleArm as _SdkSingleArm
except Exception as e:
    import sys

    logger.error(
        "无法加载 startouchclass（当前 Python: {}）。请在本环境中安装 SDK，例如："
        "pip install fastouch_sdk，或在 startouch_sdk 目录执行 pip install -e .。"
        "若已安装仍失败，多为动态库未找到（见下方详细错误）。",
        sys.executable,
    )
    logger.error("详细错误: {}: {}", type(e).__name__, e)
    raise SystemExit(1) from e

from .base_bot import BaseRobot


class _ControlMode(IntEnum):
    """与 startouch-v1/interface_py/startouchclass 一致，供 ArmController 使用。"""

    MIT_MODE = 1
    POS_VEL_MODE = 2
    VEL_MODE = 3
    POS_FORCE_MODE = 4


def _normalize_motor_control_mode(mode: Union[int, str, _ControlMode]) -> int:
    if isinstance(mode, _ControlMode):
        return int(mode)
    if isinstance(mode, str):
        normalized = mode.strip().upper()
        if normalized in _ControlMode.__members__:
            return int(_ControlMode[normalized])
        raise ValueError(
            f"Unsupported motor_control_mode: {mode}. "
            f"Valid names: {', '.join(_ControlMode.__members__.keys())}"
        )
    if isinstance(mode, int):
        try:
            return int(_ControlMode(mode))
        except ValueError as exc:
            raise ValueError(
                f"Unsupported motor_control_mode: {mode}. "
                f"Valid values: {[int(m) for m in _ControlMode]}"
            ) from exc
    raise TypeError("motor_control_mode must be int, _ControlMode, or str.")


def _find_robot_kinematics_yaml_dir() -> Optional[Path]:
    """返回包含 ``robot_kinematics.yaml`` 的目录；找不到则返回 None。"""
    cfg = os.environ.get("STARTOUCH_CONFIG_DIR", "").strip()
    if cfg:
        p = Path(cfg).expanduser()
        p = p.resolve() if p.is_absolute() else (Path.cwd() / p).resolve()
        if (p / "robot_kinematics.yaml").is_file():
            return p

    root = os.environ.get("STARTOUCH_SDK_ROOT", "").strip()
    if root:
        rp = Path(root).expanduser().resolve()
        for rel in ("src/config", "config"):
            cand = rp / rel
            if (cand / "robot_kinematics.yaml").is_file():
                return cand

    try:
        import startouch

        ext = Path(startouch.__file__).resolve().parent
        if (ext / "robot_kinematics.yaml").is_file():
            return ext
    except Exception:
        pass

    # 与 param 标定类似：未设置 STARTOUCH_SDK_ROOT 时，从常见克隆路径推断
    import startouchclass as sc_mod

    scf = Path(sc_mod.__file__).resolve()
    if scf.parent.name == "interface_py":
        repo = scf.parent.parent
        for rel in ("src/config", "config"):
            cand = repo / rel
            if (cand / "robot_kinematics.yaml").is_file():
                return cand

    for sdk_root in (Path.home() / "下载/startouch_sdk", Path.home() / "startouch_sdk"):
        for rel in ("src/config", "config"):
            cand = sdk_root / rel
            if (cand / "robot_kinematics.yaml").is_file():
                return cand

    return None


def _find_param_csv_gripper_dir() -> Optional[Path]:
    """
    返回含 ``permutationMatrix.csv``、``pi_b.csv``、``pi_fr.csv`` 的目录。

    pip 安装的 ``startouchclass`` 若位于 ``site-packages``，其内置相对路径会指向
    ``lib/python3.x/`` 而非真实标定文件，导致控制异常。

    查找顺序（后者仅在前者失败时尝试）：

    1. ``STARTOUCH_PARAM_DIR`` 显式目录；
    2. ``STARTOUCH_V1_ROOT/param_csv_gripper``；
    3. **默认旧标定**：``~/startouch-v1/param_csv_gripper``（存在则用，对齐旧 ``dual_touch``）；
    4. ``STARTOUCH_SDK_ROOT`` 下 ``src/param_csv_gripper`` 或 ``param_csv_gripper``；
    5. 从 ``startouchclass.__file__`` 推断可编辑安装仓库；
    6. 常见 ``~/下载/startouch_sdk`` 等路径。
    """
    direct = os.environ.get("STARTOUCH_PARAM_DIR", "").strip()
    if direct:
        p = Path(direct).expanduser()
        p = p.resolve() if p.is_absolute() else (Path.cwd() / p).resolve()
        if (p / "permutationMatrix.csv").is_file():
            return p

    v1 = os.environ.get("STARTOUCH_V1_ROOT", "").strip()
    if v1:
        cand = Path(v1).expanduser().resolve() / "param_csv_gripper"
        if (cand / "permutationMatrix.csv").is_file():
            return cand

    v1_home = Path.home() / "startouch-v1" / "param_csv_gripper"
    if (v1_home / "permutationMatrix.csv").is_file():
        return v1_home

    root = os.environ.get("STARTOUCH_SDK_ROOT", "").strip()
    if root:
        rp = Path(root).expanduser().resolve()
        for sub in ("src/param_csv_gripper", "param_csv_gripper"):
            cand = rp / sub
            if (cand / "permutationMatrix.csv").is_file():
                return cand

    import startouchclass as sc_mod

    scf = Path(sc_mod.__file__).resolve()
    if scf.parent.name == "interface_py":
        repo = scf.parent.parent
        for sub in ("src/param_csv_gripper", "param_csv_gripper"):
            cand = repo / sub
            if (cand / "permutationMatrix.csv").is_file():
                return cand

    for sdk_root in (Path.home() / "下载/startouch_sdk", Path.home() / "startouch_sdk"):
        for sub in ("src/param_csv_gripper", "param_csv_gripper"):
            cand = sdk_root / sub
            if (cand / "permutationMatrix.csv").is_file():
                return cand

    return None


def _require_param_csv_gripper_dir() -> Path:
    d = _find_param_csv_gripper_dir()
    if d is not None:
        return d
    raise RuntimeError(
        "找不到 param_csv_gripper（需含 permutationMatrix.csv）。"
        "pip 版 startouchclass 的默认相对路径在 site-packages 下会失效。"
        "默认会尝试 ~/startouch-v1/param_csv_gripper；若仍失败，请设置 STARTOUCH_V1_ROOT、"
        "STARTOUCH_SDK_ROOT 或 STARTOUCH_PARAM_DIR。"
    )


def _require_robot_kinematics_yaml_dir() -> Path:
    d = _find_robot_kinematics_yaml_dir()
    if d is not None:
        return d
    raise RuntimeError(
        "找不到 robot_kinematics.yaml。libstartouch 会在扩展目录与当前工作目录下查找该文件。"
        "请设置环境变量 STARTOUCH_SDK_ROOT（startouch_sdk 源码根目录），"
        "或 STARTOUCH_CONFIG_DIR（直接包含该 yaml 的目录，例如 …/startouch_sdk/src/config）；"
        "也可把 yaml 放到与 libstartouch.so 同级目录。详见本模块文件头注释。"
    )


@contextmanager
def _startouch_hardware_init_cwd() -> Iterator[None]:
    """创建 ArmController 期间切换 cwd，以便从非 SDK 目录启动进程时仍能加载运动学配置。"""
    d = _require_robot_kinematics_yaml_dir()
    old = os.getcwd()
    try:
        os.chdir(d)
        logger.debug("startouch 硬件初始化 cwd: {}", d)
        yield
    finally:
        os.chdir(old)


class SingleArm(_SdkSingleArm):
    """
    与历史 startouch-v1/interface_py 对齐：部分 pip 自带的 ``startouchclass``
    未提供 ``solve_ik_euler``；并用显式路径构造 ``ArmController``，避免 site-packages
    下默认 ``param_csv_gripper`` 路径错误。
    """

    def __init__(
        self,
        can_interface_: str = "can0",
        gripper: bool = True,
        enable_fd_: bool = False,
        motor_control_mode: Union[int, str, _ControlMode] = 1,
    ):
        import startouch

        pdir = _require_param_csv_gripper_dir()
        logger.info("startouch 标定目录 param_csv_gripper: {}", pdir)
        permutation_matrix = str(pdir / "permutationMatrix.csv")
        pi_b = str(pdir / "pi_b.csv")
        pi_fr = str(pdir / "pi_fr.csv")
        mode_value = _normalize_motor_control_mode(motor_control_mode)
        arm_kwargs = dict(
            can_interface=can_interface_,
            enable_fd=enable_fd_,
            gripper_exist=gripper,
            permutation_matrix=permutation_matrix,
            pi_b=pi_b,
            pi_fr=pi_fr,
        )
        # 与 startouch-v1 一致：仅当底层 ArmController 支持时才传入 motor_control_mode
        try:
            sig = inspect.signature(startouch.ArmController)
            if "motor_control_mode" in sig.parameters:
                arm_kwargs["motor_control_mode"] = mode_value
        except Exception:
            arm_kwargs["motor_control_mode"] = mode_value

        try:
            self.arm = startouch.ArmController(**arm_kwargs)
        except TypeError:
            if "motor_control_mode" in arm_kwargs:
                arm_kwargs.pop("motor_control_mode", None)
                self.arm = startouch.ArmController(**arm_kwargs)
            else:
                raise
        logger.info(
            "ArmController 已创建；motor_control_mode 请求值={}（若底层不支持该参数则已在 TypeError 回退中省略）",
            mode_value,
        )

    def solve_ik_euler(self, pos, euler, q_seed=None):
        inherited = getattr(_SdkSingleArm, "solve_ik_euler", None)
        if inherited is not None:
            return inherited(self, pos, euler, q_seed)
        if q_seed is None:
            q, ok = self.arm.solve_ik(list(pos), list(euler))
        else:
            if len(q_seed) != 6:
                raise ValueError(f"q_seed must be length 6, got {len(q_seed)}")
            q, ok = self.arm.solve_ik(list(pos), list(euler), list(q_seed))
        return q, ok


class _ArmPoseWorker:
    """
    简单的末端位姿工作线程：
    - 通过队列异步接收 (pos, rpy, duration)
    - 在后台线程中依次调用机械臂的 set_end_effector_pose_euler
    """

    def __init__(self, arm):
        self.arm = arm
        self._q: "queue.Queue[tuple[np.ndarray, np.ndarray, float]]" = queue.Queue()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while True:
            pos, rpy, duration = self._q.get()
            try:
                print(time.time(), "set_end_effector_pose_euler")
                self.arm.set_end_effector_pose_euler(
                    pos=pos,
                    euler=rpy,
                    tf=duration,
                )
            except Exception as e:  # noqa: BLE001
                logger.error(f"末端位姿工作线程执行失败: {e}")

    def send(self, pos: np.ndarray, rpy: np.ndarray, duration: float):
        self._q.put(
            (
                np.asarray(pos, dtype=float),
                np.asarray(rpy, dtype=float),
                float(duration),
            )
        )


class DualTouchRobot(BaseRobot):
    """
    Dual Touch 机器人实现，接口尽量与 DualPiperRobot 保持一致。
    使用 pip 安装的 ``startouchclass.SingleArm``（经本模块子类补齐 solve_ik_euler）控制双臂。
    """

    def __init__(
        self,
        can_interfaces=("can1", "can0"),
        *args,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        self.lastq_l = None
        self.lastq_r = None
        self.xr_lastq_l = None
        self.xr_lastq_r = None
        self.left_arm = None
        self.right_arm = None
        self.can_interfaces = can_interfaces
        self._running = True
        # 初始化左右臂
        if len(can_interfaces) != 2:
            raise ValueError(
                "DualTouchRobot 需要两个 CAN 接口，例如 ('can0_left', 'can0_right')"
            )
        self.SingleArm = SingleArm
        self.SingleArm_right = SingleArm
        # left_if, right_if = can_interfaces
        right_if, left_if = can_interfaces
        try:
            with _startouch_hardware_init_cwd():
                self.left_arm = self.SingleArm(
                    can_interface_=left_if, gripper=True, enable_fd_=False
                )
                logger.info(f"左臂已连接到 CAN 接口: {left_if}")

                self.right_arm = self.SingleArm_right(
                    can_interface_=right_if, gripper=True, enable_fd_=False
                )
                logger.info(f"右臂已连接到 CAN 接口: {right_if}")
        except Exception as e:  # noqa: BLE001
            logger.error(f"DualTouchRobot 连接硬件失败: {e}")
            raise
        self.max = 0
        # 状态缓存结构尽量与 DualPiperRobot 保持一致
        self._state_cache = {
            "left_arm": {"qpos": [0.0] * 6},  # [x, y, z, rx, ry, rz]
            "right_arm": {"qpos": [0.0] * 6},
            "left_joint": {"qpos": [0.0] * 6},
            "right_joint": {"qpos": [0.0] * 6},
            "left_ctrl": {"qpos": [0.0] * 6},
            "right_ctrl": {"qpos": [0.0] * 6},
            "left_vr_pose": {"qpos": [0.0] * 7},
            "right_vr_pose": {"qpos": [0.0] * 7},
            "left_gripper": [0.0],
            "right_gripper": [0.0],
        }

        self.connected = True
        # 默认先把夹爪张开到中间位置，避免夹紧
        self.set_gripper(0.04, 0.04)
        time.sleep(0.5)
        self._state_thread = threading.Thread(
            target=self._state_update_loop, daemon=True
        )
        self._state_thread.start()
        # 启动状态监控线程
        # self.start_state_monitoring()

    # ------------------------------------------------------------------
    # BaseRobot 抽象接口实现
    # ------------------------------------------------------------------
    def connect(self):
        """重新连接 Dual Touch 机器人硬件。"""
        if self.connected:
            return True

        left_if, right_if = self.can_interfaces
        try:
            with _startouch_hardware_init_cwd():
                self.left_arm = self.SingleArm(
                    can_interface_=left_if, gripper=True, enable_fd_=False
                )
                logger.info(f"左臂已重新连接到 CAN 接口: {left_if}")

                self.right_arm = self.SingleArm_right(
                    can_interface_=right_if, gripper=True, enable_fd_=False
                )
                logger.info(f"右臂已重新连接到 CAN 接口: {right_if}")
        except Exception as e:  # noqa: BLE001
            logger.error(f"DualTouchRobot 重新连接硬件失败: {e}")
            return False

        self.connected = True
        self.start_state_monitoring()
        return True

    def disconnect(self):
        """断开与 Dual Touch 机器人的连接。"""
        self._running = False
        self.safe_stop()
        time.sleep(0.5)

        # SingleArm 内部持有的资源通过 cleanup 释放
        try:
            if self.left_arm:
                self.left_arm.cleanup()
        except Exception as e:  # noqa: BLE001
            logger.warning(f"释放左臂资源时出错: {e}")

        try:
            if self.right_arm:
                self.right_arm.cleanup()
        except Exception as e:  # noqa: BLE001
            logger.warning(f"释放右臂资源时出错: {e}")

        self.connected = False

    def _state_update_loop(self):
        """在后台线程中持续更新机器人状态。"""
        while self._running:
            try:
                if self.left_arm:
                    # 关节位置
                    left_joints = self.left_arm.get_joint_positions()
                    # 末端位姿 (位置 + 欧拉角)
                    left_pos, left_rpy = self.left_arm.get_ee_pose_euler()
                    # left_rpy = R.from_euler("xyz",left_rpy,degrees=False).as_matrix() @ np.array([[0,0,1],[0,1,0],[-1,0,0]])

                    left_rpy = R.from_matrix(
                        R.from_euler("xyz", left_rpy, degrees=False).as_matrix()
                        @ np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
                    ).as_euler("xyz", degrees=False)
                    left_grip = self.left_arm.get_gripper_position() / 20 
                    # print(left_pos)
                    self._state_cache["left_joint"]["qpos"] = list(
                        np.asarray(left_joints, dtype=float)
                    )
                    self._state_cache["left_ctrl"]["qpos"] = self._state_cache[
                        "left_joint"
                    ]["qpos"]
                    self._state_cache["left_arm"]["qpos"] = list(
                        np.asarray(left_pos, dtype=float)
                    ) + list(np.asarray(left_rpy, dtype=float))
                    self._state_cache["left_gripper"] = [float(left_grip)]

                if self.right_arm:
                    right_joints = self.right_arm.get_joint_positions()
                    right_pos, right_rpy = self.right_arm.get_ee_pose_euler()
                    right_rpy = R.from_matrix(
                        R.from_euler("xyz", right_rpy, degrees=False).as_matrix()
                        @ np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
                    ).as_euler("xyz", degrees=False)

                    right_grip = self.right_arm.get_gripper_position() / 20 

                    self._state_cache["right_joint"]["qpos"] = list(
                        np.asarray(right_joints, dtype=float)
                    )
                    self._state_cache["right_ctrl"]["qpos"] = self._state_cache[
                        "right_joint"
                    ]["qpos"]
                    self._state_cache["right_arm"]["qpos"] = list(
                        np.asarray(right_pos, dtype=float)
                    ) + list(np.asarray(right_rpy, dtype=float))
                    self._state_cache["right_gripper"] = [float(right_grip)]

            except Exception as e:  # noqa: BLE001
                logger.error(f"更新 DualTouchRobot 状态时出错: {e}")

            time.sleep(0.02)

    def set_gripper(
        self,
        left_value: float,
        right_value: float,
        speed: int = 1000,  # 与 DualPiper 接口兼容，实现在 startouch 中忽略该参数
        force: int = 0,  # 与 DualPiper 接口兼容
    ):
        """
        控制左右夹爪的开合程度。

        Args:
            left_value: 左夹爪开合程度，建议范围 [0, 1]，0 为闭合，1 为张开。
            right_value: 右夹爪开合程度，建议范围 [0, 1]。
        """
        _ = speed, force  # 占位以避免未使用参数告警
        left_value = float(np.clip(left_value * 16, 0.0, 1.0))
        right_value = float(np.clip(right_value * 16, 0.0, 1.0))
        # print(left_value)
        # left_value = 1-left_value
        # right_value = 1-right_value
        if self.left_arm:
            try:
                self.left_arm.setGripperPosition(left_value)
            except Exception as e:  # noqa: BLE001
                logger.error(f"设置左夹爪位置失败: {e}")

        if self.right_arm:
            try:
                self.right_arm.setGripperPosition(right_value)
            except Exception as e:  # noqa: BLE001
                logger.error(f"设置右夹爪位置失败: {e}")

        # 同步缓存中的 VR 手柄第 7 维（trigger）
        self._state_cache["left_vr_pose"]["qpos"][6] = left_value
        self._state_cache["right_vr_pose"]["qpos"][6] = right_value

    def set_gripper_UMI_infer(
        self,
        left_value: float,
        right_value: float,
        speed: int = 1000,  # 与 DualPiper 接口兼容，实现在 startouch 中忽略该参数
        force: int = 0,  # 与 DualPiper 接口兼容
    ):
        """
        控制左右夹爪的开合程度。

        Args:
            left_value: 左夹爪开合程度，建议范围 [0, 1]，0 为闭合，1 为张开。
            right_value: 右夹爪开合程度，建议范围 [0, 1]。
        """
        _ = speed, force  # 占位以避免未使用参数告警
        left_value = float(np.clip(left_value, 0.0, 1.0))
        right_value = float(np.clip(right_value, 0.0, 1.0))
        # print(left_value)
        # left_value = 1-left_value
        # right_value = 1-right_value
        if self.left_arm:
            try:
                self.left_arm.setGripperPosition(left_value)
            except Exception as e:  # noqa: BLE001
                logger.error(f"设置左夹爪位置失败: {e}")

        if self.right_arm:
            try:
                self.right_arm.setGripperPosition(right_value)
            except Exception as e:  # noqa: BLE001
                logger.error(f"设置右夹爪位置失败: {e}")

        # 同步缓存中的 VR 手柄第 7 维（trigger）
        self._state_cache["left_vr_pose"]["qpos"][6] = left_value
        self._state_cache["right_vr_pose"]["qpos"][6] = right_value

    def move_to_joint(self, action_joint, speed: float = 30.0):
        """
        按关节空间运动到目标位置。

        参数格式与 DualPiperRobot 保持一致:
            action_joint: 长度为 14 的列表/数组
                [左 6 关节, 左夹爪, 右 6 关节, 右夹爪]
        """
        _ = speed
        if action_joint is None:
            return

        action_joint = list(action_joint)
        if len(action_joint) < 14:
            raise ValueError("action_joint 长度必须为 14: [L6, L_grip, R6, R_grip]")

        # 左臂
        left_joints = action_joint[0:6]
        left_grip = float(action_joint[6])

        # 右臂
        right_joints = action_joint[7:13]
        right_grip = float(action_joint[13])

        if self.left_arm:
            try:
                self.left_arm.set_joint(left_joints, tf=2.0)
                self.left_arm.setGripperPosition(left_grip)
            except Exception as e:  # noqa: BLE001
                logger.error(f"左臂关节运动失败: {e}")

        if self.right_arm:
            try:
                self.right_arm.set_joint(right_joints, tf=2.0)
                self.right_arm.setGripperPosition(right_grip)
            except Exception as e:  # noqa: BLE001
                logger.error(f"右臂关节运动失败: {e}")

    def move_to_pose_for_xr(
        self,
        left_target: Optional[np.ndarray] = None,
        right_target: Optional[np.ndarray] = None,
        duration: float = 1.0,
    ):
        """
        面向 XR 控制的姿态接口，保持与 DualPiperRobot 同名接口。

        期望输入:
            left_target/right_target: 4x4 齐次变换矩阵 (np.ndarray)
        """
        if self.left_arm and left_target is not None:
            left_target = np.asarray(left_target, dtype=float)
            if left_target.shape != (4, 4):
                raise ValueError("left_target 应为 4x4 齐次变换矩阵")

    def move_to_pose(
        self,
        left_target: Optional[np.ndarray] = None,
        right_target: Optional[np.ndarray] = None,
    ):
        q_l,ok_l = self.left_arm.solve_ik_euler(left_target[0:3],left_target[3:6])
        q_r,ok_r = self.right_arm.solve_ik_euler(right_target[0:3],right_target[3:6])
        q_l = np.asarray(q_l, dtype=float)
        q_r = np.asarray(q_r, dtype=float)
        T = 100
        if ok_l:
            if self.xr_lastq_l is not None:
                for i in range(T):
                    self.left_arm.set_joint_raw((q_l - self.xr_lastq_l) / T * (i + 1) + self.xr_lastq_l,[0.0001,0.0001,0.0001,0.0001,0.0001,0.0001])
                    time.sleep(0.00001)
                    self.xr_lastq_l = q_l
            else:
                self.left_arm.set_joint_raw(q_l,[0.0001,0.0001,0.0001,0.0001,0.0001,0.0001])
                self.xr_lastq_l = q_l
        if ok_r:
            if self.xr_lastq_r is not None:
                for i in range(T):
                    self.right_arm.set_joint_raw((q_r - self.xr_lastq_r) / T * (i + 1) + self.xr_lastq_r,[0.0001,0.0001,0.0001,0.0001,0.0001,0.0001])
                    time.sleep(0.00001)
                    self.xr_lastq_r = q_r
            else:
                self.right_arm.set_joint_raw(q_r,[0.0001,0.0001,0.001,0.0001,0.0001,0.0001])
                self.xr_lastq_r = q_r
    def move_to_pose_dict(
        self,
        left_target: Optional[np.ndarray] = None,
        right_target: Optional[np.ndarray] = None,
        duration: float = 2.0,
    ):
        """
        按笛卡尔空间末端位姿运动到目标位置。

        参数格式与 DualPiperRobot 保持一致:
            left_target/right_target: [x, y, z, rx, ry, rz]
                位置单位: 米
                姿态单位: 弧度 (欧拉角 RPY)
        """
        if self.left_arm and left_target is not None:
            left_target = np.asarray(left_target, dtype=float)
            if left_target.shape[0] != 6:
                raise ValueError("left_target 应为长度 6 的数组 [x, y, z, rx, ry, rz]")
            pos = left_target[:3]
            rpy = left_target[3:6]
            try:
                # 使用较长的 tf 让关节插值更慢、更平滑
                q_r,ok_r = self.left_arm.solve_ik_euler(pos,rpy)
                if ok_r:
                    self.left_arm.set_joint(q_r, tf=duration)
                # self.left_arm.set_end_effector_pose_euler_raw(pos=pos, euler=rpy)

            except Exception as e:  # noqa: BLE001
                logger.error(f"左臂笛卡尔运动失败: {e}")

        if self.right_arm and right_target is not None:
            right_target = np.asarray(right_target, dtype=float)
            if right_target.shape[0] != 6:
                raise ValueError("right_target 应为长度 6 的数组 [x, y, z, rx, ry, rz]")
            pos = right_target[:3]
            rpy = right_target[3:6]
            try:
                # 使用较长的 tf 让关节插值更慢、更平滑
                q_r,ok_r = self.right_arm.solve_ik_euler(pos,rpy)
                if ok_r:
                    self.right_arm.set_joint(q_r, tf=duration)
                # self.right_arm.set_end_effector_pose_euler_raw(pos=pos, euler=rpy)
                # self.right_arm.set_end_effector_pose_euler_raw(pos=pos, euler=rpy)
            except Exception as e:  # noqa: BLE001
                logger.error(f"右臂笛卡尔运动失败: {e}")

    def move_to_pose_infer(
        self,
        left_target: Optional[np.ndarray] = None,
        right_target: Optional[np.ndarray] = None,
        duration: float = 1.0,
    ):
        """
        转到松灵坐标系

        """
        T_both=200
        sleep_time=0.000001
        max_mse = 0.4
        # input("Press Enter to execute pose...")
        if self.left_arm and left_target is not None:
            left_target = np.asarray(left_target, dtype=float)
            if left_target.shape[0] != 6:
                raise ValueError("left_target 应为长度 6 的数组 [x, y, z, rx, ry, rz]")
            pos = left_target[:3]
            rpy = left_target[3:6]
            rpy = R.from_matrix(
                R.from_euler("xyz", rpy, degrees=False).as_matrix()
                @ np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]])
            ).as_euler("xyz", degrees=False)
            print("left_target", rpy)
            try:
                # self.left_arm.set_end_effector_pose_euler(pos=pos, euler=rpy, tf=0.001)
                q_r,ok_r = self.left_arm.solve_ik_euler(pos,rpy)
                print(f"q_l:{q_r}")
                # input("Press Enter to execute left arm pose...")

                if ok_r:
                    q_r = np.asarray(q_r, dtype=float)
                    if self.lastq_l is None:
                        # print(f"first time set lastq_l: {self.left_arm.get_joint_positions()}")
                        self.lastq_l = self.left_arm.get_joint_positions()
                    else:
                        lastq_l = np.asarray(self.lastq_l, dtype=float)
                        a = ((lastq_l - q_r) ** 2).mean()
                        if a > max_mse:
                            # self.left_arm.go_home()
                            pass
                        else:
                            # print(f"left:{a}")
                            if self.lastq_l is not None:
                                T =T_both
                                t1 = time.time()
                                # input("Press Enter to execute left arm pose...")
                                for i in range(T):
                                    self.left_arm.set_joint_raw((q_r - lastq_l) / T * (i + 1) + lastq_l,[0.00000001,0.00000001,0.00000001,0.00000001,0.00000001,0.00000001])
                                    time.sleep(sleep_time)
                            self.lastq_l = q_r
                            # self.left_arm.set_joint_raw(q_r,[0.0000000,0.0000000,0.0000000,0.0000000,0.0000000,0.0000000])
                            # self.left_arm.set_joint(q_r,tf=0.02)

            except Exception as e:  # noqa: BLE001
                logger.error(f"左臂笛卡尔运动失败: {e}")
            # try:
            #     self.left_arm.set_end_effector_pose_euler_raw(pos=pos, euler=rpy)
            # except Exception as e:  # noqa: BLE001
            #     logger.error(f"左臂笛卡尔运动失败: {e}")

        if self.right_arm and right_target is not None:
            right_target = np.asarray(right_target, dtype=float)
            if right_target.shape[0] != 6:
                raise ValueError("right_target 应为长度 6 的数组 [x, y, z, rx, ry, rz]")
            pos = right_target[:3]
            rpy = right_target[3:6]
            rpy = R.from_matrix(
                R.from_euler("xyz", rpy, degrees=False).as_matrix()
                @ np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]])
            ).as_euler("xyz", degrees=False)
            try:
                # self.left_arm.set_end_effector_pose_euler(pos=pos, euler=rpy, tf=0.001)
                q_r,ok_r = self.right_arm.solve_ik_euler(pos,rpy)
                if ok_r:
                    q_r = np.asarray(q_r, dtype=float)
                    print(f"q_r:{q_r}")
                    # input("Press Enter to execute right arm pose...")
                    if self.lastq_r is None:
                        self.lastq_r = self.right_arm.get_joint_positions()
                    else:
                        lastq_r = np.asarray(self.lastq_r, dtype=float)
                        a = ((lastq_r - q_r) ** 2).mean()
                        if a > max_mse:
                            # self.right_arm.go_home()
                            pass
                        else:
                            # print(f"left:{a}")
                            if self.lastq_r is not None:
                                T = T_both
                                for i in range(T):
                                    self.right_arm.set_joint_raw((q_r - lastq_r) / T * (i + 1) + lastq_r,[0.00000001,0.00000001,0.00000001,0.00000001,0.00000001,0.00000001])
                                    time.sleep(sleep_time)
                            self.lastq_r = q_r
                            
                            # self.right_arm.set_joint(q_r,tf = 0.02)

            except Exception as e:  # noqa: BLE001
                logger.error(f"右臂笛卡尔运动失败: {e}")
            # try:
            #     self.right_arm.set_end_effector_pose_euler_raw(pos=pos, euler=rpy)
            # except Exception as e:  # noqa: BLE001
            #     logger.error(f"右臂笛卡尔运动失败: {e}")

    def go_home_new(self):
        left_joint = self.left_arm.get_joint_positions()
        right_joint = self.right_arm.get_joint_positions()
        for i in range(100000):
            left_j = (np.array(left_joint) - np.zeros(6)) / 100000 * (99999 - i)
            right_j = (np.array(right_joint) - np.zeros(6)) / 100000 * (99999 - i)
            self.left_arm.set_joint_raw(left_j, [1, 1, 1, 1, 1, 1])
            self.right_arm.set_joint_raw(right_j, [1, 1, 1, 1, 1, 1])
            time.sleep(0.00001)
        self.left_arm.set_joint_raw([0, 0, 0, 0, 0, 0], [1, 1, 1, 1, 1, 1])
        self.right_arm.set_joint_raw([0, 0, 0, 0, 0, 0], [1, 1, 1, 1, 1, 1])

    def go_home(self):
        """让机器人回到预定义的 Home 位姿。"""
        if self.left_arm:
            try:
                self.left_arm.go_home()
            except Exception as e:  # noqa: BLE001
                logger.error(f"左臂回到 home 失败: {e}")

        if self.right_arm:
            try:
                self.right_arm.go_home()
            except Exception as e:  # noqa: BLE001
                logger.error(f"右臂回到 home 失败: {e}")

        # 简单设置一个大致的 VR 初始姿态，具体数值可根据实际标定调整
        self._state_cache["left_vr_pose"]["qpos"] = [
            0.0,
            0.0,
            0.3,
            0.0,
            1.57,
            0.0,
            0.5,
        ]
        self._state_cache["right_vr_pose"]["qpos"] = [
            0.0,
            0.0,
            0.3,
            0.0,
            1.57,
            0.0,
            0.5,
        ]

    def safe_stop(self):
        """安全停止机器人运动。"""
        # startouch 的底层接口未在此处完全暴露，先尝试调用底层 stop 接口，如不存在则忽略
        for arm_name, arm in (("left", self.left_arm), ("right", self.right_arm)):
            if arm is None:
                continue
            try:
                # 如果底层实现提供了 stop 接口，则调用
                if hasattr(arm, "arm") and hasattr(arm.arm, "stop"):
                    arm.arm.stop()
            except Exception as e:  # noqa: BLE001
                logger.warning(f"{arm_name} 臂 safe_stop 调用失败: {e}")

    def get_state_pos(self):
        """
        获取当前关节角状态，返回格式与 DualPiperRobot 保持一致:
            [左 6 关节, 左夹爪, 右 6 关节, 右夹爪]
        """
        return (
            self._state_cache["left_joint"]["qpos"]
            + self._state_cache["left_gripper"]
            + self._state_cache["right_joint"]["qpos"]
            + self._state_cache["right_gripper"]
        )

    def get_state_endpos(self):
        """
        获取当前末端位姿 (笛卡尔空间)。

        Returns:
            (left, right)
            其中 left/right = [x, y, z, rx, ry, rz, grip]
        """

        return (
            self._state_cache["left_arm"]["qpos"] + self._state_cache["left_gripper"],
            self._state_cache["right_arm"]["qpos"] + self._state_cache["right_gripper"],
        )


    def get_current_state(self):
        """获取机器人完整当前状态，结构与 DualPiperRobot 一致。"""
        return {
            "left_arm": self._state_cache["left_arm"],
            "right_arm": self._state_cache["right_arm"],
            "left_gripper": self._state_cache["left_gripper"],
            "right_gripper": self._state_cache["right_gripper"],
            "left_joint": self._state_cache["left_joint"],
            "right_joint": self._state_cache["right_joint"],
            "left_ctrl": self._state_cache["left_ctrl"],
            "right_ctrl": self._state_cache["right_ctrl"],
            "left_vr_pose": self._state_cache["left_vr_pose"],
            "right_vr_pose": self._state_cache["right_vr_pose"],
            "body": {"qpos": []},
        }
