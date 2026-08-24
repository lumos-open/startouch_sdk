# StarTouch SDK 安装与运行

本文以 RK3568 CPU-CAN 已验证分支 `charlie3568cpucan` 为准。

## 1. 目标环境

```text
Linux: aarch64 Ubuntu 20.04
Python: 3.10
conda env: lumostouch
SDK root: /home/lumos/startouch_sdk
left arm: can0
right arm: can1
joint command: 400 Hz
gripper command: 200 Hz
```

内核 CAN 驱动不在 SDK 仓，见 `/home/lumos/3568canko/README_INSTALL.md`。

## 2. 获取 SDK

```bash
cd /home/lumos
git clone -b charlie3568cpucan \
  https://github.com/lumos-open/startouch_sdk.git startouch_sdk
cd /home/lumos/startouch_sdk
```

## 3. 系统依赖

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake pkg-config \
  libeigen3-dev libyaml-cpp-dev liborocos-kdl-dev libtinyxml2-dev pybind11-dev
```

## 4. Python 环境

```bash
source /home/lumos/miniforge3/etc/profile.d/conda.sh
conda create -n lumostouch python=3.10 -y
conda activate lumostouch
python -m pip install --upgrade pip setuptools wheel
```

## 5. 安装依赖和 binding

```bash
cd /home/lumos/startouch_sdk
python -m pip install -r requirements.txt
python -m pip install --no-deps -e .
```

`scikit-build-core`、`pybind11` 和 `packaging` 已在 `pyproject.toml` 的
`build-system.requires` 声明，由 pip 的临时隔离构建环境处理，不安装为生产运行依赖。

加载统一环境：

```bash
source /home/lumos/startouch_sdk/env.sh
```

`env.sh` 默认激活 `lumostouch`，设置：

```text
STARTOUCH_SDK_ROOT=/home/lumos/startouch_sdk
STARTOUCH_CONFIG_DIR=/home/lumos/startouch_sdk/src/config
STARTOUCH_PARAM_DIR=/home/lumos/startouch_sdk/src/param_csv_gripper
LD_LIBRARY_PATH=/home/lumos/startouch_sdk/src:...
PYTHONPATH=/home/lumos/startouch_sdk/interface_py:...
```

## 6. 安装验证

```bash
cd /home/lumos/startouch_sdk
./verify_install.sh
python -m pip check
```

预期版本：

```text
startouch 0.1.8
startouchclass 0.1.8
```

查看实际加载来源：

```bash
python - <<'PY'
import startouch, startouchclass
print(startouch.__version__, startouch.__file__)
print(startouchclass.__version__, startouchclass.__file__)
PY
```

`startouch` Python extension 必须最终链接到本 SDK 的 `libstartouch.so`，且 `ldd` 不得
出现 `not found`。`verify_install.sh` 会自动检查。

默认 wheel 不打包 `deps/linux-aarch64` 中的编译器运行库，避免把为较新 glibc 构建的
`libgcc/libstdc++` 带到 Ubuntu 20.04。只有目标系统 ABI 明确匹配时才显式启用：

```bash
CMAKE_ARGS='-DSTARTOUCH_BUNDLE_RUNTIME_DEPS=ON' \
  python -m pip install --no-deps -e .
```

117 的已验证安装保持默认 `OFF`。

## 7. 运行配置

正式配置唯一来源：

```text
/home/lumos/startouch_sdk/src/config/robot_kinematics.yaml
```

夹爪参数：

```text
/home/lumos/startouch_sdk/src/param_csv_gripper/
```

不要从 `startouchlib/config` 无条件覆盖这份部署配置。修改 tool、夹爪类型、补偿或安全
开关后必须重新进行真机验收。

当前 Luna client 只开启重力补偿，科氏和惯量补偿由启动环境关闭。SDK 配置中的安全
开关属于部署策略，不应仅因安装而自动改写。

## 8. CAN 准备

先按独立驱动仓校验：

```bash
cd /home/lumos/3568canko
./verify_driver.sh
```

单独使用 SDK 时可加载并配置：

```bash
sudo /home/lumos/3568canko/load_driver.sh
```

正式 Luna client 不需要提前执行该命令，`run_client_rt.sh` 会加载驱动并重新配置
can0/can1。

## 9. Python API 最小检查

以下只验证 binding 和 dry-run 构造，不连接 CAN：

```bash
source /home/lumos/startouch_sdk/env.sh
python - <<'PY'
import startouch

arm = startouch.ArmController(
    gripper_exist=True,
    can_interface="can0",
    enable_fd=False,
    dry_run=True,
)
print("dry-run ArmController created", arm)
arm.cleanup()
PY
```

真实机械臂测试必须通过项目的正式启动/测试脚本进行，不要把 import 验证直接改成动作命令。

## 10. C++ 底层重新打包

修改 `/home/lumos/startouchlib` 后，按其 `README_INSTALL.md` 构建并显式同步库和公开
头文件，再重新执行：

```bash
python -m pip install --no-deps -e /home/lumos/startouch_sdk
/home/lumos/startouch_sdk/verify_install.sh
```

当前 aarch64 生产库：

```text
src/libstartouch.so.arm64
```

发布时应同时更新 `src/libstartouch.so` 和 `src/libstartouch.so.arm64`，并确保二者 SHA
一致。若修改了 `startouchlib` 的 C++ 接口（例如 0.1.8 的夹爪力控接口），必须先在
目标 aarch64 Ubuntu 20.04 环境重新编译 `libstartouch.so`，再将同一产物更新到上述
两个文件。`verify_install.sh` 会检查每个发布库是否导出了新接口符号；不同 Ubuntu/
架构产物之间不要求 SHA 相同。
