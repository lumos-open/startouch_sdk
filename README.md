# 此sdk为2026年3.1日更新

conda create -n startouch python=3.10

conda activate startouch

# 更新包列表
sudo apt-get update

# 安装 KDL 库
sudo apt-get install liborocos-kdl-dev

sudo apt-get install pybind11-dev

mkdir build

cd build

cmake ..

make

cd ..

# test

python interface_py/test_hardware.py

双臂同时进行控制例子，可自行切换单臂

python interface_py/ik.py

# 注：

创建机械臂实例时，后面紧跟延时sleep，机械臂规划至零位，否则会速度较快的达到零位；且不支持canfd

1. 单臂创建：

arm_controller = SingleArm(can_interface_ = "can0")

time.sleep(2)

2. 双臂创建：

arm_right = SingleArm(can_interface_="can0", enable_fd_=False)

arm_left = SingleArm(can_interface_="can1", enable_fd_=False)

time.sleep(2)

# lerobot数采部分

## 单臂主从遥操，需两个臂实习主从遥操

python lerobot_single_arm_tele.py

## 无遥操双臂数采

### 启动右臂

python interface_py/lerobot_ow_right_arm.py

### 另外开启一个终端，启动左臂

python interface_py/lerobot_ow_left_arm.py

## 双臂推理

python interface_py/lerobot_two_arm_inference.py