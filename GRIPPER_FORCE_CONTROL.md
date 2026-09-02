# TypeNex 夹爪力控说明

## 控制定义

- 电机：DM-J4310-2EC V1.2，无触觉/指尖力传感器。
- 接口：`setGripperDistanceEffort(distance, effort_nm)`。
- `effort_nm`：电机输出轴目标/限制力矩，底层转换为力位混控的相电流上限；不是独立的力矩闭环。
- 运行时优先读取电机 `TMAX`，按 `i_des = effort_nm / TMAX` 换算；读取失败时才使用 `KT_Value * Imax` 或 YAML 兜底值。

## 函数调用链

### 命令链

```text
[SDK层]
test_gripper_force_control.py
  -> SingleArm.setGripperDistanceEffort(distance, effort_nm)
  -> pybind: ArmController::setGripperDistanceEffort(...)

[lib层]
ArmController::publish_gripper_distance_effort_command(...)
  -> control_loop(): current_limit_norm = effort_nm / TMAX
  -> GripperComponent::set_distance_effort(...)
  -> DMDeviceCollection::pos_force_control_one(...)
  -> CanPacketEncoder::create_pos_force_control_command(...)

[电机接口]
SocketCAN: CAN ID = 0x300 + ESC_ID（当前 0x307）
  -> DM4310 POS_FORCE: p_des + v_des + i_des
```

### 反馈链

```text
[传感器接口]
DM4310 编码器 + 驱动器电流估算 + 温度采样
  -> CAN反馈：位置、速度、12位T、MOS温度、线圈温度、状态码
  注：无触觉传感器，T不是指尖夹持力实测值。

[lib层]
DMCANDevice::callback(...)
  -> CanPacketDecoder::parse_motor_state_data(...)
  -> tau = map(T, 0..4095, -TMAX..TMAX)
  -> Motor::get_torque()
  -> ArmController::get_gripper_state()

[SDK层]
SingleArm.get_gripper_state()
  -> state.effort_nm（测试脚本显示为 tau）
```

## 极限值

| 项目 | 当前值/范围 | 处理方式 |
|---|---:|---|
| TypeNex 指尖距离 | `0 ~ 0.24073 m` | 超界夹紧到范围内 |
| TypeNex 电机位置 | `-1.22173 ~ 0 rad` | 由距离映射 |
| `effort_nm` | `0 ~ TMAX`（运行时读取） | 负数/非有限值报错，超上限夹紧 |
| `i_des` 标幺电流 | `0 ~ 1` | CAN 中放大10000，范围 `0 ~ 10000` |
| 力位最大速度 | 当前 `1 rad/s` | 配置允许 `(0,100] rad/s` |
| `tau` 反馈 | `-TMAX ~ TMAX` | 12位解码，分辨率为 `2*TMAX/4095` |
| 反馈超时 | `100 ms` | 力控退出并切回 MIT 模式 |
| 电机状态 | `enabled=true` 且状态码 `< 0x8` | 失能或故障立即退出力控 |
| 手册额定/峰值力矩 | `3.5 / 12.5 Nm` | 电机规格；软件上限仍以运行时读取的 `TMAX` 为准 |

当前未增加应用层安全力矩范围；实际使用应结合夹爪结构、物体强度和温升另行限定。

## 测试

```bash
python interface_py/test_gripper_force_control.py \
  --execute --can can1 --target-distance 0.0 \
  --effort-nm 0.3 --hold-seconds 3
```

接触物体后关注：`active=1`、`valid=1`、`err=0x1`，以及稳定后的 `tau`。
