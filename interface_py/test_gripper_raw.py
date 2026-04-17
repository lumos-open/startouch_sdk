from startouchclass import SingleArm
import time

# 创建机械臂连接  连接接口为"can0"
arm_controller = SingleArm(can_interface_ = "can0")
time.sleep(2)

arm_controller.setGripperPosition_raw(1)
time.sleep(2)
arm_controller.setGripperPosition_raw(0)
time.sleep(2)
arm_controller.setGripperPosition_raw(0.5)
time.sleep(20)
print(arm_controller.get_ee_pose_euler())
print(arm_controller.get_ee_pose_quat())
time.sleep(2)
arm_controller.cleanup()

