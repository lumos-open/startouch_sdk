import numpy as np
import sys, termios, tty, time, math
from typing import Tuple
import os
import signal
import atexit
from startouchclass import SingleArm

class SafeArmController:
    """安全的机械臂控制器，确保程序退出时执行go_home"""
    def __init__(self, can_interface):
        self.can_interface = can_interface
        self.arm_controller = None
        self.initialized = False
        self._setup_signal_handlers()
    def _setup_signal_handlers(self):
        """设置信号处理器"""
        # 处理常规退出信号
        signal.signal(signal.SIGINT, self._signal_handler)   # Ctrl+C
        signal.signal(signal.SIGTERM, self._signal_handler)  # kill命令
        signal.signal(signal.SIGTSTP, self._signal_handler)  # Ctrl+Z
        
        # 注册atexit函数
        atexit.register(self._cleanup)
        
    def _signal_handler(self, signum, frame):
        """信号处理函数"""
        print(f"\n{self.can_interface}: 收到信号 {signum}，正在安全停止...")
        self._cleanup()
        sys.exit(0)
        
    def _cleanup(self):
        """清理函数，确保执行go_home"""
        if self.initialized and self.arm_controller:
            try:
                print(f"{self.can_interface}: 正在返回初始位置...")
                pos = [0, -0.1, -0.05, -1.29, 0, 0]         # mot2 up & mot4 up
                self.arm_controller.set_joint(pos, tf=2)
                time.sleep(2)
                print(f"{self.can_interface}: 已安全停止")
            except Exception as e:
                print(f"{self.can_interface}: 返回初始位置时出错: {e}")
            finally:
                self.initialized = False
                
    def initialize(self):
        """初始化机械臂控制器"""
        print(f"正在初始化 {self.can_interface} 的机械臂控制器...")
        self.arm_controller = SingleArm(can_interface_=self.can_interface, enable_fd_=False)
        # self.arm_controller = SingleArm(can_interface_=self.can_interface, enable_fd_=False)
        self.initialized = True
        print(f"{self.can_interface}: 初始化完成")
        self.arm_controller.go_home()
        time.sleep(3)
        return self.arm_controller
        
    def run(self):
        """运行机械臂控制循环"""
        if not self.initialized:
            self.initialize()
            
        count = 0
        try:
            print(f"{self.can_interface}: 开始执行，按 Ctrl+C 停止")
            while True:
                pos = [0, 1.54, -3, 1.29, 0, 0]        # 垂直 & mot4 down
                self.arm_controller.set_joint(pos, tf=3)
                time.sleep(3)
                pos = [2.7, 1.54, -3, 1.29, 1.6, -2.7]   # mot1 right & mot3 up & mot4 up & mot5 left &mot6 cw
                self.arm_controller.set_joint(pos, tf=3)
                time.sleep(3)
                pos = [-2.7, 1.54, -3, 0, -1.6, 2.7]    # mot1 left & mot5 left &mot6 ccw
                self.arm_controller.set_joint(pos, tf=5)
                time.sleep(5)
                # self.arm_controller.setGripperDistance(0.085)
                # time.sleep(2)
                pos = [0, 0, 0, 0, 0, 0]    # mot1 left & mot5 left &mot6 ccw
                self.arm_controller.set_joint(pos, tf=5)
                #self.arm_controller.go_home()               # mot3 down
                time.sleep(5)
                self.arm_controller.setGripperDistance(0, 10, 0.5)
                time.sleep(2)#结束
                pos = [0, 3.2, -1.5, 0, 0, 0]           # mot2 down 3.3
                self.arm_controller.set_joint(pos, tf=5)
                time.sleep(5)
                pos = [0, -0.13, 0, -1.29, 0, 0]         # mot2 up & mot4 up
                self.arm_controller.set_joint(pos, tf=5)
                time.sleep(5)
                count = count+1
                print(f"count: {count}")

        except Exception as e:
            print(f"{self.can_interface}: 发生错误: {e}")
            self._cleanup()

def main(can_interface):
    """
    主函数，控制指定CAN接口的机械臂
    Args:
        can_interface: CAN接口名称，如 "can0", "can1" 等
    """
    # 创建安全的机械臂控制器
    controller = SafeArmController(can_interface)
    controller.run()

if __name__ == "__main__":
    # 检查命令行参数
    if len(sys.argv) > 1:
        # 从命令行参数获取CAN接口
        can_if = sys.argv[1]
    else:
        # 默认使用 can0
        can_if = "can0"
        print(f"未指定CAN接口，使用默认值: {can_if}")
        print(f"使用方式: python {sys.argv[0]} [can_interface]")
        print(f"示例: python {sys.argv[0]} can1")
    
    # 运行主函数
    try:
        main(can_if)
    except KeyboardInterrupt:
        print(f"\n{can_if}: 程序被用户中断")
    except Exception as e:
        print(f"{can_if}: 程序发生错误: {e}")
    finally:
        print(f"{can_if}: 程序结束")
