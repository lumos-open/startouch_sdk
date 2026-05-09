#!/bin/bash
# 文件名：kill_joint_test_simple.sh

echo "正在终止所有 joint_single_test_auto.py 进程..."

# 查找并优雅终止
pkill -f "joint_single_test_auto.py" && echo "已发送终止信号"

# 等待2秒
echo "wait 15s"
sleep 15

# 检查并强制终止未响应的进程
if pgrep -f "joint_single_test_auto.py" > /dev/null; then
    echo "仍有进程运行，强制终止..."
    pkill -9 -f "joint_single_test_auto.py"
fi

echo "完成！"