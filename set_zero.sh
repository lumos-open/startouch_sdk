#!/bin/bash

INTERFACE="can0"
BITRATE=1000000

# CAN ID列表
IDS=(001 002 003 004 005 006 007)

# 数据尾字节（根据需求调整）
DATA_SUFFIXES=(FC FE FD)

# 是否需要延迟（0表示不延迟，非0值为秒数）
DELAY=0.2   # 设为0.1即可开启0.1秒延迟

# 函数：发送单条CAN帧
send_can() {
    local id=$1
    local data=$2
    cansend $INTERFACE ${id}#$data
    if [ $? -ne 0 ]; then
        echo "发送失败: $id#$data" >&2
        exit 1
    fi
    if [ $(echo "$DELAY > 0" | bc) -eq 1 ]; then
        sleep $DELAY
    fi
}

# 检查CAN接口是否存在
if ! ip link show $INTERFACE &>/dev/null; then
    echo "错误：接口 $INTERFACE 不存在" >&2
    exit 1
fi

# 配置CAN接口
sudo ip link set $INTERFACE down 2>/dev/null   # 忽略已down的错误
sudo ip link set $INTERFACE type can bitrate $BITRATE
sudo ip link set $INTERFACE up
if [ $? -ne 0 ]; then
    echo "错误：无法启动 $INTERFACE" >&2
    exit 1
fi

# 发送所有组合
for id in "${IDS[@]}"; do
    for suffix in "${DATA_SUFFIXES[@]}"; do
        # 构造8字节数据：前7字节为FF，最后一字节为suffix
        data="FFFFFFFFFFFFFF$suffix"
        send_can "$id" "$data"
    done
done

# 可选：发送完成后关闭接口（根据需要注释掉）
sudo ip link set $INTERFACE down

echo "标零完成。"
