#!/bin/bash

# 设置目标路径
TARGET_DIR="/root/my_web_app"

# 检查是否已经连接到设备
adb devices | grep -w "device" > /dev/null
if [ $? -ne 0 ]; then
    echo "没有检测到连接的设备，请确保设备已连接并启用ADB调试。"
    exit 1
fi

# 创建目标目录
echo "创建目标目录：$TARGET_DIR"
adb shell "mkdir -p $TARGET_DIR"

# 传输 certs 文件夹
echo "正在传输 certs 文件夹..."
adb push ./certs $TARGET_DIR/

# 传输 web_root 文件夹
echo "正在传输 web_root 文件夹..."
adb push ./web_root $TARGET_DIR/

# 传输 my_web_app 文件夹
echo "正在传输 my_web_app 文件夹..."
adb push ./my_web_app $TARGET_DIR/

echo "所有文件已成功传输到 $TARGET_DIR"
