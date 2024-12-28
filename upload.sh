#!/bin/bash

# 设置目标路径
TARGET_DIR="/root/web_app"

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

# 传输 lib 文件夹
echo "正在传输 lib 文件夹..."
adb push ./lib $TARGET_DIR/

# 传输 web_demo 文件(二进制文件)
echo "正在传输二进制文件 web_demo ..."
adb push ./web_demo $TARGET_DIR/


# 传输 run_program.sh 文件
echo "正在传输 run_program.sh 文件..."
adb push ./run_program.sh $TARGET_DIR/

echo "所有文件已成功传输到 $TARGET_DIR"
