#!/bin/bash

# 设置要执行的应用程序路径
PROGRAM="./my_web_app"  # 将这里的 your_program 替换为实际的程序路径

# 如果没有传入参数，直接执行应用程序
if [ "$#" -eq 0 ]; then
    echo "No arguments provided. Running the program directly."
    $PROGRAM
else
    # 如果传入 -dbg 参数，使用 gdbserver 启动程序
    if [ "$1" == "-dbg" ]; then
        echo "Running with gdbserver..."
        gdbserver :1234 $PROGRAM
    else
        echo "Invalid argument. Running the program directly."
        $PROGRAM
    fi
fi

