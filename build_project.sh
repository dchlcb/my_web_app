#!/bin/bash

# 退出脚本时如果出现错误
set -e

# 项目根目录
PROJECT_DIR=$(pwd)

# 构建目录
BUILD_DIR="$PROJECT_DIR/build"

# 检查是否存在构建目录，如果存在则删除
if [ -d "$BUILD_DIR" ]; then
  echo "构建目录 $BUILD_DIR 已存在，正在删除..."
  rm -rf "$BUILD_DIR"
fi

# 创建构建目录
echo "创建构建目录 $BUILD_DIR"
mkdir "$BUILD_DIR"

# 进入构建目录
cd "$BUILD_DIR"

# 运行 cmake 配置项目
echo "运行 cmake 配置项目..."
cmake ..

# 运行 make 编译项目
echo "运行 make 编译项目..."
make

# 输出构建结果
echo "构建完成！可执行文件和库已输出到 $PROJECT_DIR/bin 目录。"
