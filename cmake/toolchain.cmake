# toolchain.cmake

# 设置交叉编译工具链
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

# 设置交叉编译工具路径
#
# set(CMAKE_C_COMPILER /path/to/your/toolchain/arm-gcc)
# set(CMAKE_CXX_COMPILER /path/to/your/toolchain/arm-g++)

set(CMAKE_C_COMPILER /opt/atk-dlrk3568-5_10_sdk-toolchain/usr/bin/aarch64-buildroot-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /opt/atk-dlrk3568-5_10_sdk-toolchain/usr/bin/aarch64-buildroot-linux-gnu-g++)

# # 设置目标架构
# set(CMAKE_FIND_ROOT_PATH /path/to/your/sysroot)

# # 设置交叉编译工具链的系统根路径
# set(CMAKE_SYSROOT /path/to/your/sysroot)

# # 如果使用某些库文件，CMake 可能需要找到交叉编译工具链中的工具
# set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
# set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

