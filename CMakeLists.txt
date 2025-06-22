# 设置 CMake 最低版本和项目名称
cmake_minimum_required(VERSION 3.10)
project(MyProgram)

include(ProcessorCount)
ProcessorCount(N)
if(NOT N EQUAL 0)
    message(STATUS "Detected CPU cores: ${N}")  # 打印核心数
    set(CMAKE_BUILD_PARALLEL_LEVEL ${N})        # 设置并行编译线程数
else()
    message(WARNING "Failed to detect CPU cores! Using default build parallelism.")
endif()

# 设置 C 标准
# set(CMAKE_C_STANDARD 99)
set(CMAKE_BUILD_TYPE Debug)

# 设置输出目录
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)  # 静态库输出路径
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)  # 动态库输出路径
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)  # 可执行文件输出路径
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}) # 设置最终的可执行文件输出路径


# 包含子目录
add_subdirectory(src)
