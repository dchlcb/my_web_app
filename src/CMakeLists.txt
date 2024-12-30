# 指定 CMake 最低版本
cmake_minimum_required(VERSION 3.10)


# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../lib)  # 静态库输出路径
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../lib)  # 动态库输出路径
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)  # 可执行文件输出路径


# 添加 app 和 mongoose 子目录
add_subdirectory(app)
add_subdirectory(mongoose)


# 使用 MinGW 编译器时启用 POSIX
if(MINGW)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D_POSIX_C_SOURCE=200809L")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_POSIX_C_SOURCE=200809L")
endif()

# 启用 POSIX 特性
if(UNIX)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D_POSIX_C_SOURCE=200809L")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_POSIX_C_SOURCE=200809L")
endif()


# 指定源文件
add_executable(web_demo main.c)
# 链接静态库和动态库
target_link_libraries(web_demo PUBLIC app mongoose)
target_link_libraries(web_demo PUBLIC pthread rt)


#动态库路径通过 $ORIGIN（Linux 和 macOS）相对于可执行文件的路径自动解析为 ./lib
# set_target_properties(web_demo PROPERTIES INSTALL_RPATH "$ORIGIN/../lib")


# 包含头文件目录
target_include_directories(web_demo PUBLIC
    ${CMAKE_SOURCE_DIR}/src/app
    ${CMAKE_SOURCE_DIR}/src/mongoose
)