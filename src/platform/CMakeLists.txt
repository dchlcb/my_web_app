# 搜索所有源文件
file(GLOB APP_SOURCES "*.c")
file(GLOB APP_HEADERS "*.h")

# 设置 platform 静态库
add_library(platform STATIC ${APP_SOURCES})

# 设置 platform 库的头文件搜索路径
target_include_directories(platform PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)


