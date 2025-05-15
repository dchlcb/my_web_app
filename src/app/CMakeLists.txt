# 搜索所有源文件
file(GLOB APP_SOURCES "*.cpp")
file(GLOB APP_HEADERS "*.h")

# 设置 app 静态库
add_library(app STATIC ${APP_SOURCES})

# 设置 app 库的头文件搜索路径
target_include_directories(app PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR} 
    /opt/atk-dlrk3568-5_10_sdk-toolchain/aarch64-buildroot-linux-gnu/sysroot/usr/include/libxml2/

)

# 链接 mongoose 动态库
# target_link_libraries(app PRIVATE mongoose)
target_link_libraries(app PRIVATE xml2)
