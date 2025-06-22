# # 搜索所有源文件
# file(GLOB APP_SOURCES "*.cpp")
# file(GLOB APP_HEADERS "*.h")

# # 设置 app 静态库
# add_library(app STATIC ${APP_SOURCES})

# # 设置 app 库的头文件搜索路径
# target_include_directories(app PUBLIC
#     ${CMAKE_CURRENT_SOURCE_DIR} 
#     /opt/atk-dlrk3568-5_10_sdk-toolchain/aarch64-buildroot-linux-gnu/sysroot/usr/include/libxml2/

# )

# # 链接 mongoose 动态库
# # target_link_libraries(app PRIVATE mongoose)
# target_link_libraries(app PRIVATE xml2)



# 显式列出源文件（避免 file(GLOB)）
set(APP_SOURCES
    # 手动添加所有 .cpp 文件
    main.cpp 
    CANOpen.cpp 
    soapProcess.cpp 
    ExcuteActions.cpp 
    GetStatusBRequest.cpp 
    MotorCtrl.cpp 
    Calibrations.cpp 
    Public.cpp 
    LiquidSamplingFlow.cpp 
    MainBoardMCU.cpp 
    InternalProtocol.cpp 
    IncubationModule.cpp 
    GetIncubationRequest.cpp 
    ZBoardMCU.cpp 
    GetZboardRequest.cpp 
    AlarmManager.cpp 
    GetMainBoardRequest.cpp
)


# 静态库目标
add_library(app STATIC ${APP_SOURCES})

# 头文件搜索路径（使用绝对路径）
target_include_directories(app PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    # 使用变量或显式路径
    /opt/atk-dlrk3568-5_10_sdk-toolchain/aarch64-buildroot-linux-gnu/sysroot/usr/include/libxml2/
)

# 链接库（明确指定全路径）
find_library(LIBXML2_LIB xml2 HINTS /opt/atk-dlrk3568-5_10_sdk-toolchain/aarch64-buildroot-linux-gnu/sysroot/usr/lib)
target_link_libraries(app PRIVATE ${LIBXML2_LIB})





