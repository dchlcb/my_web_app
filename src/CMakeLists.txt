# 指定 CMake 最低版本
cmake_minimum_required(VERSION 3.10)

# 添加 app 和 mongoose 子目录
add_subdirectory(app)
# add_subdirectory(mongoose)

# 指定源文件
add_executable(my_program ./app/main.cpp)
# target_link_libraries(my_program PRIVATE xml2)

# 链接静态库和动态库
target_link_libraries(my_program PRIVATE app)
# target_link_libraries(my_program PRIVATE xml2)
# target_link_libraries(my_program PRIVATE app mongoose)

# 包含头文件目录
target_include_directories(my_program PRIVATE
    ${CMAKE_SOURCE_DIR}/src/app
    # ${CMAKE_SOURCE_DIR}/src/mongoose
)