# 设置 mongoose 动态库
add_library(mongoose STATIC
    mongoose.c
)

# 设置 mongoose 库的头文件搜索路径
target_include_directories(mongoose PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)


