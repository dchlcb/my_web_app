# 设置 sqlite3 动态库
add_library(sqlite3 SHARED
    sqlite3.c
)

# 设置 mongoose 库的头文件搜索路径
target_include_directories(sqlite3 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)





