# 设置 cJSON 动态库
add_library(cjson SHARED
    cJSON.c
)

# 设置 cJSON 库的头文件搜索路径
target_include_directories(cjson PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)







