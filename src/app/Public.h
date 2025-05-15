#ifndef PUBLIC_H
#define PUBLIC_H

#include "TypeDefine.h"
#include <vector>
#include <cstring>
#include <iostream>
#include <type_traits>
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <type_traits>

#define BUFFER_SIZE 1*1024
#define FILE_PATH "/coordinates.txt" //坐标存入文件
#define CHANGE_TOOL_PATH "/ChangeTool.txt" //进样工具存入文件

//CAN使用信号量
union semun 
{
    int val;               // 信号量的值
    struct semid_ds *buf;  // 信号量集的缓冲区
    unsigned short *array; // 信号量集的值数组
};


typedef struct sharddata
{
    uint8_t toMCU[1024];       // 发送到 MCU 的数据
    uint8_t fromMCU[1024];     // 从 MCU 接收的数据
    size_t toMCU_size;         // 发送数据大小
    size_t fromMCU_size;       // 接收数据大小
    bool dataToMCUReady;       // 发送数据就绪标志
    bool dataFromMCUReady;     // 接收数据就绪标志
}SharedData;

//写入一个坐标
int write_coordinates(const char *identifier, float x, float y, float z);

//读取一个坐标
//int read_coordinates(const char *identifier, float &out_x, float &out_y, float &out_z);
int read_coordinates(const char *identifier, float *out_x, float *out_y, float *out_z);

//合并两个字符串，并剔除其中的空格
void merge_and_remove_spaces(const char *str1, const char *str2, char **result);

//序列化Vector
bool serialize_vector(const std::vector<int>& vec, char* buffer, size_t buffer_size);

//反序列化vector
bool deserialize_vector(const char* buffer, size_t buffer_size, std::vector<int>& vec);


// 序列化vector
template <typename T>
bool serialize_vector(const std::vector<T>& vec, char* buffer, size_t buffer_size)
{
    // 确保 T 是平凡可复制类型，否则直接使用 memcpy 可能不安全
    static_assert(std::is_trivially_copyable<T>::value, "T must be trivially copyable");

    // 计算序列化所需的总字节数：
    // 一个 size_t 用于存储元素个数，加上所有元素数据所占的字节数
    size_t required_size = sizeof(size_t) + vec.size() * sizeof(T);
    if (buffer_size < required_size)
    {
        std::cerr << "Buffer too small for serialization" << std::endl;
        return false;
    }

    // 将 vector 的元素个数写入缓冲区
    size_t num_elements = vec.size();
    std::memcpy(buffer, &num_elements, sizeof(size_t));
    buffer += sizeof(size_t);

    // 如果 vector 非空，则将所有元素写入缓冲区
    if (!vec.empty())
    {
        std::memcpy(buffer, vec.data(), num_elements * sizeof(T));
    }

    return true;
}

//反序列化vector
template <typename T>
bool deserialize_vector(const char* buffer, size_t buffer_size, std::vector<T>& vec)
{
    // 确保 T 是平凡可复制的类型，否则直接 memcpy 可能不安全
    static_assert(std::is_trivially_copyable<T>::value, "T must be trivially copyable");

    // 检查缓冲区是否至少能存放一个 size_t 大小的元素个数
    if (buffer_size < sizeof(size_t)) 
    {
        std::cerr << "Buffer too small to contain vector size" << std::endl;
        return false;
    }

    // 读取元素个数
    size_t num_elements = 0;
    std::memcpy(&num_elements, buffer, sizeof(size_t));
    buffer += sizeof(size_t);
    buffer_size -= sizeof(size_t);

    // 检查缓冲区是否能容纳所有的元素
    if (buffer_size < num_elements * sizeof(T)) 
    {
        std::cerr << "Buffer too small to contain all vector elements" << std::endl;
        return false;
    }

    // 调整 vector 大小并拷贝数据
    vec.resize(num_elements);
    if (!vec.empty()) 
    {
        std::memcpy(vec.data(), buffer, num_elements * sizeof(T));
    }

    return true;
}

int write_key_value_internal(const char *file_path, const char *key, const std::string &value);

/* 写入：任意可流式化类型 */
template<typename T>
int write_key_value(const char *file_path, const char *key, const T &val)
{
    std::ostringstream oss;
    oss << val;                     /* C++11 支持 */
    return write_key_value_internal(file_path, key, oss.str());
}

/* ---------- 读取 ---------- */

/* 针对 std::string 的重载 */
int read_key_value1(const char *file_path, const char *key, std::string &out_val);

/* 针对数值等其它类型（使用 SFINAE，避免 C++17 if constexpr） */
template<typename T>
typename std::enable_if<!std::is_same<T, std::string>::value, int>::type
read_key_value(const char *file_path, const char *key, T &out_val)
{
    std::string tmp;
    int ret = read_key_value1(file_path, key, tmp);
    if (ret == 0) 
    {
        std::istringstream iss(tmp);
        if (!(iss >> out_val)) return -1;   /* 解析失败 */
    }
    return ret;
}


#endif