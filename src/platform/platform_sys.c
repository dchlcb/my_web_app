/**
 * @file platform_sys.c
 * @brief 
 * @author wpc (dchlcb@163.com)
 * @version 1.0
 * @date 2025-01-04
 * 
 * @copyright Copyright (c) 2025, Dchlcb Development Team
 * 
 * @logs:
 * Date           Version     Author      Description
 * 2025-01-04     v1.0        wpc         the first version
 */

#include "platform_mq.h"

#ifdef _WIN32
#include <windows.h>

// 微秒级延时函数
void delay_us(unsigned int microseconds) {
    LARGE_INTEGER frequency, start, current;
    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&start);

    LONGLONG target = (LONGLONG)microseconds * frequency.QuadPart / 1000000;

    do {
        QueryPerformanceCounter(&current);
    } while (current.QuadPart - start.QuadPart < target);
}

// 毫秒级延时函数
void delay_ms(unsigned int milliseconds) {
    Sleep(milliseconds); // Windows API，单位为毫秒
}

#else // POSIX 系统
#include <time.h>
#include <unistd.h>
#include <errno.h>

// 微秒级延时函数
void delay_us(unsigned int microseconds) {
    struct timespec ts;
    ts.tv_sec = microseconds / 1000000;               // 秒部分
    ts.tv_nsec = (microseconds % 1000000) * 1000;     // 纳秒部分

    while (nanosleep(&ts, &ts) == -1 && errno == EINTR) {
        // 如果被信号中断，继续延时剩余部分
    }
}

// 毫秒级延时函数
void delay_ms(unsigned int milliseconds) {
    struct timespec ts;
    ts.tv_sec = milliseconds / 1000;               // 秒部分
    ts.tv_nsec = (milliseconds % 1000) * 1000000;  // 纳秒部分

    while (nanosleep(&ts, &ts) == -1 && errno == EINTR) {
        // 如果被信号中断，继续延时剩余部分
    }
}

#endif
