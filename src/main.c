// // Copyright (c) 2024 Cesanta Software Limited
// // All rights reserved

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <fcntl.h>
#include <unistd.h>
#include <semaphore.h>
#include <sys/mman.h>

#include "hal.h"
#include "mongoose.h"
#include "net.h"

#define BLINK_PERIOD_MS 1000  // LED blinking period in millis

static void timer_fn(void *arg) {
  gpio_toggle(LED1);  // Blink LED
  (void) arg;         // Unused
}

// In RTOS environment, you can run this function in a separate task
static void run_mongoose(void) {
  struct mg_mgr mgr;        // Mongoose event manager
  mg_mgr_init(&mgr);        // Initialise event manager
  mg_log_set(MG_LL_DEBUG);  // Set log level to debug
  mg_timer_add(&mgr, BLINK_PERIOD_MS, MG_TIMER_REPEAT, timer_fn, NULL);

  net_init(&mgr);          // Initialise application defined in net.c
  for (;;) {               // Infinite event loop
    mg_mgr_poll(&mgr, 0);  // Process network events
  }
}

// int main(void) {
//   hal_init();      // Cross-platform hardware init
//   run_mongoose();  // Initialise and run network application
//   return 0;
// }


// 用于共享内存的名称
#define SHARED_MEMORY_NAME "/SharedMemory"
#define MUTEX_NAME "/Mutex"
#define MESSAGE_SIZE 256

// 定义共享内存结构
typedef struct {
    int data;
    char message[MESSAGE_SIZE];
} SharedData;

// 全局变量
int shm_fd;
sem_t *mutex;
SharedData *shared_data;

// 线程1的函数
void* thread1_func(void* arg) {
    // sem_wait(mutex);  // 加锁
    // shared_data->data = 42;
    // strncpy(shared_data->message, "Hello from thread 1", MESSAGE_SIZE);
    // printf("Thread 1: Data set to %d, message set to \"%s\"\n", shared_data->data, shared_data->message);
    // sem_post(mutex);  // 释放锁

    hal_init();      // Cross-platform hardware init
    run_mongoose();  // Initialise and run network application
    return 0;
}

// 线程2的函数
void* thread2_func(void* arg) {
    // sem_wait(mutex);  // 加锁
    // printf("Thread 2: Data = %d, message = \"%s\"\n", shared_data->data, shared_data->message);
    // sem_post(mutex);  // 释放锁

    for (;;)
    {
        printf("thread2 is running!\n");
        sleep(1);
    }
    
    return 0;
}

void init_shared_memory() {
    // 创建共享内存
    shm_fd = shm_open(SHARED_MEMORY_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        perror("shm_open");
        exit(1);
    }
    ftruncate(shm_fd, sizeof(SharedData));  // 设置共享内存大小

    shared_data = mmap(NULL, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shared_data == MAP_FAILED) {
        perror("mmap");
        close(shm_fd);
        exit(1);
    }

    // 创建信号量 (互斥锁)
    mutex = sem_open(MUTEX_NAME, O_CREAT, 0666, 1);
    if (mutex == SEM_FAILED) {
        perror("sem_open");
        munmap(shared_data, sizeof(SharedData));
        close(shm_fd);
        exit(1);
    }
}

void cleanup() {
    munmap(shared_data, sizeof(SharedData));  // 卸载共享内存
    close(shm_fd);  // 关闭共享内存文件描述符
    sem_close(mutex);  // 关闭信号量
    sem_unlink(MUTEX_NAME);  // 删除信号量
    shm_unlink(SHARED_MEMORY_NAME);  // 删除共享内存
}

int main() {
    // 初始化共享内存和信号量
    init_shared_memory();

    // 创建两个线程
    pthread_t thread1, thread2;
    pthread_create(&thread1, NULL, thread1_func, NULL);
    pthread_create(&thread2, NULL, thread2_func, NULL);

    // 等待线程结束
    pthread_join(thread1, NULL);
    pthread_join(thread2, NULL);

    // 清理资源
    cleanup();

    return 0;
}

