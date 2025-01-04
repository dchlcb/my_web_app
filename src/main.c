/**
 * @file main.c
 * @brief 
 * @author wpc (dchlcb@163.com)
 * @version 1.0
 * @date 2025-01-01
 * 
 * @copyright Copyright (c) 2025, Dchlcb Development Team
 * 
 * @logs:
 * Date           Version     Author      Description
 * 2025-01-01     v1.0        wpc         the first version
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <fcntl.h>
#include <unistd.h>
#include <semaphore.h>

#include "hal.h"
#include "mongoose.h"
#include "net.h"
#include "platform_mq.h"
#include "platform_sys.h"

#define BLINK_PERIOD_MS 1000 // LED blinking period in millis

// 全局变量
sem_t sem_test;
MessageQueue queue;


const char *msg1 = "Hello, World!";
const char *msg2 = "C Programming";
int counter = 0;

static void timer_fn(void *arg)
{
    gpio_toggle(LED1); // Blink LED
    // sem_post(&sem_test);

    // 入队测试
    // mq_enqueue(&queue, msg1, strlen(msg1) + 1);
    // mq_enqueue(&queue, msg2, strlen(msg2) + 1);
    // mq_enqueue(&queue, &counter, sizeof(counter));


    (void)arg;         // Unused
}

// In RTOS environment, you can run this function in a separate task
static void run_mongoose(void)
{
    struct mg_mgr mgr;       // Mongoose event manager
    mg_mgr_init(&mgr);       // Initialise event manager
    mg_log_set(MG_LL_DEBUG); // Set log level to debug
    mg_timer_add(&mgr, BLINK_PERIOD_MS, MG_TIMER_REPEAT, timer_fn, NULL);

    net_init(&mgr); // Initialise application defined in net.c
    for (;;)
    {                         // Infinite event loop
        mg_mgr_poll(&mgr, 0); // Process network events
        delay_ms(1);
    }
}

// 线程1的函数
void *thread1_func(void *arg)
{

    hal_init();     // Cross-platform hardware init
    run_mongoose(); // Initialise and run network application
    return 0;
}

// 线程2的函数
void *thread2_func(void *arg)
{
    
    char *data;
    size_t data_size;
    for (;;)
    {
        sem_wait(&sem_test);    //等待信号量
        printf("thread2 is running!\n");
        // 出队测试
        while (mq_dequeue(&queue, (void **)&data, &data_size) == 0)
        {
            printf("Dequeued message: %s\n", data);
            free(data); // 注意释放消息数据
            printf("Queue size after dequeuing: %zu\n", mq_size(&queue));
        }

    }

    return 0;
}

void semaphore_init()
{
    int ret = 0;

    // 创建信号量
    ret = sem_init(&sem_test, 0, 0); // 第二个参数为0表示线程间共享，1表示进程间共享，第三个参数为信号量的初始值
    if (ret)
    {
        printf("sem_init fail!");
        exit(1);
    }
}



void cleanup()
{
    sem_destroy(&sem_test); // 清理资源，信号量不再使用
}

int main()
{
    // 初始化信号量
    semaphore_init();
    mq_init(&queue);

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
