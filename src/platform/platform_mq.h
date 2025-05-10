/**
 * @file platform.h
 * @brief 平台设备，包含自定义的消息队列
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
#ifndef __PLATFORM_MQ_H__
#define __PLATFORM_MQ_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <semaphore.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PLATFORM_MQ_WAIT_NO         0
#define PLATFORM_MQ_WAIT_FOREVER    -1

// 消息节点结构
typedef struct MessageNode {
    void *data;                 // 消息数据
    size_t data_size;           // 消息数据大小
    struct MessageNode *next;   // 下一个节点指针
} MessageNode;

// 消息队列结构
typedef struct MessageQueue {
    MessageNode *head;          // 队列头节点
    MessageNode *tail;          // 队列尾节点
    pthread_mutex_t lock;       // 互斥锁
    size_t size;                // 队列中的消息数量
    sem_t sem;                 // 信号量
} MessageQueue;





int mq_enqueue(MessageQueue *queue, const void *data, size_t data_size);
int mq_dequeue(MessageQueue *queue, void **data, size_t *data_size, int timeout_ms);
int mq_dequeue_to_buffer(MessageQueue *queue, void *buffer, size_t *buffer_size, int timeout_ms);
void mq_init(MessageQueue *queue);
void mq_destroy(MessageQueue *queue);
size_t mq_size(MessageQueue *queue);





#ifdef __cplusplus
}
#endif


#endif

