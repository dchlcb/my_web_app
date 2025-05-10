/**
 * @file platform_mq.c
 * @brief
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

#include <time.h>

// 初始化消息队列
void mq_init(MessageQueue *queue)
{
    if (!queue)
        return;
    queue->head = NULL;
    queue->tail = NULL;
    queue->size = 0;
    pthread_mutex_init(&queue->lock, NULL);
    sem_init(&queue->sem, 0, 0); // 初始化信号量，初始值为0
}

// 销毁消息队列
void mq_destroy(MessageQueue *queue)
{
    if (!queue)
        return;

    // 销毁链表节点
    pthread_mutex_lock(&queue->lock);
    MessageNode *current = queue->head;
    while (current)
    {
        MessageNode *next = current->next;
        free(current->data);
        free(current);
        current = next;
    }
    queue->head = NULL;
    queue->tail = NULL;
    queue->size = 0;
    pthread_mutex_unlock(&queue->lock);

    // 销毁互斥锁和信号量
    pthread_mutex_destroy(&queue->lock);
    sem_destroy(&queue->sem);
}

// 入队（生产者）
int mq_enqueue(MessageQueue *queue, const void *data, size_t data_size)
{
    if (!queue || !data || data_size == 0)
        return -1;

    // 分配新节点
    MessageNode *node = (MessageNode *)malloc(sizeof(MessageNode));
    if (!node)
        return -1;
    node->data = malloc(data_size);
    if (!node->data)
    {
        free(node);
        return -1;
    }
    memcpy(node->data, data, data_size);
    node->data_size = data_size;
    node->next = NULL;

    // 加入队列
    pthread_mutex_lock(&queue->lock);
    if (queue->tail)
    {
        queue->tail->next = node;
    }
    else
    {
        queue->head = node;
    }
    queue->tail = node;
    queue->size++;
    pthread_mutex_unlock(&queue->lock);

    // 增加信号量
    sem_post(&queue->sem);

    return 0;
}

// 出队（消费者，阻塞等待）
int mq_dequeue(MessageQueue *queue, void **data, size_t *data_size, int timeout_ms)
{
    if (!queue || !data || !data_size)
        return -1;

    if (timeout_ms == PLATFORM_MQ_WAIT_FOREVER)
    {
        sem_wait(&queue->sem);
    }
    // else if (timeout_ms == PLATFORM_MQ_WAIT_NO)
    // {
        
    // }
    else
    {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += timeout_ms / 1000;
        ts.tv_nsec += (timeout_ms % 1000) * 1000000;
        if (ts.tv_nsec >= 1000000000)
        {
            ts.tv_sec += ts.tv_nsec / 1000000000;
            ts.tv_nsec %= 1000000000;
        }

        // 等待信号量
        if (sem_timedwait(&queue->sem, &ts) != 0)
            return -1; // 超时或错误
    }
    
    
    pthread_mutex_lock(&queue->lock);
    if (queue->head == NULL)
    {
        pthread_mutex_unlock(&queue->lock);
        return -1;
    }

    // 获取头节点数据
    MessageNode *node = queue->head;
    *data = node->data;
    *data_size = node->data_size;

    // 移动头指针
    queue->head = node->next;
    if (queue->head == NULL)
    {
        queue->tail = NULL; // 如果队列空了，更新尾指针
    }
    queue->size--;
    pthread_mutex_unlock(&queue->lock);

    // 释放节点
    free(node);

    return 0;
}

// 出队到固定缓冲区
int mq_dequeue_to_buffer(MessageQueue *queue, void *buffer, size_t *buffer_size, int timeout_ms)
{
    if (!queue || !buffer || !buffer_size)
        return -1;

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += timeout_ms / 1000;
    ts.tv_nsec += (timeout_ms % 1000) * 1000000;
    if (ts.tv_nsec >= 1000000000)
    {
        ts.tv_sec += ts.tv_nsec / 1000000000;
        ts.tv_nsec %= 1000000000;
    }

    // 等待信号量
    if (sem_timedwait(&queue->sem, &ts) != 0)
        return -1; // 超时或错误

    pthread_mutex_lock(&queue->lock);
    if (queue->head == NULL)
    { // 队列为空
        pthread_mutex_unlock(&queue->lock);
        return -1;
    }

    MessageNode *node = queue->head;
    if (*buffer_size < node->data_size) // 检查缓冲区大小
    {
        pthread_mutex_unlock(&queue->lock);
        return -1;
    }

    memcpy(buffer, node->data, node->data_size);
    *buffer_size = node->data_size;

    // 移动头指针
    queue->head = node->next;
    if (queue->head == NULL)
    {
        queue->tail = NULL; // 如果队列空了，更新尾指针
    }
    queue->size--;
    pthread_mutex_unlock(&queue->lock);

    // 释放节点
    free(node->data);
    free(node);

    return 0;
}

// 获取队列大小
size_t mq_size(MessageQueue *queue)
{
    if (!queue)
        return 0;

    pthread_mutex_lock(&queue->lock);
    size_t size = queue->size;
    pthread_mutex_unlock(&queue->lock);

    return size;
}

// 测试示例
static int main_test()
{
    MessageQueue queue;
    mq_init(&queue);

    // 入队测试
    const char *msg1 = "Hello, World!";
    const char *msg2 = "C Programming";
    mq_enqueue(&queue, msg1, strlen(msg1) + 1);
    mq_enqueue(&queue, msg2, strlen(msg2) + 1);

    printf("Queue size after enqueuing: %zu\n", mq_size(&queue));

    // 出队测试（阻塞等待）
    char *data;
    size_t data_size;
    if (mq_dequeue(&queue, (void **)&data, &data_size, 5000) == 0)
    {
        printf("Dequeued message: %s\n", data);
        free(data); // 释放消息数据
    }
    else
    {
        printf("Dequeue timed out or failed.\n");
    }

    printf("Queue size after dequeuing: %zu\n", mq_size(&queue));

    // 销毁队列
    mq_destroy(&queue);

    return 0;
}