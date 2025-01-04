/**
 * @file platform_mq.c
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

// 初始化消息队列
void mq_init(MessageQueue *queue)
{
    if (!queue)
        return;
    queue->head = NULL;
    queue->tail = NULL;
    queue->size = 0;
    pthread_mutex_init(&queue->lock, NULL);
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

    // 销毁互斥锁
    pthread_mutex_destroy(&queue->lock);
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

    return 0;
}

// 出队（消费者）
int mq_dequeue(MessageQueue *queue, void **data, size_t *data_size)
{
    if (!queue || !data || !data_size)
        return -1;

    pthread_mutex_lock(&queue->lock);
    if (queue->head == NULL)
    { // 队列为空
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

    // 出队测试
    char *data;
    size_t data_size;
    while (mq_dequeue(&queue, (void **)&data, &data_size) == 0)
    {
        printf("Dequeued message: %s\n", data);
        free(data); // 注意释放消息数据
    }

    printf("Queue size after dequeuing: %zu\n", mq_size(&queue));

    // 销毁队列
    mq_destroy(&queue);

    return 0;
}
