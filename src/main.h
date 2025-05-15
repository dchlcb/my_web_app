#ifndef _MAIN_H
#define _MAIN_H

#include <iostream>
#include <fcntl.h> // 包含fcntl函数
#include <pthread.h>
#include <string>
#include "Public.h"
#include <semaphore.h>
#include <sys/mman.h>
#include "AlarmManager.h"

extern volatile uint8_t stop_flag;

extern volatile uint32_t socketCount;
extern int clientSocket;
extern pthread_mutex_t mutexClientSocket; //客户端套接字互斥锁

//电机共享消息队列声明
extern int msgExcuteActionsID;

//校准共享消息队列声明
extern int msgCalibrationsID;

//内部协议处理信号量声明
extern sem_t* IP_sem;

//主板MCU共享内存资源声明
extern int MB_shmfd; // 初始化共享内存
extern SharedData* MB_shared;
extern sem_t* MB_sem;// 初始化信号量用于同步

//孵化MCU共享内存资源声明
extern int IB_shmfd; // 初始化共享内存
extern SharedData* IB_shared;
extern sem_t* IB_sem;// 初始化信号量用于同步

//Z轴MCU共享内存资源声明
extern int ZB_shmfd;
extern SharedData* ZB_shared;
// 初始化信号量用于同步
extern sem_t* ZB_sem;

//CAN信号量声明
extern sem_t* CANSemID;
extern int can_sfd;

//电机状态变量互斥锁条件变量声明
extern pthread_mutex_t mutexGetStaBReq;
extern pthread_mutex_t mutexStateReqVar;

//调试串口互斥锁
extern pthread_mutex_t serial_mutex;

//孵化状态变量互斥锁条件变量声明
extern pthread_mutex_t mutexGetIncubationReq;
extern pthread_rwlock_t getIncubationRequestTypeVar_rwlock;

//Z轴状态变量互斥锁条件变量声明
extern pthread_mutex_t mutexGetZboardReq;
extern pthread_rwlock_t getZboardRequestTypeVar_rwlock;

//老化状态变量互斥锁条件变量声明
extern pthread_mutex_t mutexGetMainBoardReq;
extern pthread_rwlock_t getMainBoardRequestTypeVar_rwlock;

extern AlarmManager alarmCodeVar;

#endif