#ifndef GET_STATUSB_REQUEST_H
#define GET_STATUSB_REQUEST_H

#include <pthread.h>
#include <libxml/parser.h>
#include <libxml/tree.h>
#include "TypeDefine.h"
#include "MotorCtrl.h"
#include "main.h"
#include <atomic>


typedef struct
{
    double d_distanceRespone[3]; //编码器值
    double d_speedRespone[3]; //速度
}GetMotorStateType;


//电机状态应答线程
void* GetStatusBRequestFunc(void* arg);

/*获取电机状态*/
void GetMotorStatus(GetMotorStateType* getStaBRes);

/*状态应答*/
void getStatusBRequestResponse(int sockfd, GetMotorStateType* getStaBRes);

extern volatile uint8_t request_received;

extern GetMotorStateType getMotorStateVar;

extern pthread_rwlock_t getMotorStateVar_rwlock;

extern std::atomic<int> InjectionReady; //是否已进样标志
extern std::atomic<int>  CanExecuteSample; //是否可以进行下一个样品
extern std::atomic<int>  TriggerReady; //GC是否已就绪

extern std::atomic<int> IncubationTempStableFlag;
extern std::atomic<int> ConditionTempStableFlag;
extern std::atomic<int> InjectionTempStableFlag;

#endif