#ifndef GET_INCUBATION_REQUEST_H
#define GET_INCUBATION_REQUEST_H

#include <pthread.h>
#include <libxml/parser.h>
#include <libxml/tree.h>
#include "TypeDefine.h"
#include "MotorCtrl.h"
#include "main.h"
#include <atomic>       // 原子操作支持

typedef struct
{
    uint8_t HeatAgitator; //加热标志
    float Temperature; //当前温度
    float TarTemp;     //目标温度
    int32_t Speed;     //速度
    float agitatorOnTime; //电机开启时间
    float agitatorOffTime; //电机关闭时间
    uint8_t incubationCompletion; //孵化完成标志
}GetIncubationRequestType;


//孵化模块与PC通讯应答线程
void* GetIncubationRequestFunc(void* arg);

/*状态应答*/
void getIncubationRequestResponse(int sockfd, GetIncubationRequestType* getStaBRes);

extern GetIncubationRequestType getIncubationRequestTypeVar;
extern volatile uint8_t incubation_request_received;

extern std::atomic<int> Pos1Reserved; //孵化位置1
extern std::atomic<int> Pos2Reserved; //孵化位置2
extern std::atomic<int> Pos3Reserved; //孵化位置3
extern std::atomic<int> Pos4Reserved; //孵化位置4
extern std::atomic<int> Pos5Reserved; //孵化位置5
extern std::atomic<int> Pos6Reserved; //孵化位置6

#endif