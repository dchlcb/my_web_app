#ifndef GET_ZBOARD_REQUEST_H
#define GET_ZBOARD_REQUEST_H

#include <pthread.h>
#include <libxml/parser.h>
#include <libxml/tree.h>
#include "TypeDefine.h"
#include "MotorCtrl.h"
#include "main.h"
#include <atomic>       // 原子操作支持

typedef struct
{
    bool HeatAgitator; //加热标志
    float Temperature; //当前温度
    float TarTemp;     //目标温度
}GetZboardRequestType;


//孵化模块与PC通讯应答线程
void* GetZboardRequestFunc(void* arg);

/*状态应答*/
void getZboardRequestResponse(int sockfd, GetZboardRequestType* getStaBRes);

extern GetZboardRequestType getZboardRequestTypeVar;
extern volatile uint8_t zboard_request_received;


#endif