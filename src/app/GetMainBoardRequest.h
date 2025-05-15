#ifndef GET_MAINBOARD_REQUEST_H
#define GET_MAINBOARD_REQUEST_H

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
    uint8_t valCtrlFlg; //阀控标志
}GetMainBoardRequestType;


//孵化模块与PC通讯应答线程
void* GetMainBoardRequestFunc(void* arg);


/*状态应答*/
void getAgligRequestResponse(int sockfd, GetMainBoardRequestType* getStaBRes);

extern GetMainBoardRequestType getMainBoardRequestTypeVar;
extern volatile uint8_t mainboard_request_received;


#endif