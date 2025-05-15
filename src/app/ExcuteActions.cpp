#include <iostream>
#include "ExcuteActions.h"
#include "MotorCtrl.h"
#include "Public.h"
#include "main.h"
#include "Calibrations.h"
#include "LiquidSamplingFlow.h"


void* ExcuteActionsFunc(void* arg)
{
    key_t EAKey;
    int32_t stateData;
    int32_t sendData;

    msgType* msgData = new msgType;
    msgData->msgtype = 1;

    EAKey = ftok(".", 'E');

    // 检查并删除旧消息队列
    msgExcuteActionsID = msgget(EAKey, 0);  // 尝试获取现有队列
    if (msgExcuteActionsID != -1) 
    {
        if (msgctl(msgExcuteActionsID, IPC_RMID, NULL) == -1) 
        {
            perror("msgctl IPC_RMID failed");
            delete msgData;
            exit(1);
        }
        printf("成功删除旧消息队列\n");
    } 
    else 
    {
        printf("没有找到旧消息队列\n");
    }

    msgExcuteActionsID = msgget(EAKey, IPC_CREAT | 0666);
    
    if (msgExcuteActionsID == -1) 
    {
        perror("ExcuteActions msgget failed");
        delete msgData;
        pthread_exit(NULL);
    }  

    while(stop_flag)
    {
        if(msgrcv(msgExcuteActionsID, msgData, sizeof(msgData->msgdata), 0, 0) == -1)
        {
            perror("ExcuteActions msgrcv failed");
            break;  // 其他错误，退出循环
        }
        else 
        {
            ExcuteActionsType tempData;
            //反序列化
            deserialize(msgData->msgdata, tempData, sizeof(msgData->msgdata));
            
            if(strcmp(tempData.name, "ChangeToolEnd") == 0) //换针结束
            {
                ReadCoordinatesMotorExec(HOME_NAME);
                //OMotorExec(-30);
                //O轴回零
                MotorRetZero(en_O, 500, 500, 100, 100);
                stateData = 0;
                while(!(((stateData>>12)&1) && ((stateData>>10)&1)))
                {
                    SDORead_ShellFunc(can_sfd,en_O,0x6041,0x00,reinterpret_cast<uint8_t*>(&stateData));
                    usleep(100*1000);
                }
                //编码器清零
                sendData = 35;
                SDOWrite_ShellFunc(can_sfd, en_O, 0x6098, 0x00, reinterpret_cast<uint8_t*>(&sendData), 1);
                usleep(10*1000);
                sendData = 0x0f;
                SDOWrite_ShellFunc(can_sfd, en_O, 0x6040, 0x00, reinterpret_cast<uint8_t*>(&sendData), 2);
                usleep(10*1000);
                sendData = 0x1f;
                SDOWrite_ShellFunc(can_sfd, en_O, 0x6040, 0x00, reinterpret_cast<uint8_t*>(&sendData), 2);
                usleep(10*1000);
                sendData = 0x0f;
                SDOWrite_ShellFunc(can_sfd, en_O, 0x6040, 0x00, reinterpret_cast<uint8_t*>(&sendData), 2);
                usleep(10*1000);
                oposRecord = 0;
                //切换轮廓位置模式
                MotorPositionModle(en_O);

                MotorRelPosCtrl(true, en_MOVE, en_O, 5.4, 50);
			
                stateData = 0;
                while(!((stateData>>10)&0x01))
                {
                    
                    SDORead_ShellFunc(can_sfd, en_O, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&stateData));
                    
                    usleep(100*1000);
                }

                MotorRelPosCtrl(true, en_MOVE, en_O, -5.0, 50);
			
                stateData = 0;
                while(!((stateData>>10)&0x01))
                {
                    
                    SDORead_ShellFunc(can_sfd, en_O, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&stateData));
                    
                    usleep(100*1000);
                }
                printf("ChangeToolEnd\r\n");
            }
            else if(strcmp(tempData.name, "ChangeTool") == 0) //换针
            {
                OMotorExec(30);
                ReadCoordinatesMotorExec(CHANGETOOL_NAME);
                printf("ChangeTool\r\n");
            }
            else if(strcmp(tempData.name, "Shutdown") == 0) //关机
            {
                ReadCoordinatesMotorExec(SHUTDOWN_NAME);
                printf("Shutdown\r\n");
            }
            else if(strcmp(tempData.name, "Home") == 0) //home
            {
                ReadCoordinatesMotorExec(HOME_NAME);
                printf("Home\r\n");
            }
            else if(strcmp(tempData.name, "Standby") == 0) //待机
            {
                ReadCoordinatesMotorExec(HOME_NAME);
                printf("Standby\r\n");
            }
            else
            {
                MotorRelPosCtrl(true,tempData.en_motion, tempData.en_axis, tempData.d_distance, tempData.d_speed);
            }
            
        }
        
    }

    //释放资源
    delete msgData;
    printf("电机控制线程退出\r\n");
    pthread_exit(NULL);
}