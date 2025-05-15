#include "LiquidSamplingFlow.h"
#include <pthread.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <cstring>
#include <algorithm>
#include <iomanip>  // 用于 std::setw 和 std::setfill
#include <ctime>
#include <math.h>
#include <sys/time.h>  // 用于 gettimeofday
#include "CANOpen.h"
#include "Public.h"
#include "MotorCtrl.h"
#include "Calibrations.h"
#include "InternalProtocol.h"
#include "MainBoardMCU.h"
#include "GetStatusBRequest.h"
#include "IncubationModule.h"
#include "GetIncubationRequest.h"
#include "main.h"
#include "Public.h"
#include "GetZboardRequest.h"
#include "GetMainBoardRequest.h"

//孵化完成标志位
std::atomic<int> incubationComp(0);


//孵化器孔数变量
AgitatorCntType AgitatorCntVar;


/**
 * @brief 通过非阻塞方式接收消息，带超时处理
 * @param msqid 消息队列ID
 * @param msg 指向消息结构体的指针
 * @param msgSize 消息数据部分的大小（不包括 mtype）
 * @param timeout_sec 超时时间（单位：秒）
 * @return 成功返回接收到的字节数，超时或出错返回 -1
 */
ssize_t msgrcv_with_timeout(int msqid, void* msg, size_t msgSize, long timeout_sec)
{
    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);
    
    ssize_t ret;
    while (true) 
    {
        ret = msgrcv(msqid, msg, msgSize, 0, IPC_NOWAIT);
        if (ret >= 0) 
        {
            // 成功接收到消息
            return ret;
        }
        // 出现错误且不是没有消息
        if (errno != ENOMSG) 
        {
            perror("msgrcv error");
            return -1;
        }
        // 没有消息，检查是否超时
        clock_gettime(CLOCK_MONOTONIC, &now);
        long elapsed = now.tv_sec - start.tv_sec;
        if (elapsed >= timeout_sec) 
        {
            // 超时退出
            errno = ETIMEDOUT;
            return -1;
        }
        // 稍作等待后重试，比如等待100毫秒
        usleep(100 * 1000); // 100毫秒
    }
}


//GC控制通讯
void GCCtrl(GCCtrlType gcCtrl)
{
    UARTHandler tempMCUdata;
    std::vector<uint8_t> mcuData;
    std::vector<uint8_t> mcuAddr = {0x01}; //主板MCU地址
    mcuData.push_back(0x01);
    
    if(gcCtrl == GC_PREPARE) //准备
    {
        mcuData = tempMCUdata.AssembleFrame(1,mcuAddr,1,0xFE,0xA0,CMD_EXT_WRITE,mcuData);
    }
    else if(gcCtrl == GC_START) //启动
    {
        mcuData = tempMCUdata.AssembleFrame(1,mcuAddr,1,0xFE,0xA1,CMD_EXT_WRITE,mcuData);
    }

    //消息发送到主板MCU模块
    sem_wait(MB_sem); // 加锁
    memcpy(MB_shared->toMCU, mcuData.data(), mcuData.size());
    MB_shared->toMCU_size = mcuData.size();
    MB_shared->dataToMCUReady = true;
    sem_post(MB_sem); // 解锁
}


//顶空阀控
void ValCtrl(uint8_t time)
{
    UARTHandler tempMCUdata;
    std::vector<uint8_t> mcuData;
    std::vector<uint8_t> mcuAddr = {0x01}; //MCU地址
    

    mcuData.push_back(time);
    

    mcuData = tempMCUdata.AssembleFrame(1,mcuAddr,1,0xFE,0xA3,CMD_EXT_WRITE,mcuData);
    
    //消息发送到主板MCU模块
    sem_wait(MB_sem); // 加锁
    memcpy(MB_shared->toMCU, mcuData.data(), mcuData.size());
    MB_shared->toMCU_size = mcuData.size();
    MB_shared->dataToMCUReady = true;
    sem_post(MB_sem); // 解锁
}

//孵化模块加热
void IncubationHeat(uint8_t heatAgitator, float incubationTemperature)
{
    UARTHandler tempMCUdata;
    std::vector<uint8_t> mcuData;
    std::vector<uint8_t> mcuAddr = {0x02}; //孵化模块MCU地址

    //孵化器温控标志
    mcuData.push_back(heatAgitator);

    //孵化器温控温度
    uint8_t* floatBytes = reinterpret_cast<uint8_t*>(&incubationTemperature);
    for (size_t i = 0; i < sizeof(float); ++i) 
    {
        mcuData.push_back(floatBytes[i]);
    }

    mcuData = tempMCUdata.AssembleFrame(1,mcuAddr,1,0xFE,0x11,CMD_EXT_WRITE,mcuData);

    //消息发送到孵化模块线程
    sem_wait(IB_sem); // 加锁
    memcpy(IB_shared->toMCU, mcuData.data(), mcuData.size());
    IB_shared->toMCU_size = mcuData.size();
    IB_shared->dataToMCUReady = true;
    sem_post(IB_sem); // 解锁
}

//孵化模块心跳包
void GetIncubationHeart(void)
{
    UARTHandler tempMCUdata;
    std::vector<uint8_t> mcuData;
    std::vector<uint8_t> mcuAddr = {0x02}; //孵化模块MCU地址

    
    mcuData.clear();

    mcuData = tempMCUdata.AssembleFrame(1,mcuAddr,1,0xFE,0xfe,CMD_EXT_READ,mcuData);

    //消息发送到孵化模块线程
    sem_wait(IB_sem); // 加锁
    memcpy(IB_shared->toMCU, mcuData.data(), mcuData.size());
    IB_shared->toMCU_size = mcuData.size();
    IB_shared->dataToMCUReady = true;
    sem_post(IB_sem); // 解锁
}

//启动孵化
void IncubationMotorStart(float incubationTime, int32_t agitatorSpeed, float agitatorOnTime, float agitatorOffTime)
{
    UARTHandler tempMCUdata;
    std::vector<uint8_t> mcuData;
    std::vector<uint8_t> mcuAddr = {0x02}; //孵化模块MCU地址

    //孵化器孵化时间
    uint8_t* floatBytes = reinterpret_cast<uint8_t*>(&incubationTime);
    for (size_t i = 0; i < sizeof(float); ++i) 
    {
        mcuData.push_back(floatBytes[i]);
    }

    //孵化器振摇速度
    floatBytes = reinterpret_cast<uint8_t*>(&agitatorSpeed);
    for (size_t i = 0; i < sizeof(int32_t); ++i) 
    {
        mcuData.push_back(floatBytes[i]);
    }

    //孵化器振摇开启持续时间
    floatBytes = reinterpret_cast<uint8_t*>(&agitatorOnTime);
    for (size_t i = 0; i < sizeof(float); ++i) 
    {
        mcuData.push_back(floatBytes[i]);
    }

    //孵化器振摇关闭持续时间
    floatBytes = reinterpret_cast<uint8_t*>(&agitatorOffTime);
    for (size_t i = 0; i < sizeof(float); ++i) 
    {
        mcuData.push_back(floatBytes[i]);
    }

    mcuData = tempMCUdata.AssembleFrame(1,mcuAddr,1,0xFE,0x12,CMD_EXT_WRITE,mcuData);

    //消息发送到孵化模块线程
    sem_wait(IB_sem); // 加锁
    memcpy(IB_shared->toMCU, mcuData.data(), mcuData.size());
    IB_shared->toMCU_size = mcuData.size();
    IB_shared->dataToMCUReady = true;
    sem_post(IB_sem); // 解锁
}


//孵化复位
void IncubationMotorReset(uint8_t enable, int32_t offset)
{
    UARTHandler tempMCUdata;
    std::vector<uint8_t> mcuData;
    std::vector<uint8_t> mcuAddr = {0x02}; //孵化模块MCU地址

    //孵化器复位标志
    mcuData.push_back(enable);
    
    //孵化器偏移位置
    uint8_t *intBytes = reinterpret_cast<uint8_t*>(&offset);
    for (size_t i = 0; i < sizeof(int32_t); ++i) 
    {
        mcuData.push_back(intBytes[i]);
    }

    mcuData = tempMCUdata.AssembleFrame(1,mcuAddr,1,0xFE,0x13,CMD_EXT_WRITE,mcuData);

    //消息发送到孵化模块线程
    sem_wait(IB_sem); // 加锁
    memcpy(IB_shared->toMCU, mcuData.data(), mcuData.size());
    IB_shared->toMCU_size = mcuData.size();
    IB_shared->dataToMCUReady = true;
    sem_post(IB_sem); // 解锁
}


//孵化完成标志读取
void GetIncubationCompCMD(void)
{
    UARTHandler tempMCUdata;
    std::vector<uint8_t> mcuData;
    std::vector<uint8_t> mcuAddr = {0x02}; //孵化模块MCU地址

    
    mcuData.clear();

    mcuData = tempMCUdata.AssembleFrame(1,mcuAddr,1,0xFE,0x15,CMD_EXT_READ,mcuData);

    //消息发送到孵化模块线程
    sem_wait(IB_sem); // 加锁
    memcpy(IB_shared->toMCU, mcuData.data(), mcuData.size());
    IB_shared->toMCU_size = mcuData.size();
    IB_shared->dataToMCUReady = true;
    sem_post(IB_sem); // 解锁
}


//HS进样针加热
void HSNeedleHeating(uint8_t heatSyringe, float syringeTemperature)
{
    UARTHandler tempMCUdata;
    std::vector<uint8_t> mcuData;
    std::vector<uint8_t> mcuAddr = {0x03}; //Z轴板模块MCU地址

    mcuData.push_back(heatSyringe);
    
    uint8_t* floatBytes = reinterpret_cast<uint8_t*>(&syringeTemperature);

    // 将浮点数的字节存入 mcuData
    for (size_t i = 0; i < sizeof(float); ++i) 
    {
        mcuData.push_back(floatBytes[i]);
    }

    mcuData = tempMCUdata.AssembleFrame(1,mcuAddr,1,0xFE,0xC0,CMD_EXT_WRITE,mcuData);
    
    //消息发送到孵化模块线程
    sem_wait(ZB_sem); // 加锁
    memcpy(ZB_shared->toMCU, mcuData.data(), mcuData.size());
    ZB_shared->toMCU_size = mcuData.size();
    ZB_shared->dataToMCUReady = true;
    sem_post(ZB_sem); // 解锁
}


//GC状态读取
bool GCStateRead(void)
{
    bool ret = false;

    UARTHandler tempMCUdata;
    std::vector<uint8_t> mcuData;
    std::vector<uint8_t> mcuAddr = {0x01}; //主板MCU地址

    mcuData = tempMCUdata.AssembleFrame(1,mcuAddr,1,0xFE,0xA2,CMD_EXT_READ,mcuData);
    
    //消息发送到主板MCU模块
    sem_wait(MB_sem); // 加锁
    memcpy(MB_shared->toMCU, mcuData.data(), mcuData.size());
    MB_shared->toMCU_size = mcuData.size();
    MB_shared->dataToMCUReady = true;
    sem_post(MB_sem); // 解锁

    ret = true;

    return ret;
}

//Z轴模块心跳包
void GetZboardHeart(void)
{
    UARTHandler tempMCUdata;
    std::vector<uint8_t> mcuData;
    std::vector<uint8_t> mcuAddr = {0x03}; //Z轴模块MCU地址

    mcuData.clear();

    mcuData = tempMCUdata.AssembleFrame(1,mcuAddr,1,0xFE,0xfd,CMD_EXT_READ,mcuData);

    //消息发送到孵化模块线程
    sem_wait(ZB_sem); // 加锁
    memcpy(ZB_shared->toMCU, mcuData.data(), mcuData.size());
    ZB_shared->toMCU_size = mcuData.size();
    ZB_shared->dataToMCUReady = true;
    sem_post(ZB_sem); // 解锁
}

//老化模块心跳包
void GetAgingHeart(void)
{
    UARTHandler tempMCUdata;
    std::vector<uint8_t> mcuData;
    std::vector<uint8_t> mcuAddr = {0x04}; //老化模块MCU地址

    mcuData.clear();

    mcuData = tempMCUdata.AssembleFrame(1,mcuAddr,1,0xFE,0xfc,CMD_EXT_READ,mcuData);

    //消息发送到主板MCU模块
    sem_wait(MB_sem); // 加锁
    memcpy(MB_shared->toMCU, mcuData.data(), mcuData.size());
    MB_shared->toMCU_size = mcuData.size();
    MB_shared->dataToMCUReady = true;
    sem_post(MB_sem); // 解锁
}


//老化模块加热
void AgingHeating(uint8_t heatSyringe, float syringeTemperature)
{
    UARTHandler tempMCUdata;
    std::vector<uint8_t> mcuData;
    std::vector<uint8_t> mcuAddr = {0x04}; //Z轴板模块MCU地址

    mcuData.push_back(heatSyringe);
    
    uint8_t* floatBytes = reinterpret_cast<uint8_t*>(&syringeTemperature);

    // 将浮点数的字节存入 mcuData
    for (size_t i = 0; i < sizeof(float); ++i) 
    {
        mcuData.push_back(floatBytes[i]);
    }

    mcuData = tempMCUdata.AssembleFrame(1,mcuAddr,1,0xFE,0x40,CMD_EXT_WRITE,mcuData);
    
    //消息发送到主板MCU模块
    sem_wait(MB_sem); // 加锁
    memcpy(MB_shared->toMCU, mcuData.data(), mcuData.size());
    MB_shared->toMCU_size = mcuData.size();
    MB_shared->dataToMCUReady = true;
    sem_post(MB_sem); // 解锁
}

//老化阀控
void AgingValCtrl(int8_t time)
{
    UARTHandler tempMCUdata;
    std::vector<uint8_t> mcuData;
    std::vector<uint8_t> mcuAddr = {0x04}; //MCU地址
    

    mcuData.push_back(time);
    

    mcuData = tempMCUdata.AssembleFrame(1,mcuAddr,1,0xFE,0x42,CMD_EXT_WRITE,mcuData);
    
    //消息发送到主板MCU模块
    sem_wait(MB_sem); // 加锁
    memcpy(MB_shared->toMCU, mcuData.data(), mcuData.size());
    MB_shared->toMCU_size = mcuData.size();
    MB_shared->dataToMCUReady = true;
    sem_post(MB_sem); // 解锁
}


//老化温控PID
void AgingPIDCtrl(float P, float I, float D)
{
    UARTHandler tempMCUdata;
    std::vector<uint8_t> mcuData;
    std::vector<uint8_t> mcuAddr = {0x04}; //MCU地址
    
    uint8_t* floatBytes = reinterpret_cast<uint8_t*>(&P);

    // 将浮点数的字节存入 mcuData
    for (size_t i = 0; i < sizeof(float); ++i) 
    {
        mcuData.push_back(floatBytes[i]);
    }

    floatBytes = reinterpret_cast<uint8_t*>(&I);
    // 将浮点数的字节存入 mcuData
    for (size_t i = 0; i < sizeof(float); ++i) 
    {
        mcuData.push_back(floatBytes[i]);
    }

    floatBytes = reinterpret_cast<uint8_t*>(&D);
    // 将浮点数的字节存入 mcuData
    for (size_t i = 0; i < sizeof(float); ++i) 
    {
        mcuData.push_back(floatBytes[i]);
    }
    
    mcuData = tempMCUdata.AssembleFrame(1,mcuAddr,1,0xFE,0x41,CMD_EXT_WRITE,mcuData);
    
    //消息发送到主板MCU模块
    sem_wait(MB_sem); // 加锁
    memcpy(MB_shared->toMCU, mcuData.data(), mcuData.size());
    MB_shared->toMCU_size = mcuData.size();
    MB_shared->dataToMCUReady = true;
    sem_post(MB_sem); // 解锁
}


//孵化模块温控PID
void IncubationPIDCtrl(float P, float I, float D)
{
    UARTHandler tempMCUdata;
    std::vector<uint8_t> mcuData;
    std::vector<uint8_t> mcuAddr = {0x02}; //MCU地址
    
    uint8_t* floatBytes = reinterpret_cast<uint8_t*>(&P);
    // 将浮点数的字节存入 mcuData
    for (size_t i = 0; i < sizeof(float); ++i) 
    {
        mcuData.push_back(floatBytes[i]);
    }

    floatBytes = reinterpret_cast<uint8_t*>(&I);
    // 将浮点数的字节存入 mcuData
    for (size_t i = 0; i < sizeof(float); ++i) 
    {
        mcuData.push_back(floatBytes[i]);
    }

    floatBytes = reinterpret_cast<uint8_t*>(&D);
    // 将浮点数的字节存入 mcuData
    for (size_t i = 0; i < sizeof(float); ++i) 
    {
        mcuData.push_back(floatBytes[i]);
    }
    
    mcuData = tempMCUdata.AssembleFrame(1,mcuAddr,1,0xFE,0x14,CMD_EXT_WRITE,mcuData);
    
    //消息发送到主板MCU模块
    sem_wait(IB_sem); // 加锁
    memcpy(IB_shared->toMCU, mcuData.data(), mcuData.size());
    IB_shared->toMCU_size = mcuData.size();
    IB_shared->dataToMCUReady = true;
    sem_post(IB_sem); // 解锁
}


//进样工具温控PID
void InjectionPIDCtrl(float P, float I, float D)
{
    UARTHandler tempMCUdata;
    std::vector<uint8_t> mcuData;
    std::vector<uint8_t> mcuAddr = {0x03}; //MCU地址
    
    uint8_t* floatBytes = reinterpret_cast<uint8_t*>(&P);
    // 将浮点数的字节存入 mcuData
    for (size_t i = 0; i < sizeof(float); ++i) 
    {
        mcuData.push_back(floatBytes[i]);
    }

    floatBytes = reinterpret_cast<uint8_t*>(&I);
    // 将浮点数的字节存入 mcuData
    for (size_t i = 0; i < sizeof(float); ++i) 
    {
        mcuData.push_back(floatBytes[i]);
    }

    floatBytes = reinterpret_cast<uint8_t*>(&D);
    // 将浮点数的字节存入 mcuData
    for (size_t i = 0; i < sizeof(float); ++i) 
    {
        mcuData.push_back(floatBytes[i]);
    }
    
    mcuData = tempMCUdata.AssembleFrame(1,mcuAddr,1,0xFE,0xc1,CMD_EXT_WRITE,mcuData);
    
    //消息发送到主板MCU模块
    sem_wait(ZB_sem); // 加锁
    memcpy(ZB_shared->toMCU, mcuData.data(), mcuData.size());
    ZB_shared->toMCU_size = mcuData.size();
    ZB_shared->dataToMCUReady = true;
    sem_post(ZB_sem); // 解锁
}


//流程线程
void* LiquidSamplingFlowFunc(void* arg)
{
    key_t Key;

    msgType* msgData = new msgType;
    msgData->msgtype = 1;

    Key = ftok(".", 'L');

    // 检查并删除旧消息队列
    msgLiquidSamplingFlowID = msgget(Key, 0);  // 尝试获取现有队列
    if (msgLiquidSamplingFlowID != -1) 
    {
        if (msgctl(msgLiquidSamplingFlowID, IPC_RMID, NULL) == -1) 
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

    msgLiquidSamplingFlowID = msgget(Key, IPC_CREAT | IPC_EXCL | 0666);

    LiquidSamplingFlowType tempData;

    std::memset(&AgitatorCntVar, 0, sizeof(AgitatorCntVar));
    
    if (msgLiquidSamplingFlowID == -1) 
    {
        perror("LiquidSamplingFlow msgget failed");
        delete msgData;
        pthread_exit(NULL);
    }

    while(stop_flag)
    {
        if(msgrcv(msgLiquidSamplingFlowID, msgData, sizeof(msgData->msgdata), 0, IPC_NOWAIT) == -1)
        {
            if (errno == ENOMSG)
            {
                // 没有消息，短暂休眠后继续检查 stop_flag
                usleep(1000);  // 休眠 1ms，避免 CPU 占用过高
                continue;
            }
            else
            {
                perror("LiquidSamplingFlow msgrcv failed");
                break;  // 其他错误，退出循环
            }
            
        }
        else
        {
            //反序列化
            deserialize(msgData->msgdata, tempData, sizeof(msgData->msgdata));

            //液体进样流程
            if(strcmp(tempData.methodName, "LIQ_STD") == 0)
            {
                //打印测试
                printf("tempData.methodName = %s\n", tempData.methodName);

                //就绪状态才能进行
                //if(GCStateRead()) 
                {
                    //GC进入预运行状态
                    GCCtrl(GC_PREPARE);

                    //液体进样流程执行
                    LiquidProcessExecution(&tempData);
                }

            }//顶空进样流程
            else if(strcmp(tempData.methodName, "HS_STD") == 0)
            {
                 //打印测试
                 printf("tempData.methodName = %s\n", tempData.methodName);

                //GC进入预运行状态
                GCCtrl(GC_PREPARE);
                HSProcessExecution(&tempData);
                
            }//SPME进样流程
            else if(strcmp(tempData.methodName, "SPME_STD") == 0)
            {

                //打印测试
                printf("tempData.methodName = %s\n", tempData.methodName);

                //GC进入预运行状态
                GCCtrl(GC_PREPARE);
                SPMEProcessExecution(&tempData);                
            }
        }
        printf("LiquidSamplingFlowFunc\r\n");
    }

    //释放资源
    delete msgData;
    
    printf("流程线程退出\r\n");
    pthread_exit(NULL);
}


//液体进样流程
void LiquidProcessExecution(LiquidSamplingFlowType* data)
{
    uint8_t washStep = 0;
    uint32_t stateData = 0;
    bool ret = false;
    std::string Name;

    bool z_done = false, x_done = false, y_done = false, o_done = false;
    struct timeval start, now;
    const int timeout_seconds = 10;  // 超时时间，单位：秒
    int32_t vol;

    washStep = 0;

    InjectionReady.store(0); //是否注入样品
    CanExecuteSample.store(0); //当前流程是否结束

    while (true)
    {
        switch(washStep)
        {
            case 0: //读取当前模式

                washStep = 5;
                
                break;

            case 1: //Z轴上拉
                stateData = 0;
                
                MotorRelPosCtrl(true, en_MOVE, en_Z, -70.0, 150.0);
                gettimeofday(&start, nullptr);  // 记录开始时间
                z_done = false;
                while(!z_done)
                {
                    SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&stateData));

                    if ((stateData >> 10) & 0x01)
                    {
                        z_done = true;
                    }
                    else
                    {
                        // 检查是否超时
                        gettimeofday(&now, nullptr);
                        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                        if (elapsed >= timeout_seconds) 
                        {
                            SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                            printf("LIQUID 中2 的Z轴电流值 = %d\r\n",vol);
                            alarmCodeVar.addAlarm(0xa2);
                            sleep(3);
                        }
                        else
                        {
                            alarmCodeVar.removeAlarm(0xa2);
                        }
                    }
                }
                washStep++;
                break;

            case 2: //
                washStep++;
                break;

            case 3: //切换轮廓位置模式
                //MotorPositionModle(en_Z);
                washStep++;
                break;

            case 4: //进样针锁针
                //MotorRelPosCtrl(true, en_MOVE, en_O, O_LENGTHS, 50.0);
                //while(!MotorBottominOutJudgment(en_O, 1.3));
                //MotorRelPosCtrl(true, en_STOP, en_O, O_LENGTHS, 50.0);
                washStep++;
                break;

            case 5: //溶剂清洗操作
                if(data->preWashWithSolvent1 >= 1)
                {
                    WashCtrl(SOLVENT1_NAME, data->washVialDepth, data->preWashSolventVolume, data->wastePortDepth, data->preWashAspirateFlowRate, data->preWashWithSolvent1);
                }
                if(data->preWashWithSolvent2 >= 1)
                {
                    WashCtrl(SOLVENT2_NAME, data->washVialDepth, data->preWashSolventVolume, data->wastePortDepth, data->preWashAspirateFlowRate, data->preWashWithSolvent2);
                }
                if(data->preWashWithSolvent3 >= 1)
                {
                    WashCtrl(SOLVENT3_NAME, data->washVialDepth, data->preWashSolventVolume, data->wastePortDepth, data->preWashAspirateFlowRate, data->preWashWithSolvent3);
                }
                if(data->preWashWithSolvent4 >= 1)
                {
                    WashCtrl(SOLVENT4_NAME, data->washVialDepth, data->preWashSolventVolume, data->wastePortDepth, data->preWashAspirateFlowRate, data->preWashWithSolvent4);
                }
                printf("流程 溶剂清洗完成\r\n");
                washStep++;                        
                break;

            case 6: //Z轴上拉35mm
                stateData = 0;
                
                MotorRelPosCtrl(true, en_MOVE, en_Z, -35.0, 150.0);
                
                gettimeofday(&start, nullptr);  // 记录开始时间
                z_done = false;
                while(!z_done)
                {
                    SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&stateData));
        
                    if ((stateData >> 10) & 0x01)
                    {
                        z_done = true;
                    }
                    else
                    {
                        // 检查是否超时
                        gettimeofday(&now, nullptr);
                        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                        if (elapsed >= timeout_seconds) 
                        {
                            SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                            printf("LIQUID 中6 的Z轴电流值 = %d\r\n",vol);
                            alarmCodeVar.addAlarm(0xa2);
                            sleep(3);
                        }
                        else
                        {
                            alarmCodeVar.removeAlarm(0xa2);
                        }
                    }
                }
                washStep++;                
                break;

            case 7: //样品清洗操作
                Name = data->sampleRack;
                Name.erase(std::remove(Name.begin(), Name.end(), ' '), Name.end());
                Name += " ";
                Name += data->sampleRackNum;
                if(data->sampleRinseCycles >= 1)
                {
                    WashCtrl(Name.c_str(), data->sampleVialDepth, data->sampleRinseVolume, data->wastePortDepth, data->sampleVialPenetrationSpeed, data->sampleRinseCycles);
                }
                printf("流程 样品清洗完成\r\n");
                washStep++;                     
                break;
            case 8: //冲程
                if(std::strcmp(data->delayAfterFillingStrokesUnit,"min") == 0)
                {
                    data->delayAfterFillingStrokes *= 60.0;
                }
                StrokeCtrl(data->methodName, Name.c_str(), data->fillingStrokesVolume, data->delayAfterFillingStrokes, data->fillingStrokesAspirateFlowRate, data->fillingStrokesCount, data->sampleVialDepth, data->sampleVialPenetrationSpeed);
                printf("流程 冲程完成\r\n");
                washStep++;
                break;
            case 9: //吸样
                if(std::strcmp(data->samplePostAspirateDelayUnit,"min") == 0)
                {
                    data->samplePostAspirateDelay *= 60.0;
                }
                Sampling(data->methodName, data->sampleVolume, data->sampleAspirateFlowRate, data->samplePostAspirateDelay);
                printf("流程 吸样完成\r\n");
                washStep++;
                break;
            case 10: //Z轴上升110mm
                stateData = 0;
                
                MotorRelPosCtrl(true, en_MOVE, en_Z, -110.0, 150.0);
                
                gettimeofday(&start, nullptr);  // 记录开始时间
                z_done = false;
                while(!z_done)
                {
                    SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&stateData));
        
                    if ((stateData >> 10) & 0x01)
                    {
                        z_done = true;
                    }
                    else
                    {
                        // 检查是否超时
                        gettimeofday(&now, nullptr);
                        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                        if (elapsed >= timeout_seconds) 
                        {
                            SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                            printf("LIQUID 中10 的Z轴电流值 = %d\r\n",vol);
                            alarmCodeVar.addAlarm(0xa2);
                            sleep(3);
                        }
                        else
                        {
                            alarmCodeVar.removeAlarm(0xa2);
                        }
                    }
                }
                washStep++;
                break;
            case 11: //进样
                ReadCoordinatesExec("Inlet1 1");

                InjectionReady.store(1);
                if(std::strcmp(data->preInjectionDwellTimeUnit,"min") == 0)
                {
                    data->preInjectionDwellTime *= 60.0;
                }
                PushSampling(data->methodName, data->sampleVolume, data->injectionFlowRate, data->preInjectionDwellTime, data->injectorPenetrationDepth, data->injectorPenetrationSpeed);
                printf("流程 进样完成\r\n");
                washStep++;
                break;
            case 12: //启动GC
                GCCtrl(GC_START);
                printf("流程 启动GC完成\r\n");
                washStep++;
                break;
            case 13: //进样后延迟
                if(std::strcmp(data->postInjectionDwellTimeUnit,"min") == 0)
                {
                    data->postInjectionDwellTime *= 60.0;
                }
                usleep(static_cast<int>(data->postInjectionDwellTime*1000*1000));
                washStep++;
                break;
            case 14://Z轴上升240mm
                stateData = 0;
                
                MotorRelPosCtrl(true, en_MOVE, en_Z, -(fabs(data->injectorPenetrationDepth)+240.0), 150.0);
                
                gettimeofday(&start, nullptr);  // 记录开始时间
                z_done = false;
                while(!z_done)
                {
                    SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&stateData));
        
                    if ((stateData >> 10) & 0x01)
                    {
                        z_done = true;
                    }
                    else
                    {
                        // 检查是否超时
                        gettimeofday(&now, nullptr);
                        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                        if (elapsed >= timeout_seconds) 
                        {
                            SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                            printf("LIQUID 中14 的Z轴电流值 = %d\r\n",vol);
                            alarmCodeVar.addAlarm(0xa2);
                            sleep(3);
                        }
                        else
                        {
                            alarmCodeVar.removeAlarm(0xa2);
                        }
                    }
                }
                washStep++;                
                break;
            case 15: //洗针
                if(data->postWashWithSolvent1 >= 1)
                {
                    WashCtrl(SOLVENT1_NAME, data->washVialDepth, data->postWashSolventVolume, data->wastePortDepth, data->postWashAspirateFlowRate, data->postWashWithSolvent1);
                }
                if(data->postWashWithSolvent2 >= 1)
                {
                    WashCtrl(SOLVENT2_NAME, data->washVialDepth, data->postWashSolventVolume, data->wastePortDepth, data->postWashAspirateFlowRate, data->postWashWithSolvent2);
                }
                if(data->postWashWithSolvent3 >= 1)
                {
                    WashCtrl(SOLVENT3_NAME, data->washVialDepth, data->postWashSolventVolume, data->wastePortDepth, data->postWashAspirateFlowRate, data->postWashWithSolvent3);
                }
                if(data->postWashWithSolvent4 >= 1)
                {
                    WashCtrl(SOLVENT4_NAME, data->washVialDepth, data->postWashSolventVolume, data->wastePortDepth, data->postWashAspirateFlowRate, data->postWashWithSolvent4);
                }
                printf("流程 洗针完成\r\n");
                washStep++;                     
                break;
            case 16: //回到Home位
                
                washStep++;  
                break;
            case 17: //结束
                ReadCoordinatesMotorExec(HOME_NAME);
                CanExecuteSample.store(1);
                goto end;
                break;
            default:
                break;
        }
        usleep(5*1000);        
    }

end:
    return;
}


/* 用 CLOCK_MONOTONIC 取当前毫秒 */
static inline int64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

//HS进样流程
void HSProcessExecution(LiquidSamplingFlowType* data)
{
    static uint8_t Step = 0;
    uint32_t stateData = 0;
    uint32_t x_stateData, y_stateData;
    bool ret = false;
    std::string Name;

    bool z_done = false, x_done = false, y_done = false, o_done = false;
    struct timeval start, now;
    const int timeout_seconds = 10;  // 超时时间，单位：秒
    int32_t vol;

    static int32_t incubation_stable_start_ms = -1; // 连续达标起始时刻(毫秒)
    static int32_t injection_stable_start_ms = -1; // 连续达标起始时刻(毫秒)

    Step = 0;

    bool incubationTempStable = false;
    bool syringeTempStable = false;


    InjectionReady.store(0); //是否注入样品
    CanExecuteSample.store(0); //当前流程是否结束

    incubationComp.store(0);

    //孵化模块加热
    IncubationHeat(data->heatAgitator, data->incubationTemperature);
    //sleep(1);

    //针加热
    HSNeedleHeating(data->heatSyringe, data->syringeTemperature);
    //sleep(1);

    while (true)
    {
        switch(Step)
        {

            case 0: //等待孵化器温度稳定
                //Step = 2;
                #if 1
                //pthread_rwlock_rdlock(&getIncubationRequestTypeVar_rwlock);
                //getIncubationRequestTypeVar.Temperature;
                //pthread_rwlock_unlock(&getIncubationRequestTypeVar_rwlock);

                /* 2. 判断是否在容差1℃内 */
                if (fabs(getIncubationRequestTypeVar.Temperature - data->incubationTemperature) <= 2.0)
                {
                    if (incubation_stable_start_ms < 0)               /* 首次达标 */
                    incubation_stable_start_ms = now_ms();

                    if (now_ms() - incubation_stable_start_ms >= 10*1000)
                    {
                        printf("孵化器温度已稳定\r\n");
                        IncubationTempStableFlag.store(1);
                        incubationTempStable = true;
                        incubation_stable_start_ms = -1;              /* 复位 */
                    }
                }
                else
                {
                    /* 偏离容差，计时作废 */
                    IncubationTempStableFlag.store(0);
                    incubation_stable_start_ms = -1;
                    incubationTempStable = false;
                }

                //等待进样针温度稳定
                //pthread_rwlock_rdlock(&getZboardRequestTypeVar_rwlock);
                //getZboardRequestTypeVar.Temperature;
                //pthread_rwlock_unlock(&getZboardRequestTypeVar_rwlock);

                /* 2. 判断是否在容差1℃内 */
                if (fabs(getZboardRequestTypeVar.Temperature - data->syringeTemperature) <= 2.0)
                {
                    if (injection_stable_start_ms < 0)               /* 首次达标 */
                    injection_stable_start_ms = now_ms();

                    if (now_ms() - injection_stable_start_ms >= 10*1000)
                    {
                        printf("进样针温度已稳定\r\n");
                        InjectionTempStableFlag.store(1);
                        syringeTempStable = true;                                            
                        injection_stable_start_ms = -1;              /* 复位 */
                    }
                }
                else
                {
                    /* 偏离容差，计时作废 */
                    InjectionTempStableFlag.store(0);
                    injection_stable_start_ms = -1;
                    syringeTempStable = false; 
                }

                if(syringeTempStable && incubationTempStable)
                {
                    Step = 1;
                }
            
                sleep(2);
                #endif
                break;

            case 1: 
                Step++;
                break;

            case 2: //Z轴上拉
                Step++; 
                break;

            case 3: //取样
                Name = data->sampleRack;
                Name.erase(std::remove(Name.begin(), Name.end(), ' '), Name.end());
                Name += " ";
                Name += data->sampleRackNum;
                printf("HS流程3\r\n");
                ReadCoordinatesExec(Name.c_str()); //取样
                printf("HS取样\r\n");
                Step++;
                break;

            case 4: //Z轴回升
                stateData = 0;
                
                MotorRelPosCtrl(true, en_MOVE, en_Z, -121.0, 150.0);
                gettimeofday(&start, nullptr);  // 记录开始时间
                z_done = false;

                while(!z_done)
                {
                    SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&stateData));
                
                    if ((stateData >> 10) & 0x01)
                    {
                        z_done = true;
                    }
                    else
                    {
                        // 检查是否超时
                        gettimeofday(&now, nullptr);
                        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                        if (elapsed >= timeout_seconds) 
                        {
                            SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                            printf("HS中4 的Z轴电流值 = %d\r\n",vol);
                            alarmCodeVar.addAlarm(0xa2);
                            sleep(3);
                        }
                        else
                        {
                            alarmCodeVar.removeAlarm(0xa2);
                        }
                    }
            
                    usleep(100 * 1000);
                }
                Step++;                
                break;

            case 5: //移动至孵化位置
                printf("HS流程5\r\n");
                if(Pos1Reserved.load() == 0)
                {
                    ReadCoordinatesExec(INCUBATION1_NAME);
                    Pos1Reserved.store(1);
                }
                else if(Pos2Reserved.load() == 0)
                {
                    ReadCoordinatesExec(INCUBATION2_NAME);
                    Pos2Reserved.store(1);
                }
                else if(Pos3Reserved.load() == 0)
                {
                    ReadCoordinatesExec(INCUBATION3_NAME);
                    Pos3Reserved.store(1);
                }
                else if(Pos4Reserved.load() == 0)
                {
                    ReadCoordinatesExec(INCUBATION4_NAME);
                    Pos4Reserved.store(1);
                }
                else if(Pos5Reserved.load() == 0)
                {
                    ReadCoordinatesExec(INCUBATION5_NAME);
                    Pos5Reserved.store(1);
                }
                else if(Pos6Reserved.load() == 0)
                {
                    ReadCoordinatesExec(INCUBATION6_NAME);
                    Pos6Reserved.store(1);
                }
                else
                {

                }
                printf("样品放置孵化位置\r\n");
                Step++;                
                break;

            case 6: //Z轴抬升15mm
                stateData = 0;
                
                MotorRelPosCtrl(true, en_MOVE, en_Z, -15.0, 200.0);
                
                gettimeofday(&start, nullptr);  // 记录开始时间
                z_done = false;

                while(!z_done)
                {
                    SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&stateData));
                
                    if ((stateData >> 10) & 0x01)
                    {
                        z_done = true;
                    }
                    else
                    {
                        // 检查是否超时
                        gettimeofday(&now, nullptr);
                        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                        if (elapsed >= timeout_seconds) 
                        {
                            SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                            printf("HS中6 的Z轴电流值 = %d\r\n",vol);

                            alarmCodeVar.addAlarm(0xa2);
                            
                            sleep(3);
                        }
                        else
                        {
                            alarmCodeVar.removeAlarm(0xa2);
                        }                        
                    }
            
                    usleep(100 * 1000);
                }
                Step++;                                          
                break;

            case 7: //X轴偏移放瓶 3号位和6号位方向相反（待验证）
                x_stateData = 0;
                
                MotorRelPosCtrl(true, en_MOVE, en_X, 20, 150.0);
                
                //等待X轴执行完毕
                gettimeofday(&start, nullptr);  // 记录开始时间
                x_done = false;

                while(!x_done)
                {
                    SDORead_ShellFunc(can_sfd, en_X, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&x_stateData));
                
                    if ((x_stateData >> 10) & 0x01)
                    {
                        x_done = true;
                    }
                    else
                    {
                        // 检查是否超时
                        gettimeofday(&now, nullptr);
                        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                        if (elapsed >= timeout_seconds) 
                        {
                            SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                            printf("HS中7 的X轴电流值 = %d\r\n",vol);
                            alarmCodeVar.addAlarm(0xa2);
                            sleep(3);
                        }
                        else
                        {
                            alarmCodeVar.removeAlarm(0xa2);
                        }
                    }
            
                    usleep(100 * 1000);
                }
                usleep(400*1000); //等待孵化保温盖收回
                Step++;       
                break;

            case 8: //Z轴移动安全位置
                stateData = 0;
                
                MotorRelPosCtrl(true, en_MOVE, en_Z, -90.0, 100.0);
                
                gettimeofday(&start, nullptr);  // 记录开始时间
                z_done = false;

                while(!z_done)
                {
                    SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&stateData));
                
                    if ((stateData >> 10) & 0x01)
                    {
                        z_done = true;
                    }
                    else
                    {
                        // 检查是否超时
                        gettimeofday(&now, nullptr);
                        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                        if (elapsed >= timeout_seconds) 
                        {
                            SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                            printf("HS中13 的Z轴电流值 = %d\r\n",vol);
                            alarmCodeVar.addAlarm(0xa2);
                            sleep(3);
                        }
                        else
                        {
                            alarmCodeVar.removeAlarm(0xa2);
                        }
                    }
            
                    usleep(100 * 1000);
                }            
                Step++;      
                break;

            case 9: //缓慢执行还原孵化保温盖
                y_stateData = 0;
                
                MotorRelPosCtrl(true, en_MOVE, en_Y, 140.0, 50.0);
                
                //等待Y轴执行完毕
                 //等待X轴执行完毕
                 gettimeofday(&start, nullptr);  // 记录开始时间
                 y_done = false;
 
                 while(!y_done)
                 {
                     SDORead_ShellFunc(can_sfd, en_Y, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&y_stateData));
                 
                     if ((y_stateData >> 10) & 0x01)
                     {
                         y_done = true;
                     }
                     else
                     {
                         // 检查是否超时
                         gettimeofday(&now, nullptr);
                         double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                         if (elapsed >= timeout_seconds) 
                         {
                             SDORead_ShellFunc(can_sfd, en_Y, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                             printf("HS中8 的Y轴电流值 = %d\r\n",vol);
                             alarmCodeVar.addAlarm(0xa1);
                             sleep(3);
                         }
                         else
                         {
                            alarmCodeVar.removeAlarm(0xa1);
                         }
                     }
             
                     usleep(100 * 1000);
                 }
                 Step++;  
                break;

            case 10: //开始孵化
                IncubationMotorStart(data->incubationTime, data->agitatorSpeed, data->agitatorOnTime, data->agitatorOffTime);
                printf("开始孵化\r\n");
                Step++;
                break;

            case 11: 
                x_stateData = 0;
                
                MotorRelPosCtrl(true, en_MOVE, en_X, 100, 150.0);
                
                //等待X执行完毕
                gettimeofday(&start, nullptr);  // 记录开始时间
                x_done = false;

                while(!x_done)
                {
                    SDORead_ShellFunc(can_sfd, en_X, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&x_stateData));
                
                    if ((x_stateData >> 10) & 0x01)
                    {
                        x_done = true;
                    }
                    else
                    {
                        // 检查是否超时
                        gettimeofday(&now, nullptr);
                        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                        if (elapsed >= timeout_seconds) 
                        {
                            SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                            printf("HS中10 的X轴电流值 = %d\r\n",vol);
                            alarmCodeVar.addAlarm(0xa2);
                            sleep(3);
                        }
                        else
                        {
                            alarmCodeVar.removeAlarm(0xa2);
                        }
                    }
            
                    usleep(100 * 1000);
                }
                Step++;                 
                break;

            case 12: //开阀洗针
                stateData = 0;
                
                MotorRelPosCtrl(true, en_MOVE, en_O, 72.0, 100.0);
                
                gettimeofday(&start, nullptr);  // 记录开始时间
                o_done = false;

                while(!o_done)
                {
                    SDORead_ShellFunc(can_sfd, en_O, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&stateData));
                
                    if ((stateData >> 10) & 0x01)
                    {
                        o_done = true;
                    }
                    else
                    {
                        // 检查是否超时
                        gettimeofday(&now, nullptr);
                        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                        if (elapsed >= timeout_seconds) 
                        {
                            SDORead_ShellFunc(can_sfd, en_O, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                            printf("HS中11 的O轴电流值 = %d\r\n",vol);
                            alarmCodeVar.addAlarm(0xa3);
                            sleep(3);
                        }
                        else
                        {
                            alarmCodeVar.removeAlarm(0xa3);
                        }
                    }
            
                    usleep(100 * 1000);
                }
                //ValCtrl(1);
                // 将分钟转换为微秒
                if(std::strcmp(data->preInjectionPurgeTimeUnit,"min") == 0)
                {
                    data->preInjectionPurgeTime *= 60.0;
                }
                usleep(static_cast<int>(data->preInjectionPurgeTime * 1000 * 1000));  // 60 * 1000000 是将分钟转换为微秒
                Step++;
                printf("开阀洗针\r\n");    
                break;

            case 13: //关闭电磁阀
                //ValCtrl(0);
                stateData = 0;
                
                MotorRelPosCtrl(true, en_MOVE, en_O, -72.0, 100.0);
                
                gettimeofday(&start, nullptr);  // 记录开始时间
                o_done = false;

                while(!o_done)
                {
                    SDORead_ShellFunc(can_sfd, en_O, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&stateData));
                
                    if ((stateData >> 10) & 0x01)
                    {
                        o_done = true;
                    }
                    else
                    {
                        // 检查是否超时
                        gettimeofday(&now, nullptr);
                        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                        if (elapsed >= timeout_seconds) 
                        {
                            SDORead_ShellFunc(can_sfd, en_O, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                            printf("HS中12 的O轴电流值 = %d\r\n",vol);
                            alarmCodeVar.addAlarm(0xa3);
                            sleep(3);
                        }
                        else
                        {
                            alarmCodeVar.removeAlarm(0xa3);
                        }
                    }
            
                    usleep(100 * 1000);
                }
                printf("关闭洗针阀\r\n"); 
                Step++;          
                break;

            case 14: //Y轴移动安全位置
                stateData = 0;
                
                MotorRelPosCtrl(true, en_MOVE, en_Y, -140.0, 150.0);
                
                gettimeofday(&start, nullptr);  // 记录开始时间
                y_done = false;

                while(!y_done)
                {
                    SDORead_ShellFunc(can_sfd, en_Y, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&stateData));
                
                    if ((stateData >> 10) & 0x01)
                    {
                        y_done = true;
                    }
                    else
                    {
                        // 检查是否超时
                        gettimeofday(&now, nullptr);
                        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                        if (elapsed >= timeout_seconds) 
                        {
                            SDORead_ShellFunc(can_sfd, en_Y, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                            printf("HS中14 的Y轴电流值 = %d\r\n",vol);
                            alarmCodeVar.addAlarm(0xa1);
                            sleep(3);
                        }
                        else
                        {
                            alarmCodeVar.removeAlarm(0xa1);
                        }
                    }
            
                    usleep(100 * 1000);
                }          
                Step++;
                break;
                
            case 15: //等待孵化完成
                if(incubationComp.load())
                {
                    incubationComp.store(0);
                    Step++;
                    printf("孵化完成\r\n");
                }
                else
                {
                    GetIncubationCompCMD();
                    sleep(1);
                }
                break;

            case 16: //冲程
                printf("HS流程16\r\n");
                if(std::strcmp(data->delayAfterFillingStrokesUnit,"min") == 0)
                {
                    data->delayAfterFillingStrokes *= 60.0;
                }
                if(Pos1Reserved.load() == 1)
                {
                    StrokeCtrl(data->methodName, INCUBATION1_NAME, data->fillingStrokesVolume, data->delayAfterFillingStrokes, data->fillingStrokesAspirateFlowRate, data->fillingStrokesCount, data->sampleVialDepth, data->sampleVialPenetrationSpeed);
                }
                else if(Pos2Reserved.load() == 1)
                {
                    StrokeCtrl(data->methodName, INCUBATION2_NAME, data->fillingStrokesVolume, data->delayAfterFillingStrokes, data->fillingStrokesAspirateFlowRate, data->fillingStrokesCount, data->sampleVialDepth, data->sampleVialPenetrationSpeed);
                }
                else if(Pos3Reserved.load() == 1)
                {
                    StrokeCtrl(data->methodName, INCUBATION3_NAME, data->fillingStrokesVolume, data->delayAfterFillingStrokes, data->fillingStrokesAspirateFlowRate, data->fillingStrokesCount, data->sampleVialDepth, data->sampleVialPenetrationSpeed);
                }
                else if(Pos4Reserved.load() == 1)
                {
                    StrokeCtrl(data->methodName, INCUBATION4_NAME, data->fillingStrokesVolume, data->delayAfterFillingStrokes, data->fillingStrokesAspirateFlowRate, data->fillingStrokesCount, data->sampleVialDepth, data->sampleVialPenetrationSpeed);
                }
                else if(Pos5Reserved.load() == 1)
                {
                    StrokeCtrl(data->methodName, INCUBATION5_NAME, data->fillingStrokesVolume, data->delayAfterFillingStrokes, data->fillingStrokesAspirateFlowRate, data->fillingStrokesCount, data->sampleVialDepth, data->sampleVialPenetrationSpeed);
                }
                else if(Pos6Reserved.load() == 1)
                {
                    StrokeCtrl(data->methodName, INCUBATION6_NAME, data->fillingStrokesVolume, data->delayAfterFillingStrokes, data->fillingStrokesAspirateFlowRate, data->fillingStrokesCount, data->sampleVialDepth, data->sampleVialPenetrationSpeed);
                }
                else
                {

                }
                printf("冲程\r\n"); 
                Step++;             
                break;

            case 17: //取样
                printf("HS流程17\r\n");
                if(std::strcmp(data->samplePostAspirateDelayUnit,"min") == 0)
                {
                    data->samplePostAspirateDelay *= 60.0;
                }
                Sampling(data->methodName, data->sampleVolume, data->sampleAspirateFlowRate, data->samplePostAspirateDelay);
                printf("取样\r\n"); 
                Step++;                
                break;

            case 18: //Z轴回升
                stateData = 0;
                
                MotorRelPosCtrl(true, en_MOVE, en_Z, -100.0, 150.0);
                
                gettimeofday(&start, nullptr);  // 记录开始时间
                z_done = false;
                while(!z_done)
                {
                    SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&stateData));
                
                    if ((stateData >> 10) & 0x01)
                    {
                        z_done = true;
                    }
                    else
                    {
                        // 检查是否超时
                        gettimeofday(&now, nullptr);
                        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                        if (elapsed >= timeout_seconds) 
                        {
                            SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                            printf("HS中18 的Z轴电流值 = %d\r\n",vol);
                            alarmCodeVar.addAlarm(0xa2);
                            sleep(3);
                        }
                        else
                        {
                            alarmCodeVar.removeAlarm(0xa2);
                        }
                    }
            
                    usleep(100 * 1000);
                }  
                Step++;                   
                break;

            case 19: //进样
                printf("HS流程19\r\n");
                ReadCoordinatesExec("Inlet1 1");
                InjectionReady.store(1);
                if(std::strcmp(data->preInjectionDwellTimeUnit,"min") == 0)
                {
                    data->preInjectionDwellTime *= 60.0;
                }
                PushSampling(data->methodName, data->sampleVolume, data->injectionFlowRate, data->preInjectionDwellTime, data->injectorPenetrationDepth, data->injectorPenetrationSpeed);
                GCCtrl(GC_START);
                printf("进样\r\n");  
                Step++;               
                break;

            case 20: //启动GC
                //GCCtrl(GC_START);
                printf("启动GC\r\n"); 
                Step++;
                break;

            case 21: //进样后延迟
                if(std::strcmp(data->postInjectionDwellTimeUnit,"min") == 0)
                {
                    data->postInjectionDwellTime *= 60.0;
                }
                usleep(static_cast<int>(data->postInjectionDwellTime * 1000000));
                Step++;
                break;

            case 22://Z轴上升240mm
                stateData = 0;
                
                MotorRelPosCtrl(true, en_MOVE, en_Z, -(fabs(data->injectorPenetrationDepth)+230.0), 150.0);
                gettimeofday(&start, nullptr);  // 记录开始时间
                z_done = false;
                while(!z_done)
                {
                    SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&stateData));
                
                    if ((stateData >> 10) & 0x01)
                    {
                        z_done = true;
                    }
                    else
                    {
                        // 检查是否超时
                        gettimeofday(&now, nullptr);
                        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                        if (elapsed >= timeout_seconds) 
                        {
                            SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                            printf("HS中22 的Z轴电流值 = %d\r\n",vol);
                            alarmCodeVar.addAlarm(0xa2);
                            sleep(3);
                        }
                        else
                        {
                            alarmCodeVar.removeAlarm(0xa2);
                        }
                    }
            
                    usleep(100 * 1000);
                }  
                Step++;                
                break;

            case 23: //取回样品瓶
                printf("HS流程23\r\n");
                if(Pos1Reserved.load() == 1)
                {
                    ReadCoordinatesExec(INCUBATION1_NAME);
                    Pos1Reserved.store(0);
                }
                else if(Pos2Reserved.load() == 1)
                {
                    ReadCoordinatesExec(INCUBATION2_NAME);
                    Pos2Reserved.store(0);
                }
                else if(Pos3Reserved.load() == 1)
                {
                    ReadCoordinatesExec(INCUBATION3_NAME);
                    Pos3Reserved.store(0);
                }
                else if(Pos4Reserved.load() == 1)
                {
                    ReadCoordinatesExec(INCUBATION4_NAME);
                    Pos4Reserved.store(0);
                }
                else if(Pos5Reserved.load() == 1)
                {
                    ReadCoordinatesExec(INCUBATION5_NAME);
                    Pos5Reserved.store(0);
                }
                else if(Pos6Reserved.load() == 1)
                {
                    ReadCoordinatesExec(INCUBATION6_NAME);
                    Pos6Reserved.store(0);
                }
                printf("取回样品\r\n");
                Step++;                     
                break;

            case 24: //Z轴回升
                stateData = 0;
                
                MotorRelPosCtrl(true, en_MOVE, en_Z, -120, 150.0);
                gettimeofday(&start, nullptr);  // 记录开始时间
                z_done = false;
                while(!z_done)
                {
                    SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&stateData));
                
                    if ((stateData >> 10) & 0x01)
                    {
                        z_done = true;
                    }
                    else
                    {
                        // 检查是否超时
                        gettimeofday(&now, nullptr);
                        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                        if (elapsed >= timeout_seconds) 
                        {
                            SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                            printf("HS中24 的Z轴电流值 = %d\r\n",vol);
                            alarmCodeVar.addAlarm(0xa2);
                            sleep(3);
                        }
                        else
                        {
                            alarmCodeVar.removeAlarm(0xa2);
                        }
                    }
            
                    usleep(100 * 1000);
                }  
                Step++;                     
                break;

            case 25: //缓慢放回遮盖
                y_stateData = 0;
                    
                MotorRelPosCtrl(true, en_MOVE, en_Y, 140.0, 50.0);
                gettimeofday(&start, nullptr);  // 记录开始时间
                //等待Y轴执行完毕
                y_done = false;
                while(!y_done)
                {
                    SDORead_ShellFunc(can_sfd, en_Y, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&y_stateData));
                
                    if ((y_stateData >> 10) & 0x01)
                    {
                        y_done = true;
                    }
                    else
                    {
                        // 检查是否超时
                        gettimeofday(&now, nullptr);
                        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                        if (elapsed >= timeout_seconds) 
                        {
                            SDORead_ShellFunc(can_sfd, en_Y, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                            printf("HS中25 的Y轴电流值 = %d\r\n",vol);
                            alarmCodeVar.addAlarm(0xa1);
                            sleep(3);
                        }
                        else
                        {
                            alarmCodeVar.removeAlarm(0xa1);
                        }
                    }
            
                    usleep(100 * 1000);
                }  
                Step++;
                break;

            case 26: //移动到原始位置
                printf("HS流程26\r\n");
                ReadCoordinatesExec(Name.c_str());
                Step++;
                break;

            case 27:
                stateData = 0;
                
                MotorRelPosCtrl(true, en_MOVE, en_Z, -7, 150.0);
                gettimeofday(&start, nullptr);  // 记录开始时间
                z_done = false;
                while(!z_done)
                {
                    SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&stateData));
                
                    if ((stateData >> 10) & 0x01)
                    {
                        z_done = true;
                    }
                    else
                    {
                        // 检查是否超时
                        gettimeofday(&now, nullptr);
                        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                        if (elapsed >= timeout_seconds) 
                        {
                            SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                            printf("HS中27 的Z轴电流值 = %d\r\n",vol);
                            alarmCodeVar.addAlarm(0xa2);
                            sleep(3);
                        }
                        else
                        {
                            alarmCodeVar.removeAlarm(0xa2);
                        }
                    }
            
                    usleep(100 * 1000);
                }  
                Step++;                  
                break;
            
            case 28:
                printf("样品放回原始位置完成\r\n"); 
                x_stateData = 0;

                MotorRelPosCtrl(true, en_MOVE, en_X, -25, 100.0);
                
                //MotorRelPosCtrl(true, en_MOVE, en_Y, 20.0, 100.0);
                
                //等待X轴执行完毕
                gettimeofday(&start, nullptr);  // 记录开始时间
                x_done = false;

                while(!x_done)
                {
                    SDORead_ShellFunc(can_sfd, en_X, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&x_stateData));
                
                    if ((x_stateData >> 10) & 0x01)
                    {
                        x_done = true;
                    }
                    else
                    {
                        // 检查是否超时
                        gettimeofday(&now, nullptr);
                        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                        if (elapsed >= timeout_seconds) 
                        {
                            SDORead_ShellFunc(can_sfd, en_X, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                            printf("HS中28 的X轴电流值 = %d\r\n",vol);
                            alarmCodeVar.addAlarm(0xa0);
                            sleep(3);
                        }
                        else
                        {
                            alarmCodeVar.removeAlarm(0xa0);
                        }
                    }
            
                    usleep(100 * 1000);
                }      
                Step++;
                break;

            case 29: //洗针
                printf("开阀洗针\r\n");
                stateData = 0;
                
                MotorRelPosCtrl(true, en_MOVE, en_O, 72.0, 100.0);
                gettimeofday(&start, nullptr);  // 记录开始时间
                o_done = false;
                while(!o_done)
                {
                    SDORead_ShellFunc(can_sfd, en_O, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&stateData));
                
                    if ((stateData >> 10) & 0x01)
                    {
                        o_done = true;
                    }
                    else
                    {
                        // 检查是否超时
                        gettimeofday(&now, nullptr);
                        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                        if (elapsed >= timeout_seconds) 
                        {
                            SDORead_ShellFunc(can_sfd, en_O, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                            printf("HS中29 的O轴电流值 = %d\r\n",vol);
                            alarmCodeVar.addAlarm(0xa3);
                            sleep(3);
                        }
                        else
                        {
                            alarmCodeVar.removeAlarm(0xa3);
                        }
                    }
            
                    usleep(100 * 1000);
                }  
                //ValCtrl(data->preInjectionPurgeTime);
                // 将分钟转换为微秒
                if(std::strcmp(data->postInjectionPurgeTimeUnit,"min") == 0)
                {
                    data->postInjectionPurgeTime *= 60.0;
                }
                usleep(static_cast<int>(data->postInjectionPurgeTime * 1000 * 1000));  // 60 * 1000000 是将分钟转换为微秒
                Step++;                   
                break;

            case 30: //关闭电磁阀
                //ValCtrl(0);

                stateData = 0;
                
                MotorRelPosCtrl(true, en_MOVE, en_O, -72.0, 100.0);
                gettimeofday(&start, nullptr);  // 记录开始时间
                o_done = false;
                while(!o_done)
                {
                    SDORead_ShellFunc(can_sfd, en_O, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&stateData));
                
                    if ((stateData >> 10) & 0x01)
                    {
                        o_done = true;
                    }
                    else
                    {
                        // 检查是否超时
                        gettimeofday(&now, nullptr);
                        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                        if (elapsed >= timeout_seconds) 
                        {
                            SDORead_ShellFunc(can_sfd, en_O, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                            printf("HS中30 的O轴电流值 = %d\r\n",vol);
                            alarmCodeVar.addAlarm(0xa3);
                            sleep(3);
                        }
                        else
                        {
                            alarmCodeVar.removeAlarm(0xa3);
                        }
                    }
            
                    usleep(100 * 1000);
                }  
                printf("关闭洗针阀\r\n"); 
                Step++;                
                break;

            case 31: //结束
                ReadCoordinatesMotorExec(HOME_NAME);
                CanExecuteSample.store(1);
                printf("流程结束\r\n");
                goto end;
                break;
            default:
                break;
        }
        usleep(5*1000);        
    }

end:
    return;
}


#if 1
//SPME进样流程
void SPMEProcessExecution(LiquidSamplingFlowType* data)
{
    static uint8_t Step = 0;
    uint32_t stateData = 0;
    uint32_t x_stateData, y_stateData, z_stateData;
    bool ret = false;
    std::string Name;

    bool z_done = false, x_done = false, y_done = false, o_done = false;
    struct timeval start, now;
    const int timeout_seconds = 10;  // 超时时间，单位：秒
    int32_t vol;   

    static int32_t incubation_stable_start_ms = -1; // 连续达标起始时刻(毫秒)
    static int32_t conditioning_stable_start_ms = -1; // 连续达标起始时刻(毫秒)

    Step = 0;

    bool incubationTempStable = false;
    bool conditionTempStable = false;

    InjectionReady.store(0); //是否注入样品
    CanExecuteSample.store(0); //当前流程是否结束

    incubationComp.store(0);

    //孵化模块加热
    IncubationHeat(data->heatAgitator, data->incubationTemperature);

    if(std::strcmp(data->conditioningPort, "Burn-In 1") == 0)
    {
        //老化模块加热
        AgingHeating(1, data->conditioningTemperature);
    }

    while (true)
    {
        switch(Step)
        {
            case 0: //等待孵化器温度稳定
                #if 1
                if(std::strcmp(data->conditioningPort, "Burn-In 1") == 0)
                {
                    /* 2. 判断是否在容差内 */
                    if (fabs(getIncubationRequestTypeVar.Temperature - data->incubationTemperature) <= 2.0)
                    {
                        if (incubation_stable_start_ms < 0)               /* 首次达标 */
                        incubation_stable_start_ms = now_ms();

                        if (now_ms() - incubation_stable_start_ms >= 10*1000)
                        {
                            printf("孵化温度已稳定\r\n");
                            IncubationTempStableFlag.store(1);
                            incubationTempStable = true;                         /* 进入下一流程 */
                            incubation_stable_start_ms = -1;              /* 复位 */
                        }
                    }
                    else
                    {
                        /* 偏离容差，计时作废 */
                        incubation_stable_start_ms = -1;
                        IncubationTempStableFlag.store(0);
                        incubationTempStable = false;
                    }

                    if (fabs(getMainBoardRequestTypeVar.Temperature - data->conditioningTemperature) <= 10.0)
                    {
                        if (conditioning_stable_start_ms < 0)               /* 首次达标 */
                        conditioning_stable_start_ms = now_ms();

                        if (now_ms() - conditioning_stable_start_ms >= 10*1000)
                        {
                            printf("老化温度已稳定\r\n");
                            ConditionTempStableFlag.store(1);
                            conditionTempStable = true;                         /* 进入下一流程 */
                            conditioning_stable_start_ms = -1;              /* 复位 */
                        }
                    }
                    else
                    {
                        /* 偏离容差，计时作废 */
                        ConditionTempStableFlag.store(0);
                        conditioning_stable_start_ms = -1;
                        conditionTempStable = false;
                    }

                    if(conditionTempStable&&incubationTempStable)
                    {
                        Step = 1;
                    }

                }
                else
                {
                    /* 2. 判断是否在容差1℃内 */
                    if (fabs(getIncubationRequestTypeVar.Temperature - data->incubationTemperature) <= 2.0)
                    {
                        if (incubation_stable_start_ms < 0)               /* 首次达标 */
                        incubation_stable_start_ms = now_ms();

                        if (now_ms() - incubation_stable_start_ms >= 10*1000)
                        {
                            printf("孵化温度已稳定\r\n");
                            IncubationTempStableFlag.store(1);
                            incubationTempStable = true;                          /* 进入下一流程 */
                            incubation_stable_start_ms = -1;              /* 复位 */
                        }
                    }
                    else
                    {
                        /* 偏离容差，计时作废 */
                        IncubationTempStableFlag.store(0);
                        incubation_stable_start_ms = -1;
                    }
                    
                    if(incubationTempStable)
                    {
                        Step = 1;
                    }
                }

                sleep(2);
                #endif
                Step = 1; //执行流程
                break;

            case 1:
                Step = 3; //执行流程
                break;

            case 2: //Z轴上拉
                stateData = 0;
                
                MotorRelPosCtrl(true, en_MOVE, en_Z, -70.0, 150.0);
                
                while(!((stateData>>10)&0x01))
                {
                    #ifdef MOTOR_DEB
                    printf("HS流程2\r\n");
                    int32_t readdata;
                    SDORead_ShellFunc(can_sfd, en_Z, 0x6062, 0x00, reinterpret_cast<uint8_t*>(&readdata));
                    
                    printf("Z的6062 = %x\r\n", readdata);
                    usleep(15*1000);
                    SDORead_ShellFunc(can_sfd, en_Z, 0x6064, 0x00, reinterpret_cast<uint8_t*>(&readdata));
                    printf("Z的6064 = %x\r\n", readdata);
                    usleep(100*1000);
                    #endif                    
                    
                    SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&stateData));
                    usleep(100*1000);
                }
                printf("流程重新开始\r\n");
                Step++;
                break;

            case 3: //取样
                Name = data->sampleRack;
                printf("data->sampleRack = %s\r\n",data->sampleRack);
                Name.erase(std::remove(Name.begin(), Name.end(), ' '), Name.end());
                Name += " ";
                Name += data->sampleRackNum;
                printf("data->sampleRackNum = %s\r\n",data->sampleRackNum);
                printf("SPME流程3\r\n");
                if(atoi(reinterpret_cast<char *>(data->sampleRackNum)) > 0)
                {
                    ReadCoordinatesExec(Name.c_str()); //取样
                    Step++;
                }
                else
                {
                    sleep(3);
                }
                printf("SPME取样\r\n");
                break;

            case 4: //Z轴回升
                stateData = 0;
                    
                MotorRelPosCtrl(true, en_MOVE, en_Z, -121.0, 150.0);
                gettimeofday(&start, nullptr);  // 记录开始时间
                z_done = false;

                while(!z_done)
                {
                    SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&stateData));
                
                    if ((stateData >> 10) & 0x01)
                    {
                        z_done = true;
                    }
                    else
                    {
                        // 检查是否超时
                        gettimeofday(&now, nullptr);
                        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                        if (elapsed >= timeout_seconds) 
                        {
                            SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                            printf("SPME中4 的Z轴电流值 = %d\r\n",vol);
                            alarmCodeVar.addAlarm(0xa2);
                            sleep(3);
                        }
                        else
                        {
                            alarmCodeVar.removeAlarm(0xa2);
                        }
                    }
            
                    usleep(100 * 1000);
                }
                printf("SPME中4\r\n");
                Step++;                
                break;

            case 5: //移动至孵化位置
                printf("HS流程5\r\n");
                if(Pos1Reserved.load() == 0)
                {
                    ReadCoordinatesExec(INCUBATION1_NAME);
                    Pos1Reserved.store(1);
                }
                else if(Pos2Reserved.load() == 0)
                {
                    ReadCoordinatesExec(INCUBATION2_NAME);
                    Pos2Reserved.store(1);
                }
                else if(Pos3Reserved.load() == 0)
                {
                    ReadCoordinatesExec(INCUBATION3_NAME);
                    Pos3Reserved.store(1);
                }
                else if(Pos4Reserved.load() == 0)
                {
                    ReadCoordinatesExec(INCUBATION4_NAME);
                    Pos4Reserved.store(1);
                }
                else if(Pos5Reserved.load() == 0)
                {
                    ReadCoordinatesExec(INCUBATION5_NAME);
                    Pos5Reserved.store(1);
                }
                else if(Pos6Reserved.load() == 0)
                {
                    ReadCoordinatesExec(INCUBATION6_NAME);
                    Pos6Reserved.store(1);
                }
                else
                {

                }
                printf("样品放置孵化位置\r\n");
                Step++;                
                break;

            case 6: //Z轴抬升10mm
                stateData = 0;
                    
                MotorRelPosCtrl(true, en_MOVE, en_Z, -15.0, 200.0);
                
                gettimeofday(&start, nullptr);  // 记录开始时间
                z_done = false;

                while(!z_done)
                {
                    SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&stateData));
                
                    if ((stateData >> 10) & 0x01)
                    {
                        z_done = true;
                    }
                    else
                    {
                        // 检查是否超时
                        gettimeofday(&now, nullptr);
                        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                        if (elapsed >= timeout_seconds) 
                        {
                            SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                            printf("SPME中6 的Z轴电流值 = %d\r\n",vol);

                            alarmCodeVar.addAlarm(0xa2);
                            
                            sleep(3);
                        }
                        else
                        {
                            alarmCodeVar.removeAlarm(0xa2);
                        }                        
                    }
            
                    usleep(100 * 1000);
                }
                printf("SPME中6\r\n");
                Step++;                                                   
                break;

            case 7: //X轴偏移放瓶
                x_stateData = 0;
                    
                MotorRelPosCtrl(true, en_MOVE, en_X, 20, 150.0);
                
                //等待X轴执行完毕
                gettimeofday(&start, nullptr);  // 记录开始时间
                x_done = false;

                while(!x_done)
                {
                    SDORead_ShellFunc(can_sfd, en_X, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&x_stateData));
                
                    if ((x_stateData >> 10) & 0x01)
                    {
                        x_done = true;
                    }
                    else
                    {
                        // 检查是否超时
                        gettimeofday(&now, nullptr);
                        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                        if (elapsed >= timeout_seconds) 
                        {
                            SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                            printf("SPME中7 的X轴电流值 = %d\r\n",vol);
                            alarmCodeVar.addAlarm(0xa2);
                            sleep(3);
                        }
                        else
                        {
                            alarmCodeVar.removeAlarm(0xa2);
                        }
                    }
            
                    usleep(100 * 1000);
                }
                usleep(400*1000); //等待孵化保温盖收回
                printf("SPME中7\r\n");
                Step++;       
                break;

            case 8: //Z轴移动安全位置
                stateData = 0;
                        
                MotorRelPosCtrl(true, en_MOVE, en_Z, -30, 150.0);
                
                gettimeofday(&start, nullptr);  // 记录开始时间
                z_done = false;

                while(!z_done)
                {
                    SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&stateData));
                
                    if ((stateData >> 10) & 0x01)
                    {
                        z_done = true;
                    }
                    else
                    {
                        // 检查是否超时
                        gettimeofday(&now, nullptr);
                        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                        if (elapsed >= timeout_seconds) 
                        {
                            SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                            printf("SPME中8 的Z轴电流值 = %d\r\n",vol);

                            alarmCodeVar.addAlarm(0xa2);
                            
                            sleep(3);
                        }
                        else
                        {
                            alarmCodeVar.removeAlarm(0xa2);
                        }                        
                    }
            
                    usleep(100 * 1000);
                }
                printf("SPME中8\r\n");
                Step++;            
                break;   

            case 9: //缓慢执行还原孵化保温盖
                y_stateData = 0;
                    
                MotorRelPosCtrl(true, en_MOVE, en_Y, 140.0, 50.0);
                
                //等待Y轴执行完毕
                //等待X轴执行完毕
                gettimeofday(&start, nullptr);  // 记录开始时间
                y_done = false;

                while(!y_done)
                {
                    SDORead_ShellFunc(can_sfd, en_Y, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&y_stateData));
                
                    if ((y_stateData >> 10) & 0x01)
                    {
                        y_done = true;
                    }
                    else
                    {
                        // 检查是否超时
                        gettimeofday(&now, nullptr);
                        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                        if (elapsed >= timeout_seconds) 
                        {
                            SDORead_ShellFunc(can_sfd, en_Y, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                            printf("SPME中9 的Y轴电流值 = %d\r\n",vol);
                            alarmCodeVar.addAlarm(0xa1);
                            sleep(3);
                        }
                        else
                        {
                            alarmCodeVar.removeAlarm(0xa1);
                        }
                    }
            
                    usleep(100 * 1000);
                }
                printf("SPME中9\r\n");
                Step++;  
                break;

            case 10: //开始孵化
                IncubationMotorStart(data->spmeIncubationTime, data->agitatorSpeed, data->agitatorOnTime, data->agitatorOffTime);
                printf("开始孵化\r\n");
                Step++;
                break;             

            case 11: 
                x_stateData = 0;
                    
                MotorRelPosCtrl(true, en_MOVE, en_X, 100, 150.0);
                
                //等待X执行完毕
                gettimeofday(&start, nullptr);  // 记录开始时间
                x_done = false;

                while(!x_done)
                {
                    SDORead_ShellFunc(can_sfd, en_X, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&x_stateData));
                
                    if ((x_stateData >> 10) & 0x01)
                    {
                        x_done = true;
                    }
                    else
                    {
                        // 检查是否超时
                        gettimeofday(&now, nullptr);
                        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                        if (elapsed >= timeout_seconds) 
                        {
                            SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                            printf("SPME中11 的X轴电流值 = %d\r\n",vol);
                            alarmCodeVar.addAlarm(0xa2);
                            sleep(3);
                        }
                        else
                        {
                            alarmCodeVar.removeAlarm(0xa2);
                        }
                    }
            
                    usleep(100 * 1000);
                }
                printf("SPME中10\r\n");
                Step++;   
                break;

            case 12: //开始老化
                printf("SPME中12 data->preConditioningTimeUnit = %s\r\n", data->preConditioningTimeUnit);
                printf("conditioningPort = %s\r\n", data->conditioningPort);
                if(std::strcmp(data->conditioningPort, "Burn-In 1") == 0)
                {
                    ReadCoordinatesExec(BURNIN_NAME);
                    //开阀
                    AgingValCtrl(1);
                }
                else
                {
                    ReadCoordinatesExec(INJECTOR);
                }

                if(std::strcmp(data->preConditioningTimeUnit,"min") == 0)
                {
                    data->preConditioningTime *= 60.0;
                }

                if(std::strcmp(data->preConditioningTimeUnit,"min") == 0)
                {
                    data->preConditioningTime *= 60.0;
                }
                
                SpmeSampling(55.0, data->preConditioningTime, 30, 0);

                if(std::strcmp(data->conditioningPort, "Burn-In 1") == 0)
                {
                    AgingValCtrl(0);
                }
                
                Step++;            
                break;

            case 13: //开始老化
                Step++; 
                break;

            case 14: //Z轴回升
                stateData = 0;
  
                MotorRelPosCtrl(true, en_MOVE, en_Z, -zposRecord, 150.0);
                
                gettimeofday(&start, nullptr);  // 记录开始时间
                z_done = false;

                while(!z_done)
                {
                    SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&stateData));
                
                    if ((stateData >> 10) & 0x01)
                    {
                        z_done = true;
                    }
                    else
                    {
                        // 检查是否超时
                        gettimeofday(&now, nullptr);
                        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                        if (elapsed >= timeout_seconds) 
                        {
                            SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                            printf("SPME中14 的Z轴电流值 = %d\r\n",vol);

                            alarmCodeVar.addAlarm(0xa2);
                            
                            sleep(3);
                        }
                        else
                        {
                            alarmCodeVar.removeAlarm(0xa2);
                        }                        
                    }
            
                    usleep(100 * 1000);
                }
                printf("SPME中14\r\n");
                Step++;
                break;

            case 15: //等待孵化结束
                
                y_stateData = 0;
                    
                MotorRelPosCtrl(true, en_MOVE, en_Y, -yposRecord, 150.0);
                
                gettimeofday(&start, nullptr);  // 记录开始时间
                y_done = false;

                while(!y_done)
                {
                    SDORead_ShellFunc(can_sfd, en_Y, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&y_stateData));
                
                    if ((y_stateData >> 10) & 0x01)
                    {
                        y_done = true;
                    }
                    else
                    {
                        // 检查是否超时
                        gettimeofday(&now, nullptr);
                        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                        if (elapsed >= timeout_seconds) 
                        {
                            SDORead_ShellFunc(can_sfd, en_Y, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                            printf("SPME中15 的Y轴电流值 = %d\r\n",vol);
                            alarmCodeVar.addAlarm(0xa1);
                            sleep(3);
                        }
                        else
                        {
                            alarmCodeVar.removeAlarm(0xa1);
                        }
                    }
            
                    usleep(100 * 1000);
                }
                Step++;          
                break;

            case 16: //Y轴后移
                printf("SPME中16\r\n");
                if(incubationComp.load())
                {
                    incubationComp.store(0);
                    Step++;
                    printf("孵化完成\r\n");
                }
                else
                {
                    GetIncubationCompCMD();
                    sleep(1);
                }  
                break;

            case 17: //移动到目标位置
                if(Pos1Reserved.load() == 1)
                {
                    HSReadCoordinatesExec(INCUBATION1_NAME);
                }
                else if(Pos2Reserved.load() == 1)
                {
                    HSReadCoordinatesExec(INCUBATION2_NAME);
                }
                else if(Pos3Reserved.load() == 1)
                {
                    HSReadCoordinatesExec(INCUBATION3_NAME);
                }
                else if(Pos4Reserved.load() == 1)
                {
                    HSReadCoordinatesExec(INCUBATION4_NAME);
                }
                else if(Pos5Reserved.load() == 1)
                {
                    HSReadCoordinatesExec(INCUBATION5_NAME);
                }
                else if(Pos6Reserved.load() == 1)
                {
                    HSReadCoordinatesExec(INCUBATION6_NAME);
                }
                else
                {

                }
                printf("SPME中17\r\n");           
                Step++; 
                break;
    
            case 18: //样品富集
                printf("取样 data->sampleExtractTimeUnit = %s\r\n", data->sampleExtractTimeUnit); 
                if(std::strcmp(data->sampleExtractTimeUnit,"min") == 0)
                {
                    data->sampleExtractTime *= 60.0;

                }
                SpmeSampling(data->sampleVialDepth, data->sampleExtractTime, data->sampleVialPenetrationSpeed, 0);
                Step++;                
                break;

            case 19: //Z轴回升
                stateData = 0;
                
                MotorRelPosCtrl(true, en_MOVE, en_Z, -100.0, 150.0);
                
                gettimeofday(&start, nullptr);  // 记录开始时间
                z_done = false;

                while(!z_done)
                {
                    SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&stateData));
                
                    if ((stateData >> 10) & 0x01)
                    {
                        z_done = true;
                    }
                    else
                    {
                        // 检查是否超时
                        gettimeofday(&now, nullptr);
                        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                        if (elapsed >= timeout_seconds) 
                        {
                            SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                            printf("SPME中18 的Z轴电流值 = %d\r\n",vol);

                            alarmCodeVar.addAlarm(0xa2);
                            
                            sleep(3);
                        }
                        else
                        {
                            alarmCodeVar.removeAlarm(0xa2);
                        }                        
                    }
            
                    usleep(100 * 1000);
                }
                printf("SPME中19\r\n");
                Step++;                 
                break;

            case 20: //进样
                ReadCoordinatesExec("Inlet1 1");
                printf("进样 data->sampleDesorbTimeUnit = %s\r\n", data->sampleDesorbTimeUnit); 
                if(std::strcmp(data->sampleDesorbTimeUnit,"min") == 0)
                {
                    data->sampleDesorbTime *= 60.0;
                }
                SpmeSampling(data->injectorPenetrationDepth,data->sampleDesorbTime, data->injectorPenetrationSpeed, 1);
                InjectionReady.store(1); 
                Step++;               
                break;

            case 21: //启动GC
                printf("启动GC\r\n"); 
                Step++;
                break;

            case 22://Z轴上升240mm
                stateData = 0;
                
                MotorRelPosCtrl(true, en_MOVE, en_Z, -230.0, 150.0);
                
                gettimeofday(&start, nullptr);  // 记录开始时间
                z_done = false;

                while(!z_done)
                {
                    SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&stateData));
                
                    if ((stateData >> 10) & 0x01)
                    {
                        z_done = true;
                    }
                    else
                    {
                        // 检查是否超时
                        gettimeofday(&now, nullptr);
                        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                        if (elapsed >= timeout_seconds) 
                        {
                            SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                            printf("SPME中22 的Z轴电流值 = %d\r\n",vol);

                            alarmCodeVar.addAlarm(0xa2);
                            
                            sleep(3);
                        }
                        else
                        {
                            alarmCodeVar.removeAlarm(0xa2);
                        }                        
                    }
            
                    usleep(100 * 1000);
                }
                printf("SPME中22\r\n");
                Step++;                
                break;

            case 23: //取回样品瓶
                printf("SPME中23\r\n");
                if(Pos1Reserved.load() == 1)
                {
                    ReadCoordinatesExec(INCUBATION1_NAME);
                    Pos1Reserved.store(0);
                }
                else if(Pos2Reserved.load() == 1)
                {
                    ReadCoordinatesExec(INCUBATION2_NAME);
                    Pos2Reserved.store(0);
                }
                else if(Pos3Reserved.load() == 1)
                {
                    ReadCoordinatesExec(INCUBATION3_NAME);
                    Pos3Reserved.store(0);
                }
                else if(Pos4Reserved.load() == 1)
                {
                    ReadCoordinatesExec(INCUBATION4_NAME);
                    Pos4Reserved.store(0);
                }
                else if(Pos5Reserved.load() == 1)
                {
                    ReadCoordinatesExec(INCUBATION5_NAME);
                    Pos5Reserved.store(0);
                }
                else if(Pos6Reserved.load() == 1)
                {
                    ReadCoordinatesExec(INCUBATION6_NAME);
                    Pos6Reserved.store(0);
                }
                printf("取回样品\r\n");
                Step++;                     
                break;

            case 24: //Z轴回升
                stateData = 0;
                
                MotorRelPosCtrl(true, en_MOVE, en_Z, -120, 150.0);
                
                gettimeofday(&start, nullptr);  // 记录开始时间
                z_done = false;

                while(!z_done)
                {
                    SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&stateData));
                
                    if ((stateData >> 10) & 0x01)
                    {
                        z_done = true;
                    }
                    else
                    {
                        // 检查是否超时
                        gettimeofday(&now, nullptr);
                        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                        if (elapsed >= timeout_seconds) 
                        {
                            SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                            printf("SPME中23 的Z轴电流值 = %d\r\n",vol);

                            alarmCodeVar.addAlarm(0xa2);
                            
                            sleep(3);
                        }
                        else
                        {
                            alarmCodeVar.removeAlarm(0xa2);
                        }                        
                    }
            
                    usleep(100 * 1000);
                }
                printf("SPME中24\r\n");
                Step++;                     
                break;

            case 25: //缓慢放回遮盖
                y_stateData = 0;
                    
                MotorRelPosCtrl(true, en_MOVE, en_Y, 140.0, 50.0);
                
                gettimeofday(&start, nullptr);  // 记录开始时间
                y_done = false;

                while(!y_done)
                {
                    SDORead_ShellFunc(can_sfd, en_Y, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&y_stateData));
                
                    if ((y_stateData >> 10) & 0x01)
                    {
                        y_done = true;
                    }
                    else
                    {
                        // 检查是否超时
                        gettimeofday(&now, nullptr);
                        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                        if (elapsed >= timeout_seconds) 
                        {
                            SDORead_ShellFunc(can_sfd, en_Y, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                            printf("SPME中24 的Y轴电流值 = %d\r\n",vol);

                            alarmCodeVar.addAlarm(0xa1);
                            
                            sleep(3);
                        }
                        else
                        {
                            alarmCodeVar.removeAlarm(0xa1);
                        }                        
                    }
            
                    usleep(100 * 1000);
                }
                printf("SPME中25\r\n");
                Step++;
                break;

            case 26: //移动到原始位置
                printf("SPME中26\r\n");
                ReadCoordinatesExec(Name.c_str());
                Step++;
                break;

            case 27:
                stateData = 0;
                
                MotorRelPosCtrl(true, en_MOVE, en_Z, -5, 150.0);
                
                gettimeofday(&start, nullptr);  // 记录开始时间
                z_done = false;

                while(!z_done)
                {
                    SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&stateData));
                
                    if ((stateData >> 10) & 0x01)
                    {
                        z_done = true;
                    }
                    else
                    {
                        // 检查是否超时
                        gettimeofday(&now, nullptr);
                        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                        if (elapsed >= timeout_seconds) 
                        {
                            SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                            printf("SPME中26 的Z轴电流值 = %d\r\n",vol);

                            alarmCodeVar.addAlarm(0xa2);
                            
                            sleep(3);
                        }
                        else
                        {
                            alarmCodeVar.removeAlarm(0xa2);
                        }                        
                    }
            
                    usleep(100 * 1000);
                }
                printf("SPME中27\r\n");
                Step++;                  
                break;
            
            case 28:
                printf("样品放回原始位置完成\r\n"); 
                x_stateData = 0;
                
                MotorRelPosCtrl(true, en_MOVE, en_X, -25, 100.0);
                
                //等待X轴执行完毕
                gettimeofday(&start, nullptr);  // 记录开始时间
                x_done = false;

                while(!x_done)
                {
                    SDORead_ShellFunc(can_sfd, en_X, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&x_stateData));
                
                    if ((x_stateData >> 10) & 0x01)
                    {
                        x_done = true;
                    }
                    else
                    {
                        // 检查是否超时
                        gettimeofday(&now, nullptr);
                        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                        if (elapsed >= timeout_seconds) 
                        {
                            SDORead_ShellFunc(can_sfd, en_X, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                            printf("SPME中27 的X轴电流值 = %d\r\n",vol);
                            alarmCodeVar.addAlarm(0xa0);
                            sleep(3);
                        }
                        else
                        {
                            alarmCodeVar.removeAlarm(0xa0);
                        }
                    }
            
                    usleep(100 * 1000);
                }      
                Step++;
                break;

            case 29:
                stateData = 0;
                    
                MotorRelPosCtrl(true, en_MOVE, en_Z, -50, 150.0);
                
                gettimeofday(&start, nullptr);  // 记录开始时间
                z_done = false;

                while(!z_done)
                {
                    SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&stateData));
                
                    if ((stateData >> 10) & 0x01)
                    {
                        z_done = true;
                    }
                    else
                    {
                        // 检查是否超时
                        gettimeofday(&now, nullptr);
                        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                        if (elapsed >= timeout_seconds) 
                        {
                            SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                            printf("SPME中28 的Z轴电流值 = %d\r\n",vol);

                            alarmCodeVar.addAlarm(0xa2);
                            
                            sleep(3);
                        }
                        else
                        {
                            alarmCodeVar.removeAlarm(0xa2);
                        }                        
                    }
            
                    usleep(100 * 1000);
                }
                printf("SPME中29\r\n");
                Step++;                   
                break;

            case 30: //老化清洗
                printf("SPME中30 data->postConditioningTimeUnit = %s\r\n", data->postConditioningTimeUnit);
                if(std::strcmp(data->conditioningPort, "Burn-In 1") == 0)
                {
                    ReadCoordinatesExec(BURNIN_NAME);
                    AgingValCtrl(1);
                }
                else
                {
                    ReadCoordinatesExec(INJECTOR);
                }
                
                if(std::strcmp(data->postConditioningTimeUnit,"min") == 0)
                {
                    data->postConditioningTime *= 60.0;
                }
                
                SpmeSampling(55.0,data->postConditioningTime,30,0);

                if(std::strcmp(data->conditioningPort, "Burn-In 1") == 0)
                {
                    AgingValCtrl(0);
                }
                
                Step++;  
                break;

            case 31: //结束
                ReadCoordinatesMotorExec(HOME_NAME);
                CanExecuteSample.store(1);
                printf("流程结束\r\n");
                goto end;
                break;
            default:
                break;
        }
        usleep(5*1000);        
    }

end:
    return;
}
#endif


//SPME取样
void SpmeSampling(double sampleVialDepth, double time, double speed, uint8_t gcCtrl)
{
    uint32_t o_stateData;
    uint32_t z_stateData;
    double dis = 0;
    uint8_t ExecStep0Flg = 0;
    uint8_t ExecStep1Flg = 0;
    uint8_t ExecStep2Flg = 0;
    uint8_t ExecStep3Flg = 0;

    float L = 0;

    bool z_done = false, o_done = false;
    struct timeval start, now;
    const int timeout_seconds = 10;  // 超时时间，单位：秒
    int32_t vol;

    //读出纤维头长度L
    read_key_value(CHANGE_TOOL_PATH, "FiberLength", L); 

    printf("pre sampleVialDepth = %f\r\n", sampleVialDepth);


    // Z轴条件判断
    z_stateData = 0;
    if(sampleVialDepth < 30.0)
    {
        ExecStep0Flg = 1;
    }
    else if(sampleVialDepth>=30.0 && sampleVialDepth<=40.0)
    {
        ExecStep1Flg = 1;
    }
    else if(sampleVialDepth>40.0 && sampleVialDepth<=(40.0+L))
    {
        ExecStep2Flg = 1;
    }
    else if(sampleVialDepth > 70.0)
    {
        ExecStep3Flg = 1;
        sampleVialDepth = 70.0;
    }

    printf("L = %f\r\n", L);
    printf("sampleVialDepth = %f\r\n", sampleVialDepth);
    printf("ExecStep1Flg = %d\r\n", ExecStep1Flg);
    printf("ExecStep2Flg = %d\r\n", ExecStep2Flg);
    printf("ExecStep3Flg = %d\r\n", ExecStep3Flg);

    if(ExecStep2Flg)
    {
        MotorRelPosCtrl(true, en_MOVE, en_Z, 40.0, speed);
    }
    else
    {
        MotorRelPosCtrl(true, en_MOVE, en_Z, sampleVialDepth, speed);
    }
    gettimeofday(&start, nullptr);  // 记录开始时间
    z_done = false;
    while(!z_done)
    {
        SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&z_stateData));
    
        if ((z_stateData >> 10) & 0x01)
        {
            z_done = true;
        }
        else
        {
            // 检查是否超时
            gettimeofday(&now, nullptr);
            double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
            if (elapsed >= timeout_seconds) 
            {
                SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                printf("SpmePushSampling中1 的Z轴电流值 = %d\r\n",vol);
                sleep(3);
            }
        }

        usleep(100 * 1000);
    }
    

    if(!ExecStep0Flg)
    {
        o_stateData = 0;
        MotorRelPosCtrl(true, en_MOVE, en_O, L, 50);
        gettimeofday(&start, nullptr);  // 记录开始时间
        o_done = false;
        while(!o_done)
        {
            SDORead_ShellFunc(can_sfd, en_O, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&o_stateData));
        
            if ((o_stateData >> 10) & 0x01)
            {
                o_done = true;
            }
            else
            {
                // 检查是否超时
                gettimeofday(&now, nullptr);
                double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                if (elapsed >= timeout_seconds) 
                {
                    SDORead_ShellFunc(can_sfd, en_O, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                    printf("SpmePushSampling中2 的O轴电流值 = %d\r\n",vol);
                    sleep(3);
                }
            }
    
            usleep(100 * 1000);
        }
    }
   
    
    if(ExecStep2Flg || ExecStep3Flg)
    {
        z_stateData = 0;
        MotorRelPosCtrl(true, en_MOVE, en_Z, sampleVialDepth-40, speed);
        gettimeofday(&start, nullptr);  // 记录开始时间
        z_done = false;
        while(!z_done)
        {
            SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&z_stateData));
        
            if ((z_stateData >> 10) & 0x01)
            {
                z_done = true;
            }
            else
            {
                // 检查是否超时
                gettimeofday(&now, nullptr);
                double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                if (elapsed >= timeout_seconds) 
                {
                    SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                    printf("SpmePushSampling中1 的Z轴电流值 = %d\r\n",vol);
                    sleep(3);
                }
            }
    
            usleep(100 * 1000);
        }        
    }

    if(gcCtrl)
    {
        GCCtrl(GC_START);
    }
    

    usleep(static_cast<int>(time*1000*1000));

    //收针过程
    if(ExecStep2Flg || ExecStep3Flg)
    {
        z_stateData = 0;
        MotorRelPosCtrl(true, en_MOVE, en_Z, -(sampleVialDepth-40), speed);
        gettimeofday(&start, nullptr);  // 记录开始时间
        z_done = false;
        while(!z_done)
        {
            SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&z_stateData));
        
            if ((z_stateData >> 10) & 0x01)
            {
                z_done = true;
            }
            else
            {
                // 检查是否超时
                gettimeofday(&now, nullptr);
                double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                if (elapsed >= timeout_seconds) 
                {
                    SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                    printf("SpmePushSampling中1 的Z轴电流值 = %d\r\n",vol);
                    sleep(3);
                }
            }
    
            usleep(100 * 1000);
        }
    }

    if(ExecStep1Flg || ExecStep2Flg || ExecStep3Flg)
    {
        o_stateData = 0;
        MotorRelPosCtrl(true, en_MOVE, en_O, -L, 50);
        gettimeofday(&start, nullptr);  // 记录开始时间
        o_done = false;
        while(!o_done)
        {
            SDORead_ShellFunc(can_sfd, en_O, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&o_stateData));
        
            if ((o_stateData >> 10) & 0x01)
            {
                o_done = true;
            }
            else
            {
                // 检查是否超时
                gettimeofday(&now, nullptr);
                double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                if (elapsed >= timeout_seconds) 
                {
                    SDORead_ShellFunc(can_sfd, en_O, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                    printf("SpmePushSampling中3 的O轴电流值 = %d\r\n",vol);
                    sleep(3);
                }
            }

            usleep(100 * 1000);
        }
    }
    
    if(ExecStep1Flg)
    {
        z_stateData = 0;
        MotorRelPosCtrl(true, en_MOVE, en_Z, -sampleVialDepth, speed);
        gettimeofday(&start, nullptr);  // 记录开始时间
        z_done = false;
        while(!z_done)
        {
            SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&z_stateData));
        
            if ((z_stateData >> 10) & 0x01)
            {
                z_done = true;
            }
            else
            {
                // 检查是否超时
                gettimeofday(&now, nullptr);
                double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                if (elapsed >= timeout_seconds) 
                {
                    SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                    printf("SpmePushSampling中1 的Z轴电流值 = %d\r\n",vol);
                    sleep(3);
                }
            }
    
            usleep(100 * 1000);
        }
    }


    if(ExecStep2Flg || ExecStep3Flg)
    {
        z_stateData = 0;
        MotorRelPosCtrl(true, en_MOVE, en_Z, -40, 100);
        gettimeofday(&start, nullptr);  // 记录开始时间
        z_done = false;
        while(!z_done)
        {
            SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&z_stateData));
        
            if ((z_stateData >> 10) & 0x01)
            {
                z_done = true;
            }
            else
            {
                // 检查是否超时
                gettimeofday(&now, nullptr);
                double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                if (elapsed >= timeout_seconds) 
                {
                    SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                    printf("SpmePushSampling中1 的Z轴电流值 = %d\r\n",vol);
                    sleep(3);
                }
            }
    
            usleep(100 * 1000);
        }
    }
    
}


//取样
void Sampling(const char* methName,double Volume, double Rate, double Delay)
{
    uint32_t o_stateData;

    bool o_done = false;
    struct timeval start, now;
    const int timeout_seconds = 10;  // 超时时间，单位：秒
    int32_t vol;

    //抽取对应体积溶液 
    double temp;

    //temp = (Volume*5.4+0.6)*1.4;
    if(std::strcmp(methName,"HS_STD") == 0)
    {
        temp = Volume*60.0/2.5;
    }
    else
    {
        temp = (Volume*5.4)*1.4;
    }
    
    o_stateData = 0;
    
    MotorRelPosCtrl(true, en_MOVE, en_O, temp, Rate);
    
    gettimeofday(&start, nullptr);  // 记录开始时间
    o_done = false;
    while(!o_done)
    {
        SDORead_ShellFunc(can_sfd, en_O, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&o_stateData));
    
        if ((o_stateData >> 10) & 0x01)
        {
            o_done = true;
        }
        else
        {
            // 检查是否超时
            gettimeofday(&now, nullptr);
            double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
            if (elapsed >= timeout_seconds) 
            {
                SDORead_ShellFunc(can_sfd, en_O, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                printf("Sampling中1 的O轴电流值 = %d\r\n",vol);
                sleep(3);
            }
        }

        usleep(100 * 1000);
    }  

    usleep(static_cast<int>(Delay*1000*1000));

    //排出40%
    if(std::strcmp(methName,"HS_STD") == 0)
    {
        return;
    }
    else
    {
        temp = -(temp - (Volume*5.4));
    }
    
    MotorRelPosCtrl(true, en_MOVE, en_O, temp, Rate);
    o_stateData = 0;
    gettimeofday(&start, nullptr);  // 记录开始时间
    o_done = false;
    while(!o_done)
    {
        SDORead_ShellFunc(can_sfd, en_O, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&o_stateData));
    
        if ((o_stateData >> 10) & 0x01)
        {
            o_done = true;
        }
        else
        {
            // 检查是否超时
            gettimeofday(&now, nullptr);
            double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
            if (elapsed >= timeout_seconds) 
            {
                SDORead_ShellFunc(can_sfd, en_O, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                printf("Sampling中2 的O轴电流值 = %d\r\n",vol);
                sleep(3);
            }
        }

        usleep(100 * 1000);
    }   
}


//SPME老化
void SpmeConditioning(double time)
{
    uint32_t o_stateData;
    uint32_t z_stateData;

    ReadCoordinatesExec(BURNIN_NAME);

    //扎针深度进行
    z_stateData = 0;
    MotorRelPosCtrl(true, en_MOVE, en_Z, 30, 100);
    while(!((z_stateData>>10)&0x01))
    {
        SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&z_stateData));
        usleep(100*1000);
        #ifdef MOTOR_DEB
        int32_t readdata;
        SDORead_ShellFunc(can_sfd, en_Z, 0x6062, 0x00, reinterpret_cast<uint8_t*>(&readdata));
        printf("Z的6062 = %x\r\n", readdata);
        usleep(15*1000);
        SDORead_ShellFunc(can_sfd, en_Z, 0x6064, 0x00, reinterpret_cast<uint8_t*>(&readdata));
        printf("Z的6064 = %x\r\n", readdata);
        usleep(100*1000);
        #endif
    }
    
    //扎针深度进行
    z_stateData = 0;
    o_stateData = 0;
    MotorRelPosCtrl(true, en_MOVE, en_Z, 15, 100);
    MotorRelPosCtrl(true, en_MOVE, en_O, 20, 100);
    while(!(((z_stateData>>10)&0x01) && ((z_stateData>>10)&0x01)))
    {
        SDORead_ShellFunc(can_sfd, en_O, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&z_stateData));
        usleep(15*1000);
        SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&z_stateData));
        usleep(100*1000);
        #ifdef MOTOR_DEB
        int32_t readdata;
        SDORead_ShellFunc(can_sfd, en_Z, 0x6062, 0x00, reinterpret_cast<uint8_t*>(&readdata));
        printf("Z的6062 = %x\r\n", readdata);
        usleep(15*1000);
        SDORead_ShellFunc(can_sfd, en_Z, 0x6064, 0x00, reinterpret_cast<uint8_t*>(&readdata));
        printf("Z的6064 = %x\r\n", readdata);
        usleep(100*1000);
        #endif
    }

    //开启电磁阀
    
    usleep(static_cast<int>(time*1000*1000));

    //关闭电磁阀

    z_stateData = 0;
    o_stateData = 0;
    MotorRelPosCtrl(true, en_MOVE, en_Z, -15, 100);
    MotorRelPosCtrl(true, en_MOVE, en_O, -20, 100);
    while(!(((z_stateData>>10)&0x01) && ((z_stateData>>10)&0x01)))
    {
        SDORead_ShellFunc(can_sfd, en_O, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&z_stateData));
        usleep(15*1000);
        SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&z_stateData));
        usleep(100*1000);
        #ifdef MOTOR_DEB
        int32_t readdata;
        SDORead_ShellFunc(can_sfd, en_Z, 0x6062, 0x00, reinterpret_cast<uint8_t*>(&readdata));
        printf("Z的6062 = %x\r\n", readdata);
        usleep(15*1000);
        SDORead_ShellFunc(can_sfd, en_Z, 0x6064, 0x00, reinterpret_cast<uint8_t*>(&readdata));
        printf("Z的6064 = %x\r\n", readdata);
        usleep(100*1000);
        #endif
    }
    
    //扎针深度进行
    z_stateData = 0;
    MotorRelPosCtrl(true, en_MOVE, en_Z, -30, 100);
    while(!((z_stateData>>10)&0x01))
    {
        SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&z_stateData));
        usleep(100*1000);
        #ifdef MOTOR_DEB
        int32_t readdata;
        SDORead_ShellFunc(can_sfd, en_Z, 0x6062, 0x00, reinterpret_cast<uint8_t*>(&readdata));
        printf("Z的6062 = %x\r\n", readdata);
        usleep(15*1000);
        SDORead_ShellFunc(can_sfd, en_Z, 0x6064, 0x00, reinterpret_cast<uint8_t*>(&readdata));
        printf("Z的6064 = %x\r\n", readdata);
        usleep(100*1000);
        #endif
    }
    
}

#if 1
// SPME 进样函数
void SpmePushSampling(double time) 
{
    uint32_t o_stateData;
    uint32_t z_stateData;
    bool z_done = false, o_done = false;
    struct timeval start, now;
    const int timeout_seconds = 10;  // 超时时间，单位：秒
    int32_t vol;


    // 阶段 1：Z 轴移动
    z_stateData = 0;
    MotorRelPosCtrl(true, en_MOVE, en_Z, 30, 100);
    gettimeofday(&start, nullptr);  // 记录开始时间
    while(!z_done)
    {
        SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&z_stateData));
    
        if ((z_stateData >> 10) & 0x01)
        {
            z_done = true;
        }
        else
        {
            // 检查是否超时
            gettimeofday(&now, nullptr);
            double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
            if (elapsed >= timeout_seconds) 
            {
                SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                printf("SpmePushSampling中1 的Z轴电流值 = %d\r\n",vol);
                sleep(3);
            }
        }

        usleep(100 * 1000);
    }

    // 阶段 2：Z 和 O 轴同时移动
    z_stateData = 0;
    o_stateData = 0;
    MotorRelPosCtrl(true, en_MOVE, en_Z, 15, 100);
    MotorRelPosCtrl(true, en_MOVE, en_O, 20, 100);
    
    gettimeofday(&start, nullptr);  // 记录开始时间
    
    z_done = false;
    o_done = false;
    while (!(z_done && o_done)) 
    {
        if (!z_done) 
        {
            SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&z_stateData));
            if ((z_stateData >> 10) & 0x01) z_done = true;
        }
        if (!o_done) 
        {
            SDORead_ShellFunc(can_sfd, en_O, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&o_stateData));
            if ((o_stateData >> 10) & 0x01) o_done = true;
        }
        usleep(100 * 1000);

        // 检查是否超时
        gettimeofday(&now, nullptr);
        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
        if (elapsed >= timeout_seconds) 
        {
            SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
            printf("SpmePushSampling中2 的Z轴电流值 = %d\r\n",vol);
            SDORead_ShellFunc(can_sfd, en_O, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
            printf("SpmePushSampling中2 的O轴电流值 = %d\r\n",vol);
            sleep(3);
        }
    }

    // 等待一段时间
    usleep(static_cast<int>(time * 60 * 1000 * 1000));

    // 阶段 3：Z 和 O 轴反向移动
    z_stateData = 0;
    o_stateData = 0;
    MotorRelPosCtrl(true, en_MOVE, en_Z, -15, 100);
    MotorRelPosCtrl(true, en_MOVE, en_O, -20, 100);
    gettimeofday(&start, nullptr);  // 重新记录开始时间
    z_done = false;
    o_done = false;

    while (!(z_done && o_done)) 
    {
        if (!z_done) 
        {
            SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&z_stateData));
            if ((z_stateData >> 10) & 0x01) z_done = true;
        }
        if (!o_done) 
        {
            SDORead_ShellFunc(can_sfd, en_O, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&o_stateData));
            if ((o_stateData >> 10) & 0x01) o_done = true;
        }
        usleep(100 * 1000);

        // 检查是否超时
        gettimeofday(&now, nullptr);
        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
        if (elapsed >= timeout_seconds) 
        {
            SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
            printf("SpmePushSampling中2 的Z轴电流值 = %d\r\n",vol);
            SDORead_ShellFunc(can_sfd, en_O, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
            printf("SpmePushSampling中2 的O轴电流值 = %d\r\n",vol);
            sleep(3);
        }
    }

    // 阶段 4：Z 轴最后反向移动
    z_stateData = 0;
    MotorRelPosCtrl(true, en_MOVE, en_Z, -30, 100);
    gettimeofday(&start, nullptr);  // 记录开始时间
    z_done = false;
    while(!z_done)
    {
        SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&z_stateData));
    
        if ((z_stateData >> 10) & 0x01)
        {
            z_done = true;
        }
        else
        {
            // 检查是否超时
            gettimeofday(&now, nullptr);
            double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
            if (elapsed >= timeout_seconds) 
            {
                SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                printf("SpmePushSampling中3 的Z轴电流值 = %d\r\n",vol);
                sleep(3);
            }
        }

        usleep(100 * 1000);
    }
}

#else
//SPME进样
void SpmePushSampling(double time)
{
    uint32_t o_stateData;
    uint32_t z_stateData;
    const int timeout_seconds = 10;  // 超时时间，单位：秒

    //扎针深度进行
    z_stateData = 0;
    MotorRelPosCtrl(true, en_MOVE, en_Z, 30, 100);
    while(!((z_stateData>>10)&0x01))
    {
        SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&z_stateData));
        usleep(100*1000);
        #ifdef MOTOR_DEB
        int32_t readdata;
        SDORead_ShellFunc(can_sfd, en_Z, 0x6062, 0x00, reinterpret_cast<uint8_t*>(&readdata));
        printf("Z的6062 = %x\r\n", readdata);
        usleep(15*1000);
        SDORead_ShellFunc(can_sfd, en_Z, 0x6064, 0x00, reinterpret_cast<uint8_t*>(&readdata));
        printf("Z的6064 = %x\r\n", readdata);
        usleep(100*1000);
        #endif
    }
    
    //扎针深度进行
    z_stateData = 0;
    o_stateData = 0;
    MotorRelPosCtrl(true, en_MOVE, en_Z, 15, 100);
    MotorRelPosCtrl(true, en_MOVE, en_O, 20, 100);
    while(!(((z_stateData>>10)&0x01) && ((z_stateData>>10)&0x01)))
    {
        SDORead_ShellFunc(can_sfd, en_O, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&z_stateData));
        usleep(15*1000);
        SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&z_stateData));
        usleep(100*1000);
        #ifdef MOTOR_DEB
        int32_t readdata;
        SDORead_ShellFunc(can_sfd, en_Z, 0x6062, 0x00, reinterpret_cast<uint8_t*>(&readdata));
        printf("Z的6062 = %x\r\n", readdata);
        usleep(15*1000);
        SDORead_ShellFunc(can_sfd, en_Z, 0x6064, 0x00, reinterpret_cast<uint8_t*>(&readdata));
        printf("Z的6064 = %x\r\n", readdata);
        usleep(100*1000);
        #endif
    }

    usleep(static_cast<int>(time*60*1000*1000));

    z_stateData = 0;
    o_stateData = 0;
    MotorRelPosCtrl(true, en_MOVE, en_Z, -15, 100);
    MotorRelPosCtrl(true, en_MOVE, en_O, -20, 100);
    while(!(((z_stateData>>10)&0x01) && ((z_stateData>>10)&0x01)))
    {
        SDORead_ShellFunc(can_sfd, en_O, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&z_stateData));
        usleep(15*1000);
        SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&z_stateData));
        usleep(100*1000);
        #ifdef MOTOR_DEB
        int32_t readdata;
        SDORead_ShellFunc(can_sfd, en_Z, 0x6062, 0x00, reinterpret_cast<uint8_t*>(&readdata));
        printf("Z的6062 = %x\r\n", readdata);
        usleep(15*1000);
        SDORead_ShellFunc(can_sfd, en_Z, 0x6064, 0x00, reinterpret_cast<uint8_t*>(&readdata));
        printf("Z的6064 = %x\r\n", readdata);
        usleep(100*1000);
        #endif
    }
    
    //扎针深度进行
    z_stateData = 0;
    MotorRelPosCtrl(true, en_MOVE, en_Z, -30, 100);
    while(!((z_stateData>>10)&0x01))
    {
        SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&z_stateData));
        usleep(100*1000);
        #ifdef MOTOR_DEB
        int32_t readdata;
        SDORead_ShellFunc(can_sfd, en_Z, 0x6062, 0x00, reinterpret_cast<uint8_t*>(&readdata));
        printf("Z的6062 = %x\r\n", readdata);
        usleep(15*1000);
        SDORead_ShellFunc(can_sfd, en_Z, 0x6064, 0x00, reinterpret_cast<uint8_t*>(&readdata));
        printf("Z的6064 = %x\r\n", readdata);
        usleep(100*1000);
        #endif
    }
    
}

#endif

//进样
void PushSampling(const char* methName, double Volume, double Rate, double Delay, double Depth, double Speed)
{
    uint32_t o_stateData;
    uint32_t z_stateData;

    bool z_done = false, o_done = false;
    struct timeval start, now;
    const int timeout_seconds = 10;  // 超时时间，单位：秒
    int32_t vol;

    //扎针深度进行
    z_stateData = 0;
    MotorRelPosCtrl(true, en_MOVE, en_Z, Depth, Speed);
    gettimeofday(&start, nullptr);  // 记录开始时间
    z_done = false;
    while(!z_done)
    {
        SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&z_stateData));

        if ((z_stateData >> 10) & 0x01)
        {
            z_done = true;
        }
        else
        {
            // 检查是否超时
            gettimeofday(&now, nullptr);
            double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
            if (elapsed >= timeout_seconds) 
            {
                SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                printf("PushSampling 中1 的Z轴电流值 = %d\r\n",vol);
                alarmCodeVar.addAlarm(0xa2);
                sleep(3);
            }
            else
            {
                alarmCodeVar.removeAlarm(0xa2);
            }
        }
    }

    usleep(static_cast<int>(Delay*1000*1000));

    //抽取对应体积溶液 
    double temp;

    if(std::strcmp(methName,"HS_STD") == 0)
    {
        temp = Volume*60.0/2.5;
    }
    else
    {
        temp = Volume*5.4;
    }
    
    
    temp = -temp;
    o_stateData = 0;
    MotorRelPosCtrl(true, en_MOVE, en_O, temp, Rate);
    gettimeofday(&start, nullptr);  // 记录开始时间
    o_done = false;
    while(!o_done)
    {
        SDORead_ShellFunc(can_sfd, en_O, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&o_stateData));

        if ((o_stateData >> 10) & 0x01)
        {
            o_done = true;
        }
        else
        {
            // 检查是否超时
            gettimeofday(&now, nullptr);
            double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
            if (elapsed >= timeout_seconds) 
            {
                SDORead_ShellFunc(can_sfd, en_O, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                printf("PushSampling 中2 的O轴电流值 = %d\r\n",vol);
                alarmCodeVar.addAlarm(0xa3);
                sleep(3);
            }
            else
            {
                alarmCodeVar.removeAlarm(0xa3);
            }            
        }
    }
}


//冲程操作
void StrokeCtrl(const char* methName, const char* stName, double Volume, double Delay, double Rate, int32_t count, double Depth, double ZRate)
{
    uint32_t z_stateData;
    uint32_t o_stateData;

    bool z_done = false, o_done = false;
    struct timeval start, now;
    const int timeout_seconds = 10;  // 超时时间，单位：秒
    int32_t vol;

    double speed;

    //读取坐标并执行到相应位置
    if(std::strncmp(stName,"Agitator1",9) == 0)
    {
        HSReadCoordinatesExec(stName);
    }
    else
    {
        ReadCoordinatesExec(stName);
    }
    

    //扎针深度进行
    z_stateData = 0;
    if(std::strncmp(stName,"Agitator1",9) == 0)
    {
        MotorRelPosCtrl(true, en_MOVE, en_Z, Depth+18, ZRate);
    }
    else
    {
        MotorRelPosCtrl(true, en_MOVE, en_Z, Depth, ZRate);
    }
    
    gettimeofday(&start, nullptr);  // 记录开始时间
    while(!z_done)
    {
        SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&z_stateData));
    
        if ((z_stateData >> 10) & 0x01)
        {
            z_done = true;
        }
        else
        {
            // 检查是否超时
            gettimeofday(&now, nullptr);
            double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
            if (elapsed >= timeout_seconds) 
            {
                SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                printf("StrokeCtrl中1 的Z轴电流值 = %d\r\n",vol);
                alarmCodeVar.addAlarm(0xa2);
                sleep(3);
            }
            else
            {
                alarmCodeVar.removeAlarm(0xa2);
            }
        }

        usleep(100 * 1000);
    }  

    for(int i=0; i<count; i++)
    {
        //抽取对应体积溶液 
        double temp;

        if(std::strcmp(methName,"HS_STD") == 0)
        {
            temp = Volume*60.0/2.5;
            speed = 300;
        }
        else
        {
            temp = Volume*5.4;
            speed = 3000;
        }       
        
        
        o_stateData = 0;
        MotorRelPosCtrl(true, en_MOVE, en_O, temp, Rate);
        gettimeofday(&start, nullptr);  // 记录开始时间
        o_done = false;
        while(!o_done)
        {
            SDORead_ShellFunc(can_sfd, en_O, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&o_stateData));
        
            if ((o_stateData >> 10) & 0x01)
            {
                o_done = true;
            }
            else
            {
                // 检查是否超时
                gettimeofday(&now, nullptr);
                double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                if (elapsed >= timeout_seconds) 
                {
                    SDORead_ShellFunc(can_sfd, en_O, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                    printf("StrokeCtrl中2 的O轴电流值 = %d\r\n",vol);
                    alarmCodeVar.addAlarm(0xa3);
                    sleep(3);
                }
                else
                {
                    alarmCodeVar.removeAlarm(0xa3);
                }
            }
    
            usleep(100 * 1000);
        }  
        
        //排液
        o_stateData = 0;
        temp = -temp;
        MotorRelPosCtrl(true, en_MOVE, en_O, temp, speed);
        gettimeofday(&start, nullptr);  // 记录开始时间
        o_done = false;
        while(!o_done)
        {
            SDORead_ShellFunc(can_sfd, en_O, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&o_stateData));
        
            if ((o_stateData >> 10) & 0x01)
            {
                o_done = true;
            }
            else
            {
                // 检查是否超时
                gettimeofday(&now, nullptr);
                double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                if (elapsed >= timeout_seconds) 
                {
                    SDORead_ShellFunc(can_sfd, en_O, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                    printf("StrokeCtrl中3 的O轴电流值 = %d\r\n",vol);
                    alarmCodeVar.addAlarm(0xa3);
                    sleep(3);
                }
                else
                {
                    alarmCodeVar.removeAlarm(0xa3);
                }
            }
    
            usleep(100 * 1000);
        }  
    }

    //冲程等待时间
    usleep(static_cast<int>(Delay*1000*1000));
}

//清洗操作
void WashCtrl(const char* stName, double washDepth, double Volume, double wasteDepth, double washRate, int32_t count)
{
    uint32_t z_stateData;
    uint32_t o_stateData;

    bool z_done = false, o_done = false;
    struct timeval start, now;
    const int timeout_seconds = 10;  // 超时时间，单位：秒
    int32_t vol;

    for(int i=0; i<count; i++)
    {
        //读取坐标并执行到相应位置
        ReadCoordinatesExec(stName);
        std::cout << "读取坐标并执行到相应位置" << std::endl;

        //清洗扎针深度进行
        z_stateData = 0;
        MotorRelPosCtrl(true, en_MOVE, en_Z, washDepth, 100.0);
        gettimeofday(&start, nullptr);  // 记录开始时间
        z_done = false;
        while(!z_done)
        {
            SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&z_stateData));

            if ((z_stateData >> 10) & 0x01)
            {
                z_done = true;
            }
            else
            {
                // 检查是否超时
                gettimeofday(&now, nullptr);
                double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                if (elapsed >= timeout_seconds) 
                {
                    SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                    printf("WashCtrl 中1 的Z轴电流值 = %d\r\n",vol);
                    alarmCodeVar.addAlarm(0xa2);
                    sleep(3);
                }
                else
                {
                    alarmCodeVar.removeAlarm(0xa2);
                }
            }
        }

        //抽取对应体积溶液 
        double temp;

        if(std::strncmp(stName, "Ri", 2) == 0)
        {
            temp = (10.0*5.4)*(Volume/100.0);
        }
        else
        {
            temp = Volume*5.4;
        }
        
        o_stateData = 0;
        MotorRelPosCtrl(true, en_MOVE, en_O, temp, washRate);
        gettimeofday(&start, nullptr);  // 记录开始时间
        o_done = false;
        while(!o_done)
        {
            SDORead_ShellFunc(can_sfd, en_O, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&o_stateData));

            if ((o_stateData >> 10) & 0x01)
            {
                o_done = true;
            }
            else
            {
                // 检查是否超时
                gettimeofday(&now, nullptr);
                double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                if (elapsed >= timeout_seconds) 
                {
                    SDORead_ShellFunc(can_sfd, en_O, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                    printf("WashCtrl 中2 的O轴电流值 = %d\r\n",vol);
                    alarmCodeVar.addAlarm(0xa3);
                    sleep(3);
                }
                else
                {
                    alarmCodeVar.removeAlarm(0xa3);
                }
            }
        }

        //Z轴回升
        z_stateData = 0;
        if(std::strncmp(stName, "Ri", 2) == 0)
        {
            MotorRelPosCtrl(true, en_MOVE, en_Z, -(40.0+washDepth), 150.0);
        }
        else
        {
            MotorRelPosCtrl(true, en_MOVE, en_Z, -(100.0+washDepth), 150.0);
        }

        gettimeofday(&start, nullptr);  // 记录开始时间
        z_done = false;
        while(!z_done)
        {
            SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&z_stateData));

            if ((z_stateData >> 10) & 0x01)
            {
                z_done = true;
            }
            else
            {
                // 检查是否超时
                gettimeofday(&now, nullptr);
                double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                if (elapsed >= timeout_seconds) 
                {
                    SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                    printf("WashCtrl 中3 的Z轴电流值 = %d\r\n",vol);
                    alarmCodeVar.addAlarm(0xa2);
                    sleep(3);
                }
                else
                {
                    alarmCodeVar.removeAlarm(0xa2);
                }
            }
        }

        //移动到废液瓶并进行扎针深度
        ReadCoordinatesExec(WLIB_NAME);
        z_stateData = 0;

        MotorRelPosCtrl(true, en_MOVE, en_Z, wasteDepth, 130.0);

        gettimeofday(&start, nullptr);  // 记录开始时间
        z_done = false;
        while(!z_done)
        {
            SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&z_stateData));

            if ((z_stateData >> 10) & 0x01)
            {
                z_done = true;
            }
            else
            {
                // 检查是否超时
                gettimeofday(&now, nullptr);
                double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                if (elapsed >= timeout_seconds) 
                {
                    SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                    printf("WashCtrl 中4 的Z轴电流值 = %d\r\n",vol);
                    alarmCodeVar.addAlarm(0xa2);
                    sleep(3);
                }
                else
                {
                    alarmCodeVar.removeAlarm(0xa2);
                }
            }
        }
        
        //排废液
        o_stateData = 0;
        temp = -temp;
        MotorRelPosCtrl(true, en_MOVE, en_O, temp, 500);
        gettimeofday(&start, nullptr);  // 记录开始时间
        o_done = false;
        while(!o_done)
        {
            SDORead_ShellFunc(can_sfd, en_O, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&o_stateData));

            if ((o_stateData >> 10) & 0x01)
            {
                o_done = true;
            }
            else
            {
                // 检查是否超时
                gettimeofday(&now, nullptr);
                double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                if (elapsed >= timeout_seconds) 
                {
                    SDORead_ShellFunc(can_sfd, en_O, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                    printf("WashCtrl 中5 的O轴电流值 = %d\r\n",vol);
                    alarmCodeVar.addAlarm(0xa3);
                    sleep(3);
                }
                else
                {
                    alarmCodeVar.removeAlarm(0xa3);
                }
            }
        }

        //Z轴回升40mm
        z_stateData = 0;
        #ifdef NANOTEC
        MotorRelPosCtrl(true, en_MOVE, en_Z, -(40.0+wasteDepth), 150.0);
        #else
        MotorRelPosCtrl(true, en_MOVE, en_Z, (40.0+wasteDepth), 150.0);
        #endif
        gettimeofday(&start, nullptr);  // 记录开始时间
        z_done = false;
        while(!z_done)
        {
            SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&z_stateData));

            if ((z_stateData >> 10) & 0x01)
            {
                z_done = true;
            }
            else
            {
                // 检查是否超时
                gettimeofday(&now, nullptr);
                double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                if (elapsed >= timeout_seconds) 
                {
                    SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                    printf("WashCtrl 中6 的Z轴电流值 = %d\r\n",vol);
                    alarmCodeVar.addAlarm(0xa2);
                    sleep(3);
                }
                else
                {
                    alarmCodeVar.removeAlarm(0xa2);
                }
            }
        }
    }
}


void OMotorExec(double dis)
{

    bool o_done = false;
    struct timeval start, now;
    const int timeout_seconds = 10;  // 超时时间，单位：秒
    int32_t vol;

    uint32_t o_stateData = 0;

    //等待o轴执行完毕
    o_stateData = 0;

    MotorRelPosCtrl(true, en_MOVE, en_O, dis, 200.0);
    gettimeofday(&start, nullptr);  // 记录开始时间
    o_done = false;

    //Y轴执行完毕
    while(!o_done)
    {
        SDORead_ShellFunc(can_sfd, en_O, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&o_stateData));

        if ((o_stateData >> 10) & 0x01)
        {
            o_done = true;
        }
        else
        {
            // 检查是否超时
            gettimeofday(&now, nullptr);
            double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
            if (elapsed >= timeout_seconds) 
            {
                SDORead_ShellFunc(can_sfd, en_O, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                printf("OMotorExec中1 的o轴电流值 = %d\r\n",vol);
                alarmCodeVar.addAlarm(0xa3);
                sleep(3);
            }
            else
            {
                alarmCodeVar.removeAlarm(0xa3);
            }
        }

        usleep(100 * 1000);
    }
    
}



//读取坐标并执行到相应位置
void ReadCoordinatesMotorExec(const char* pointName)
{
    float x,y,z;
    float retY;
    int32_t x_reso,y_reso,z_reso;

    bool z_done = false, x_done = false, y_done = false;
    struct timeval start, now;
    const int timeout_seconds = 10;  // 超时时间，单位：秒
    int32_t vol;

    uint32_t x_stateData = 0;
    uint32_t y_stateData = 0;
    uint32_t z_stateData = 0;

    if(read_coordinates(pointName, &x, &y, &z) < 0)
    {
        printf("未找到对应标识坐标%s\r\n",pointName);
        return;
    }

    printf("1x = %f\n",x);
    printf("1y = %f\n",y);
    printf("1z = %f\n",z);


	//读取目标绝对位置值
	SDORead_ShellFunc(can_sfd, en_X, 0x6064, 0x00, reinterpret_cast<uint8_t*>(&x_reso));
	usleep(15*1000);

	SDORead_ShellFunc(can_sfd, en_Y, 0x6064, 0x00, reinterpret_cast<uint8_t*>(&y_reso));
	usleep(15*1000);

	SDORead_ShellFunc(can_sfd, en_Z, 0x6064, 0x00, reinterpret_cast<uint8_t*>(&z_reso));
	usleep(15*1000);
    

    printf("1当前距离 x = %d\r\n", x_reso);
    printf("1当前距离 y = %d\r\n", y_reso);
    printf("1当前距离 z = %d\r\n", z_reso);
    

    //转化为实际距离
    double x_pos = ResoConvPos(en_X, x_reso);
    double y_pos = ResoConvPos(en_Y, y_reso);
    double z_pos = ResoConvPos(en_Z, z_reso);

    x_pos = fabs(x_pos);
    y_pos = fabs(y_pos);
    z_pos = fabs(z_pos);

    #if 1
    printf("1实际距离 x = %f\r\n", x_pos);
    printf("1实际距离 y = %f\r\n", y_pos);
    printf("1实际距离 z = %f\r\n", z_pos);
    printf("1移动距离 x = %f\r\n", x-x_pos);
    printf("1移动距离 y = %f\r\n", y-y_pos);
    printf("1移动距离 z = %f\r\n", z-z_pos);
    #endif

    if(strcmp(pointName, CHANGETOOL_NAME) != 0 && strcmp(pointName, SHUTDOWN_NAME) != 0)
    {
        //等待Z轴执行完毕
        z_stateData = 0;
        int32_t z_z_pos = z-z_pos;
        if(abs(z_z_pos) > 0)
        {
            MotorRelPosCtrl(true, en_MOVE, en_Z, z-z_pos, 200.0);
            gettimeofday(&start, nullptr);  // 记录开始时间
            z_done = false;
            //Y轴执行完毕
            while(!z_done)
            {
                SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&z_stateData));
    
                if ((z_stateData >> 10) & 0x01)
                {
                    z_done = true;
                }
                else
                {
                    // 检查是否超时
                    gettimeofday(&now, nullptr);
                    double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                    if (elapsed >= timeout_seconds) 
                    {
                        SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                        printf("ReadCoordinatesMotorExec中1 的Z轴电流值 = %d\r\n",vol);
                        alarmCodeVar.addAlarm(0xa2);
                        sleep(3);
                    }
                    else
                    {
                        alarmCodeVar.removeAlarm(0xa2);
                    }
                }
    
                usleep(100 * 1000);
            }
        }
        
    }
     
    int32_t x_x_pos = x-x_pos;
    int32_t y_y_pos = y-y_pos;

    if((abs(x_x_pos) > 0) ||  abs(y_y_pos) > 0)
    {
        MotorRelPosCtrl(true, en_MOVE, en_X, x-x_pos, 300.0);

        MotorRelPosCtrl(true, en_MOVE, en_Y, y-y_pos, 200.0);
    
        //等待X轴与Y轴执行完毕
        gettimeofday(&start, nullptr);  // 记录开始时间
        while (!(x_done && y_done)) 
        {
            if (!x_done) 
            {
                SDORead_ShellFunc(can_sfd, en_X, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&x_stateData));
                if ((x_stateData >> 10) & 0x01) x_done = true;
            }
            if (!y_done) 
            {
                SDORead_ShellFunc(can_sfd, en_Y, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&y_stateData));
                if ((y_stateData >> 10) & 0x01) y_done = true;
            }
            usleep(100 * 1000);
    
            // 检查是否超时
            gettimeofday(&now, nullptr);
            double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
            if (elapsed >= timeout_seconds) 
            {
                SDORead_ShellFunc(can_sfd, en_X, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                printf("ReadCoordinatesMotorExec中2 的X轴电流值 = %d\r\n",vol);
                if(vol>=1790)
                {
                    alarmCodeVar.addAlarm(0xa0);
                }
                else
                {
                    alarmCodeVar.removeAlarm(0xa0);
                }
                SDORead_ShellFunc(can_sfd, en_Y, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                printf("ReadCoordinatesMotorExec中3 的Y轴电流值 = %d\r\n",vol);
                if(vol>=1790)
                {
                    alarmCodeVar.addAlarm(0xa1);
                }
                else
                {
                    alarmCodeVar.removeAlarm(0xa1);
                }
                sleep(3);
            }
            else
            {
                alarmCodeVar.removeAlarm(0xa0);
                alarmCodeVar.removeAlarm(0xa1);
            }
            
        }
    }

    if(strcmp(pointName, CHANGETOOL_NAME) == 0 || strcmp(pointName, SHUTDOWN_NAME) == 0)
    {
       //等待Z轴执行完毕
       z_stateData = 0;
       int32_t z_z_pos = z-z_pos;
       if(abs(z_z_pos) > 0)
       {
           MotorRelPosCtrl(true, en_MOVE, en_Z, z-z_pos, 200.0);
           gettimeofday(&start, nullptr);  // 记录开始时间
           z_done = false;
           //Y轴执行完毕
           while(!z_done)
           {
               SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&z_stateData));
   
               if ((z_stateData >> 10) & 0x01)
               {
                   z_done = true;
               }
               else
               {
                   // 检查是否超时
                   gettimeofday(&now, nullptr);
                   double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                   if (elapsed >= timeout_seconds) 
                   {
                       SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                       printf("ReadCoordinatesMotorExec中1 的Z轴电流值 = %d\r\n",vol);
                       alarmCodeVar.addAlarm(0xa2);
                       sleep(3);
                   }
                   else
                   {
                       alarmCodeVar.removeAlarm(0xa2);
                   }
               }
   
               usleep(100 * 1000);
           }
       }
    }
}


//读取坐标并执行到相应位置
void ReadCoordinatesExec(const char* pointName)
{
    float x,y,z;
    float retY;
    int32_t x_reso,y_reso,z_reso;

    bool z_done = false, x_done = false, y_done = false;
    struct timeval start, now;
    const int timeout_seconds = 10;  // 超时时间，单位：秒
    int32_t vol;

    uint32_t x_stateData = 0;
    uint32_t y_stateData = 0;
    uint32_t z_stateData = 0;

    if(read_coordinates(pointName, &x, &y, &z) < 0)
    {
        printf("未找到对应标识坐标 %s\r\n",pointName);
        return;
    }

    printf("x = %f\n",x);
    printf("y = %f\n",y);
    printf("z = %f\n",z);

    if(std::strncmp(pointName,"Agitator1",9) == 0)
    {
        retY = 120; 
        y += retY;
    }

	//读取目标绝对位置值
	SDORead_ShellFunc(can_sfd, en_X, 0x6064, 0x00, reinterpret_cast<uint8_t*>(&x_reso));
	usleep(15*1000);

	SDORead_ShellFunc(can_sfd, en_Y, 0x6064, 0x00, reinterpret_cast<uint8_t*>(&y_reso));
	usleep(15*1000);

	SDORead_ShellFunc(can_sfd, en_Z, 0x6064, 0x00, reinterpret_cast<uint8_t*>(&z_reso));
	usleep(15*1000);
    

    //printf("当前距离 x = %d\r\n", x_reso);
    //printf("当前距离 y = %d\r\n", y_reso);
    //printf("当前距离 z = %d\r\n", z_reso);
    

    //转化为实际距离
    double x_pos = ResoConvPos(en_X, x_reso);
    double y_pos = ResoConvPos(en_Y, y_reso);
    double z_pos = ResoConvPos(en_Z, z_reso);

    x_pos = fabs(x_pos);
    y_pos = fabs(y_pos);
    z_pos = fabs(z_pos);

    #if 0
    printf("实际距离 x = %f\r\n", x_pos);
    printf("实际距离 y = %f\r\n", y_pos);
    printf("实际距离 z = %f\r\n", z_pos);
    printf("移动距离 x = %f\r\n", x-x_pos);
    printf("移动距离 y = %f\r\n", y-y_pos);
    printf("移动距离 z = %f\r\n", z-z_pos);
    #endif

    MotorRelPosCtrl(true, en_MOVE, en_X, x-x_pos, 700.0);

    MotorRelPosCtrl(true, en_MOVE, en_Y, y-y_pos, 200.0);

    //等待X轴与Y轴执行完毕
    gettimeofday(&start, nullptr);  // 记录开始时间
    while (!(x_done && y_done)) 
    {
        if (!x_done) 
        {
            SDORead_ShellFunc(can_sfd, en_X, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&x_stateData));
            if ((x_stateData >> 10) & 0x01) x_done = true;
        }
        if (!y_done) 
        {
            SDORead_ShellFunc(can_sfd, en_Y, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&y_stateData));
            if ((y_stateData >> 10) & 0x01) y_done = true;
        }
        usleep(100 * 1000);

        // 检查是否超时
        gettimeofday(&now, nullptr);
        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
        if (elapsed >= timeout_seconds) 
        {
            SDORead_ShellFunc(can_sfd, en_X, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
            printf("ReadCoordinatesExec中1 的X轴电流值 = %d\r\n",vol);
            if(vol>=1790)
            {
                alarmCodeVar.addAlarm(0xa0);
            }
            else
            {
                alarmCodeVar.removeAlarm(0xa0);
            }
            SDORead_ShellFunc(can_sfd, en_Y, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
            printf("ReadCoordinatesExec中1 的Y轴电流值 = %d\r\n",vol);
            if(vol>=1790)
            {
                alarmCodeVar.addAlarm(0xa1);
            }
            else
            {
                alarmCodeVar.removeAlarm(0xa1);
            }
            sleep(3);
        }
        else
        {
            alarmCodeVar.removeAlarm(0xa0);
            alarmCodeVar.removeAlarm(0xa1);
        }
        
    }

    if(std::strncmp(pointName,"Agitator1",9) == 0)
    {
        MotorRelPosCtrl(true, en_MOVE, en_Y, -retY, 200.0); //回退50mm

        y_stateData = 0;
        gettimeofday(&start, nullptr);  // 记录开始时间
        y_done = false;
        
        //Y轴执行完毕
        while(!y_done)
        {
            SDORead_ShellFunc(can_sfd, en_Y, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&y_stateData));
    
            if ((y_stateData >> 10) & 0x01)
            {
                y_done = true;
            }
            else
            {
                // 检查是否超时
                gettimeofday(&now, nullptr);
                double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
                if (elapsed >= timeout_seconds) 
                {
                    SDORead_ShellFunc(can_sfd, en_Y, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                    printf("ReadCoordinatesExec中2 的Y轴电流值 = %d\r\n",vol);
                    alarmCodeVar.addAlarm(0xa1);
                    sleep(3);
                }
                else
                {
                    alarmCodeVar.removeAlarm(0xa1);
                }
            }
    
            usleep(100 * 1000);
        }
    }

    //等待Z轴执行完毕
    z_stateData = 0;
    MotorRelPosCtrl(true, en_MOVE, en_Z, z-z_pos, 200.0);
    gettimeofday(&start, nullptr);  // 记录开始时间
    z_done = false;
    //Y轴执行完毕
    while(!z_done)
    {
        SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&z_stateData));

        if ((z_stateData >> 10) & 0x01)
        {
            z_done = true;
        }
        else
        {
            // 检查是否超时
            gettimeofday(&now, nullptr);
            double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
            if (elapsed >= timeout_seconds) 
            {
                SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                printf("ReadCoordinatesExec中3 的Z轴电流值 = %d\r\n",vol);
                alarmCodeVar.addAlarm(0xa2);
                sleep(3);
            }
            else
            {
                alarmCodeVar.removeAlarm(0xa2);
            }
        }

        usleep(100 * 1000);
    }
}


//读取坐标并执行到相应位置
void HSReadCoordinatesExec(const char* pointName)
{
    float x,y,z;
    int32_t x_reso,y_reso,z_reso;

    bool z_done = false, x_done = false, y_done = false;
    struct timeval start, now;
    const int timeout_seconds = 10;  // 超时时间，单位：秒
    int32_t vol;

    uint32_t x_stateData = 0;
    uint32_t y_stateData = 0;
    uint32_t z_stateData = 0;

    read_coordinates(pointName, &x, &y, &z);

    printf("HS x = %f\n",x);
    printf("HS y = %f\n",y);
    printf("HS z = %f\n",z);

	//读取目标绝对位置值
	SDORead_ShellFunc(can_sfd, en_X, 0x6064, 0x00, reinterpret_cast<uint8_t*>(&x_reso));
	usleep(10*1000);

	SDORead_ShellFunc(can_sfd, en_Y, 0x6064, 0x00, reinterpret_cast<uint8_t*>(&y_reso));
	usleep(10*1000);

	SDORead_ShellFunc(can_sfd, en_Z, 0x6064, 0x00, reinterpret_cast<uint8_t*>(&z_reso));
	usleep(10*1000);
    

    //转化为实际距离
    double x_pos = ResoConvPos(en_X, x_reso);
    double y_pos = ResoConvPos(en_Y, y_reso);
    double z_pos = ResoConvPos(en_Z, z_reso);

    x_pos = fabs(x_pos);
    y_pos = fabs(y_pos);
    z_pos = fabs(z_pos);

    MotorRelPosCtrl(true, en_MOVE, en_X, x-x_pos, 500.0);

    MotorRelPosCtrl(true, en_MOVE, en_Y, y-y_pos, 200.0);

    //等待X轴与Y轴执行完毕
    gettimeofday(&start, nullptr);  // 记录开始时间
    while (!(x_done && y_done)) 
    {
        if (!x_done) 
        {
            SDORead_ShellFunc(can_sfd, en_X, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&x_stateData));
            if ((x_stateData >> 10) & 0x01) x_done = true;
        }
        if (!y_done) 
        {
            SDORead_ShellFunc(can_sfd, en_Y, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&y_stateData));
            if ((y_stateData >> 10) & 0x01) y_done = true;
        }
        usleep(100 * 1000);

        // 检查是否超时
        gettimeofday(&now, nullptr);
        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
        if (elapsed >= timeout_seconds) 
        {
            SDORead_ShellFunc(can_sfd, en_X, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
            printf("HSReadCoordinatesExec 1 的X轴电流值 = %d\r\n",vol);
            if(vol>=1790)
            {
                alarmCodeVar.addAlarm(0xa0);
            }
            else
            {
                alarmCodeVar.removeAlarm(0xa0);
            }
            SDORead_ShellFunc(can_sfd, en_Y, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
            printf("HSReadCoordinatesExec 1 的Y轴电流值 = %d\r\n",vol);
            if(vol>=1790)
            {
                alarmCodeVar.addAlarm(0xa1);
            }
            else
            {
                alarmCodeVar.removeAlarm(0xa1);
            }
            sleep(3);
        }
        else
        {
            alarmCodeVar.removeAlarm(0xa0);
            alarmCodeVar.removeAlarm(0xa1);
        }
    }

    //等待Z轴执行完毕
    z_stateData = 0;
    MotorRelPosCtrl(true, en_MOVE, en_Z, z-z_pos-18, 200.0);
    gettimeofday(&start, nullptr);  // 记录开始时间
    z_done = false;
    //Y轴执行完毕
    while(!z_done)
    {
        SDORead_ShellFunc(can_sfd, en_Z, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&z_stateData));

        if ((z_stateData >> 10) & 0x01)
        {
            z_done = true;
        }
        else
        {
            // 检查是否超时
            gettimeofday(&now, nullptr);
            double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1000000.0;
            if (elapsed >= timeout_seconds) 
            {
                SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&vol));
                printf("HSReadCoordinatesExec 2 的Z轴电流值 = %d\r\n",vol);
                alarmCodeVar.addAlarm(0xa2);
                sleep(3);
            }
            else
            {
                alarmCodeVar.removeAlarm(0xa2);
            }
        }

        usleep(100 * 1000);
    }
}

