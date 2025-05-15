#include "InternalProtocol.h"
#include <cstdio>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <cstring>
#include "MainBoardMCU.h"
#include "TypeDefine.h"
#include <stdio.h>
#include "IncubationModule.h"
#include <semaphore.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include "GetStatusBRequest.h"
#include "GetIncubationRequest.h"
#include "GetZboardRequest.h"
#include "main.h"
#include "LiquidSamplingFlow.h"
#include "GetMainBoardRequest.h"


#if 0
UARTHandler delDataHandler;


void* InternalProtocolThreadFunc(void* arg)
{
    IP_sem = sem_open("/InternalProtocol", O_CREAT, 0666, 1);

    while(stop_flag)
    {
        sem_wait(IP_sem);
        delDataHandler.ProcessData();
        sem_post(IP_sem);
        usleep(2*1000);
    }

    printf("内部协议处理线程退出\r\n");

    pthread_exit(NULL);
}
#endif


UARTHandler::UARTHandler() : rxWriteIndex(0), rxReadIndex(0) 
{
    // UART初始化，配置UART并使能接收中断
}

// 组帧函数
std::vector<uint8_t> UARTHandler::AssembleFrame(
    uint8_t mcuAddrLen,
    const std::vector<uint8_t>& mcuAddr,
    uint8_t pcAddrLen,
    uint8_t pcAddr,
    uint8_t cmdCode,
    uint8_t cmdExtCode,
    const std::vector<uint8_t>& data
) {
    std::vector<uint8_t> frame;
    // 帧头
    frame.push_back(FRAME_HEADER_1);
    frame.push_back(FRAME_HEADER_2);

    // MCU地址长度
    frame.push_back(mcuAddrLen);

    // 多级转发地址和目标地址
    if (mcuAddrLen > 1) 
    {
        frame.insert(frame.end(), mcuAddr.begin(), mcuAddr.end());
    } 
    else 
    {
        frame.push_back(mcuAddr[0]); // 目标地址
    }

    // PC地址长度和PC地址
    frame.push_back(pcAddrLen);
    frame.push_back(pcAddr);

    // 命令码和命令扩展码
    frame.push_back(cmdCode);
    frame.push_back(cmdExtCode);

    // 数据长度（大端）
    uint16_t dataLength = data.size();
    frame.push_back((dataLength >> 8) & 0xFF);
    frame.push_back(dataLength & 0xFF);

    // 数据（大端）
    frame.insert(frame.end(), data.begin(), data.end());

    // CRC校验
    AppendCRC16(frame);

    // 帧尾
    frame.push_back(FRAME_TAIL_1);
    frame.push_back(FRAME_TAIL_2);

    return frame;
}

// 处理接收到的数据
void UARTHandler::ProcessData() 
{
    static std::vector<uint8_t> frameBuffer;
    static bool isParsingFrame = false;

    while (!dataQueue.empty()) 
    {
        uint8_t byte = dataQueue.front();
        dataQueue.pop();

        if (!isParsingFrame) 
        {
            if (byte == FRAME_HEADER_1) 
            {
                frameBuffer.clear();
                frameBuffer.push_back(byte);
                isParsingFrame = true;
            }
        } 
        else 
        {
            frameBuffer.push_back(byte);

            if (frameBuffer.size() == 2 && frameBuffer[1] != FRAME_HEADER_2) 
            {
                // 帧头不匹配，重新开始
                isParsingFrame = false;
                frameBuffer.clear();
            } 
            else if (frameBuffer.size() >= 6) 
            {
                // 根据帧结构计算帧的总长度
                size_t frameLength = 0;
                size_t index = 2; // 跳过帧头
                uint8_t mcuAddrLen = frameBuffer[index++];
                if (mcuAddrLen > 1) 
                {
                    index += mcuAddrLen; // 跳过多级转发地址
                    index++;             // 跳过目标地址
                } 
                else 
                {
                    index++; // 跳过目标地址
                }
                index++; // 跳过PC地址长度
                index++; // 跳过PC地址
                index++; // 跳过命令码
                index++; // 跳过命令扩展码

                if (index + 2 > frameBuffer.size()) 
                {
                    continue; // 等待数据长度字段
                }

                uint16_t dataLength = (frameBuffer[index] << 8) | frameBuffer[index + 1];
                index += 2; // 跳过数据长度

                frameLength = index + dataLength + 2 + 2; // 数据+CRC+帧尾
                //printf("frameLength = %d\r\n",frameLength);
                //printf("frameBuffer.size() = %d\r\n",frameBuffer.size());

                if (frameBuffer.size() >= frameLength) 
                {
                    // 检查帧尾
                    if (frameBuffer[frameLength - 2] == FRAME_TAIL_1 &&
                        frameBuffer[frameLength - 1] == FRAME_TAIL_2) 
                    {
                        // 验证CRC并解析帧
                        if (VerifyCRC16(frameBuffer)) 
                        {
                            ParseFrame(frameBuffer);
                        }
                        else
                        {
                            printf("CRC校验未通过\r\n");
                        }
                    }
                    else
                    {
                        printf("帧尾不正确\r\n");
                    }
                    // 重置解析状态
                    isParsingFrame = false;
                    frameBuffer.clear();
                }
            }
        }
    }
}


// 解析帧函数
void UARTHandler::ParseFrame(const std::vector<uint8_t>& frame) 
{
    size_t index = 2; // 跳过帧头

    // MCU地址长度
    uint8_t mcuAddrLen = frame[index++];
    std::vector<uint8_t> mcuAddr;

    // 多级转发地址和目标地址
    if (mcuAddrLen > 1) 
    {
        mcuAddr.insert(mcuAddr.end(), frame.begin() + index, frame.begin() + index + mcuAddrLen);
        index += mcuAddrLen;
        uint8_t targetAddr = frame[index++];
    } 
    else 
    {
        uint8_t targetAddr = frame[index++];
    }

    // PC地址长度和PC地址
    uint8_t pcAddrLen = frame[index++];
    uint8_t pcAddr = frame[index++];

    // 命令码和命令扩展码
    uint8_t cmdCode = frame[index++];
    uint8_t cmdExtCode = frame[index++];

    // 数据长度（大端）
    uint16_t dataLength = (frame[index++] << 8);
    dataLength |= frame[index++];

    // 数据
    std::vector<uint8_t> data(frame.begin() + index, frame.begin() + index + dataLength);
    index += dataLength;

    size_t offset = 0;

    // 根据命令码处理数据
    switch (cmdCode)
    {
        case 0xA0: //GC进入准备
        	if(cmdExtCode == CMD_EXT_READ) //读
        	{

        	}
        	else if(cmdExtCode == CMD_EXT_WRITE) //写
        	{

        	}
            else if(cmdExtCode == CMD_EXT_READ_RESP) //读应答
            {

            }
            else if(cmdExtCode == CMD_EXT_WRITE_RESP) //写应答
            {
                if(data[0] == 1)
        		{
        			printf("GC准备0xA0命令下发成功\r\n");
        		}
                else
                {
                    printf("GC准备0xA0命令下发失败\r\n");
                } 
            }
            break;

        case 0xA1: //GC进入启动
        	if(cmdExtCode == CMD_EXT_READ) //读
        	{

        	}
        	else if(cmdExtCode == CMD_EXT_WRITE) //写
        	{

        	}
            else if(cmdExtCode == CMD_EXT_READ_RESP) //读应答
            {

            }
            else if(cmdExtCode == CMD_EXT_WRITE_RESP) //写应答
            {
        		if(data[0] == 1)
        		{
        			printf("GC启动0xA1命令下发成功\r\n");
        		}
                else
                {
                    printf("GC启动0xA1命令下发失败\r\n");
                }
            }
            break;

        case 0xA2: //GC就绪状态
        	if(cmdExtCode == CMD_EXT_READ) //读
        	{

        	}
            else if(cmdExtCode == CMD_EXT_READ_RESP) //读应答
            {
                
                if(data[0] == 1)
                {
                    TriggerReady.store(1);
                    printf("GC就绪\r\n");
                }
                else
                {
                    TriggerReady.store(0);
                    printf("GC未就绪\r\n");
                }
            }
            else if(cmdExtCode == CMD_EXT_WRITE_RESP) //写应答
            {

            }
            break;

        case 0xA3: //顶空阀控
        	if(cmdExtCode == CMD_EXT_READ) //读
        	{

        	}
        	else if(cmdExtCode == CMD_EXT_WRITE) //写
        	{

        	}
            else if(cmdExtCode == CMD_EXT_READ_RESP) //读应答
            {

            }
            else if(cmdExtCode == CMD_EXT_WRITE_RESP) //写应答
            {
                if(data[0] == 1)
        		{
        			printf("顶空阀控0xA3命令下发成功\r\n");
        		}
                else
                {
                    printf("顶空阀控0xA3命令下发失败\r\n");
                } 
            }
        break;

        case 0x10: 
            if(cmdExtCode == CMD_EXT_READ) //读
            {

            }
            else if(cmdExtCode == CMD_EXT_READ_RESP) //读应答
            {

            }
            else if(cmdExtCode == CMD_EXT_WRITE_RESP) //写应答
            {
                if(data[0] == 1)
        		{
        			printf("0x10命令下发成功\r\n");
        		}
                else
                {
                    printf("0x10命令下发失败\r\n");
                }                
            }
        break;

        case 0x11: //孵化温度应答
            if(cmdExtCode == CMD_EXT_READ) //读
            {

            }
            else if(cmdExtCode == CMD_EXT_READ_RESP) //读应答
            {

            }
            else if(cmdExtCode == CMD_EXT_WRITE_RESP) //写应答
            {
                if(data[0] == 1)
        		{
        			printf("孵化模块0x11命令下发成功\r\n");
        		}
                else
                {
                    printf("孵化模块0x11命令下发失败\r\n");
                }
            }
        break;

        case 0x12: //孵化电机应答
            if(cmdExtCode == CMD_EXT_READ) //读
            {

            }
            else if(cmdExtCode == CMD_EXT_READ_RESP) //读应答
            {

            }
            else if(cmdExtCode == CMD_EXT_WRITE_RESP) //写应答
            {
                if(data[0] == 1)
        		{
        			printf("孵化模块0x12命令下发成功\r\n");
        		}
                else
                {
                    printf("孵化模块0x12命令下发失败\r\n");
                }
            }
        break;

        case 0x13: //孵化模块复位
            if(cmdExtCode == CMD_EXT_READ) //读
            {

            }
            else if(cmdExtCode == CMD_EXT_READ_RESP) //读应答
            {

            }
            else if(cmdExtCode == CMD_EXT_WRITE_RESP) //写应答
            {
                if(data[0] == 1)
        		{
        			printf("孵化模块复位0x13命令下发成功\r\n");
        		}
                else
                {
                    printf("孵化模块复位0x13命令下发失败\r\n");
                }
            }
        break;

        case 0x15: //孵化完成标志位
            if(cmdExtCode == CMD_EXT_READ) //读
            {

            }
            else if(cmdExtCode == CMD_EXT_READ_RESP) //读应答
            {
                if(data[0])
                {
                    incubationComp.store(1);
                    printf("孵化完成\r\n");
                }
                else
                {
                    incubationComp.store(0);
                    printf("孵化未完成\r\n");
                }
            }
            else if(cmdExtCode == CMD_EXT_WRITE_RESP) //写应答
            {

            }
        break;

        case 0xfc: //老化模块心跳包
            if(cmdExtCode == CMD_EXT_READ_RESP) //读应答
            {
                pthread_rwlock_wrlock(&getMainBoardRequestTypeVar_rwlock);
                std::memcpy(&getMainBoardRequestTypeVar.HeatAgitator, &data[offset], sizeof(uint8_t));
                offset += sizeof(uint8_t);
                std::memcpy(&getMainBoardRequestTypeVar.TarTemp, &data[offset], sizeof(float));
                offset += sizeof(float);
                std::memcpy(&getMainBoardRequestTypeVar.Temperature, &data[offset], sizeof(float));
                printf("老化模块温度 %f\r\n", getMainBoardRequestTypeVar.Temperature);
                offset += sizeof(float);
                std::memcpy(&getMainBoardRequestTypeVar.valCtrlFlg, &data[offset], sizeof(uint8_t));
                pthread_rwlock_unlock(&getMainBoardRequestTypeVar_rwlock);   
            }
        break;

        case 0xC0: //进样工具温度应答
            if(cmdExtCode == CMD_EXT_READ) //读
            {

            }
            else if(cmdExtCode == CMD_EXT_READ_RESP) //读应答
            {

            }
            else if(cmdExtCode == CMD_EXT_WRITE_RESP) //写应答
            {
                if(data[0] == 1)
        		{
        			printf("进样工具加热0xC0命令下发成功\r\n");
        		}
                else
                {
                    printf("进样工具加热0xC0命令下发失败\r\n");
                }
            }
        break;

        case 0xfd: //Z轴模块心跳包
            if(cmdExtCode == CMD_EXT_READ_RESP) //读应答
            {
                pthread_rwlock_wrlock(&getZboardRequestTypeVar_rwlock);
                std::memcpy(&getZboardRequestTypeVar.HeatAgitator, &data[offset], sizeof(uint8_t));
                offset += sizeof(uint8_t);
                std::memcpy(&getZboardRequestTypeVar.TarTemp, &data[offset], sizeof(float));
                offset += sizeof(float);
                std::memcpy(&getZboardRequestTypeVar.Temperature, &data[offset], sizeof(float));
                printf("Z轴模块温度 %f\r\n", getZboardRequestTypeVar.Temperature);
                pthread_rwlock_unlock(&getZboardRequestTypeVar_rwlock); 
            }
        break;

        case 0xfe: //孵化模块心跳包
            if(cmdExtCode == CMD_EXT_READ_RESP) //读应答
            {
                pthread_rwlock_wrlock(&getIncubationRequestTypeVar_rwlock);
                std::memcpy(&getIncubationRequestTypeVar.HeatAgitator, &data[offset], sizeof(uint8_t));
                offset += sizeof(uint8_t);
                std::memcpy(&getIncubationRequestTypeVar.TarTemp, &data[offset], sizeof(float));
                offset += sizeof(float);
                std::memcpy(&getIncubationRequestTypeVar.Temperature, &data[offset], sizeof(float));
                printf("孵化模块温度 %f\r\n", getIncubationRequestTypeVar.Temperature);
                offset += sizeof(float);
                std::memcpy(&getIncubationRequestTypeVar.Speed, &data[offset], sizeof(int32_t));
                offset += sizeof(int32_t);
                std::memcpy(&getIncubationRequestTypeVar.agitatorOnTime, &data[offset], sizeof(float));
                offset += sizeof(float);
                std::memcpy(&getIncubationRequestTypeVar.agitatorOffTime, &data[offset], sizeof(float));
                offset += sizeof(float);
                std::memcpy(&getIncubationRequestTypeVar.incubationCompletion, &data[offset], sizeof(uint8_t));
                printf("孵化完成标志 %d\r\n", getIncubationRequestTypeVar.incubationCompletion);
                pthread_rwlock_unlock(&getIncubationRequestTypeVar_rwlock);   
            }
        break;

        default:
            // 未知的命令扩展码
            break;
    }
}

// 计算CRC16校验
uint16_t UARTHandler::CalculateCRC16(const uint8_t* data, size_t length) 
{
    uint16_t crc = 0xFFFF;
    for (size_t pos = 0; pos < length; pos++) 
    {
        crc ^= data[pos];
        for (size_t i = 0; i < 8; i++) 
        {
            if (crc & 0x0001) 
            {
                crc >>= 1;
                crc ^= 0xA001;
            } 
            else 
            {
                crc >>= 1;
            }
        }
    }
    // 返回高低字节交换的CRC值
    return (crc << 8) | (crc >> 8);
}

// 附加CRC16校验到帧尾
void UARTHandler::AppendCRC16(std::vector<uint8_t>& frame) 
{
    uint16_t crc = CalculateCRC16(frame.data(), frame.size());
    frame.push_back((crc >> 8) & 0xFF);
    frame.push_back(crc & 0xFF);
}

// 验证帧的CRC16校验
bool UARTHandler::VerifyCRC16(const std::vector<uint8_t>& frame) 
{
    size_t crcIndex = frame.size() - 4; // CRC开始的位置
    uint16_t receivedCrc = (frame[crcIndex] << 8) | frame[crcIndex + 1];
    uint16_t calculatedCrc = CalculateCRC16(frame.data(), crcIndex);
    return receivedCrc == calculatedCrc;
}


