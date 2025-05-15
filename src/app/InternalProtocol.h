#ifndef _INTERNAL_PROTOCOL_H
#define _INTERNAL_PROTOCOL_H

#include <cstdint>
#include <vector>
#include <queue>
#include <semaphore.h>

// 定义帧常量
#define FRAME_HEADER_1 0x7D
#define FRAME_HEADER_2 0x7B
#define FRAME_TAIL_1   0x7D
#define FRAME_TAIL_2   0x7D

// 命令扩展码
#define CMD_EXT_READ       0x55
#define CMD_EXT_WRITE      0x66
#define CMD_EXT_READ_RESP  0xAA
#define CMD_EXT_WRITE_RESP 0x99

// UART接收缓冲区大小
#define UART_BUFFER_SIZE  1024

class UARTHandler 
{
public:
    UARTHandler();
    void ProcessData();     // 处理接收到的数据
    std::vector<uint8_t> AssembleFrame //组帧函数
    (
        uint8_t mcuAddrLen,
        const std::vector<uint8_t>& mcuAddr,
        uint8_t pcAddrLen,
        uint8_t pcAddr,
        uint8_t cmdCode,
        uint8_t cmdExtCode,
        const std::vector<uint8_t>& data
    );
    void ParseFrame(const std::vector<uint8_t>& frame); // 解析帧函数

     std::queue<uint8_t> dataQueue;

private:
    uint16_t CalculateCRC16(const uint8_t* data, size_t length); //计算CRC校验值
    void AppendCRC16(std::vector<uint8_t>& frame); //计算并附加CRC16校验码到帧尾
    bool VerifyCRC16(const std::vector<uint8_t>& frame); //校验CRC16

    uint8_t uartRxBuffer[UART_BUFFER_SIZE];
    size_t rxWriteIndex;
    size_t rxReadIndex;
   
};

extern uint8_t g_GCReadyFlag;

//extern UARTHandler delDataHandler;

//extern void* InternalProtocolThreadFunc(void* arg);

#endif

