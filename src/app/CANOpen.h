#ifndef _CAN_OPEN_H
#define _CAN_OPEN_H



//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>
//#include <linux/can.h>
//#include <linux/can/raw.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/types.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <sys/time.h>
#include <signal.h>
#include <time.h>
#include <iostream>

//调试输出
#define SDO_DEBUG 1


/*========================================================================
*                       全局共享的变量与宏定义
=========================================================================*/


/* 初始化配置并启动CAN设备 */
//int CanDev_Init(const char *dev_name,unsigned int Baulate_CAN);

//设备初始化
void DevInit(void);

/**
 * @brief 打开并初始化串口
 * @param portname 串口设备名，例如 "/dev/ttyS9"
 * @param baudrate 波特率，例如 460800
 * @return -1 表示打开失败，否则返回串口文件描述符
 */
int OpenSerialPort(const char* portname, int baudrate);



/*========================================================================
*                              NMT网络管理
=========================================================================*/
#define NMT_OPT_COMWD       0x01 //操作命令字
#define NMT_STOP_COMWD      0x02 //停止节点命令字
#define NMT_PREOPT_COMWD    0x80 //预操作命令字
#define NMT_RESTSOFT_COMWD  0x81 //复位节点命令字
#define NMT_RESTCOMM_COMWD  0x82 //复位通讯命令字

/* NMT报文发送 */
bool NMT_ShellFunc(const int can_sfd,const int slave_id,const char shell);
/* 同步报文发送 */
bool NVIC_ShellFunc(const int can_sfd);

/*========================================================================
*                              SDO服务
=========================================================================*/
/* SDO写请求报文 */
bool SDOWrite_ShellFunc(const int can_sfd,const int slave_id,const short mindex,const char lindex,const unsigned char *data,const int dlen);
/* SDO读请求报文 */
bool SDORead_ShellFunc(const int can_sfd,const int slave_id,const short mindex,const char lindex,unsigned char *data);

/*========================================================================
*                              PDO服务
=========================================================================*/
/* RPDO映射表配置 */
bool RPDO1_RemapFunc(const int can_sfd,const int slave_id);
bool RPDO2_RemapFunc(const int can_sfd,const int slave_id);
bool RPDO3_RemapFunc(const int can_sfd,const int slave_id);
bool RPDO4_RemapFunc(const int can_sfd,const int slave_id);
/* TPDO映射表配置 */
bool TPDO_RemapFunc(const int can_sfd,const int slave_id);


/* RPDO数据发送 */
bool SendRPDO_Func(const int can_sfd,const int slave_id,const int pdo_id,const unsigned char *data,int dlen);

#if 1
/* TPDO数据接收 */
bool RecvTPDO_Func(const int can_sfd,const int slave_id,const int pdo_id,unsigned char *data);
#endif

/* 同步报文发送 */
bool NVIC_ShellFunc(const int can_sfd);

//失能TPDO
void TPDO_Disable(const int can_sfd,const int slave_id);


//序列化函数
template <typename T>
void serialize(T& data, char* buffer, size_t buffer_size) 
{
    if (sizeof(T) > buffer_size) 
	{
        std::cout << "Data size exceeds buffer capacity" << std::endl;
		return;
    }
    std::memcpy(buffer, &data, sizeof(T));
}

//反序列化函数
template <typename T>
void deserialize(const char* buffer, T& data, size_t buffer_size) 
{
    if (sizeof(T) > buffer_size) 
	{
         std::cout << "Buffer size is smaller than data size" << std::endl;
		 return;
    }
    std::memcpy(&data, buffer, sizeof(T));
}

extern uint8_t RetZeroFlag;
#endif