#include <sys/ipc.h>
#include <sys/sem.h>
#include "CANOpen.h"
#include "soapProcess.h"
#include "main.h"
#include "MotorCtrl.h"
#include "Public.h"
/*=========================================================================================================================
*                                          电机 轮廓位置 控制模式 
==========================================================================================================================*/ 

uint8_t RetZeroFlag = 0;

/**
 * @brief 打开并初始化串口
 * @param portname 串口设备名，例如 "/dev/ttyS9"
 * @param baudrate 波特率，例如 460800
 * @return -1 表示打开失败，否则返回串口文件描述符
 */
int OpenSerialPort(const char* portname, int baudrate)
{
    int fd = open(portname, O_RDWR | O_NOCTTY);
    if(fd < 0)
    {
        std::perror("open serial port error");
        return -1;
    }

    // 清除串口非阻塞标志，使得 read 可以阻塞
    fcntl(fd, F_SETFL, 0);

    // 配置串口
    struct termios options;
    tcgetattr(fd, &options);

    // 波特率设置
    speed_t speed;
    switch(baudrate)
    {
		case 460800: speed = B460800; break;
        case 115200: speed = B115200; break;
		case 57600:  speed = B57600; break; 
        case 9600:   speed = B9600;   break;
        default:     speed = B115200; break;
    }
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    // 8N1
    options.c_cflag &= ~CSIZE;
    options.c_cflag |=  CS8;       // 8位数据位
    options.c_cflag &= ~PARENB;    // 无校验
    options.c_cflag &= ~CSTOPB;    // 1位停止位
    options.c_iflag &= ~(INPCK | ISTRIP);

	/* Set c_iflag input options */
    options.c_iflag &=~(IXON | IXOFF | IXANY);
    options.c_iflag &=~(INLCR | IGNCR | ICRNL);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // 原始模式，保证我们发送什么就是什么
    options.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag  &= ~OPOST;

	cfmakeraw(&options);

	if(std::strcmp(portname,"/dev/ttyS3") == 0)
	{
		options.c_cc[VMIN]  = 1; //立即返回
		options.c_cc[VTIME] = 0; // 超时5s
	}
	else if(std::strcmp(portname,"/dev/ttyS8") == 0)
	{
		options.c_cc[VMIN]  = 1; //立即返回
		options.c_cc[VTIME] = 0; // 超时5s		
	}
	else if(std::strcmp(portname,"/dev/ttyS4") == 0)
	{
		options.c_cc[VMIN]  = 1; //立即返回
		options.c_cc[VTIME] = 0; // 超时5s		
	}
	else
	{
		options.c_cc[VMIN]  = 0; //立即返回
		options.c_cc[VTIME] = 30; // 超时2s
	}

	
    if(tcsetattr(fd, TCSANOW, &options) != 0)
    {
        std::perror("tcsetattr error");
        close(fd);
        return -1;
    }

    // 清空收发缓冲
    tcflush(fd, TCIOFLUSH);

    return fd;
}

/**
 * @brief 关闭串口
 */
void CloseSerialPort(int fd)
{
    if(fd >= 0)
    {
        close(fd);
    }
}

#if 0
/*========================================================================
*                        功能模块的具体实现
=========================================================================*/
/* 初始化配置并启动CAN设备 */
int CanDev_Init(const char *dev_name,unsigned int Baulate_CAN)
{
	int can_sfd = 0;
	char command[128];
	if(NULL == dev_name)
	{
		printf("Can't init can device is NULL!\n\r");
		can_sfd = -1;
		return can_sfd;
	}
	struct ifreq ifr;
	struct sockaddr_can addr;
	
	/* 创建socket套接字 */
	can_sfd = socket(PF_CAN,SOCK_RAW,CAN_RAW);
	if(0>=can_sfd)
	{
		perror("socket");
		return can_sfd;
	}
	strcpy(ifr.ifr_name,dev_name);     //绑定名字
	ioctl(can_sfd,SIOCGIFINDEX,&ifr);  //指定socket连接的目标设备
	
	/* socket与CAN设备建立连接 */
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	bind(can_sfd,(struct sockaddr*)&addr,sizeof(addr));
	
	/* 关闭回环测试功能 */
	int ret = 0;
	setsockopt(can_sfd,SOL_CAN_RAW,CAN_RAW_LOOPBACK,&ret,sizeof(ret));
	
	/* 配置CAN通信波特率 */
	// 将CAN接口设置为down状态
    //snprintf(command, sizeof(command), "ip link set %s down", dev_name);
    //system(command);
	switch(Baulate_CAN)
	{
		case 1000000u:   //1M
    		snprintf(command, sizeof(command), "ip link set %s type can bitrate %d", dev_name, Baulate_CAN);
			break;
		case 500000u:   //500K
    		snprintf(command, sizeof(command), "ip link set %s type can bitrate %d", dev_name, Baulate_CAN);
			break;
		case 250000u:   //250k
    		snprintf(command, sizeof(command), "ip link set %s type can bitrate %d", dev_name, Baulate_CAN);
			break;
		default:
			break;
	}
	system(command);	

	// 将CAN接口设置为up状态
    snprintf(command, sizeof(command), "ip link set %s up", dev_name);
	sleep(1);
 	if(system(command) != 0)
	{
		std::cout << "error:" << "system(command)" << std::endl;
		return -1;
	}

	return can_sfd;
}
#endif

//设备初始化
void DevInit(void)
{
	const char* sem_name = "/SEM_NAME";  // 信号量名称

 	// 步骤 1：检查信号量是否已经存在
    CANSemID = sem_open(sem_name, 0);  // 尝试打开信号量，不创建
    if (CANSemID != SEM_FAILED) 
	{
        // 信号量存在，执行清除操作
        sem_close(CANSemID);        // 步骤 2：关闭信号量
        sem_unlink(sem_name);       // 步骤 3：删除信号量名称
        printf("残存的信号量已删除。\n");
    } 
	else 
	{
        printf("信号量不存在，无需删除。\n");
    }

    // 步骤 4：重新创建信号量
    CANSemID = sem_open(sem_name, O_CREAT, 0666, 1);
    if (CANSemID == SEM_FAILED) 
	{
        perror("sem_open failed");  // 错误处理
        exit(1) ;
    }
    printf("信号量已成功创建。\n");

    can_sfd = OpenSerialPort("/dev/ttyS9",115200);

    if(can_sfd < 0)
    {
        std::cout << "Open ttyS9 device failed!" <<std::endl;
        exit(1);
    }

	/*电机回零初始化*/
	MotorRetZeroInit();

	//开机启动回零标志完成
	RetZeroFlag = 1;
}


/* NMT报文发送模块 */
bool NMT_ShellFunc(const int can_sfd,const int slave_id,const char shell)
{
	int ret = false;
	int err_count = 0;

	if(0>=can_sfd)
	{
		printf("Please set can device id to nmt mode!\n\r");
		ret = false;
		return ret;
	}

	uint8_t txBuf[1+1+1+1+4+8+1];
	memset(txBuf,0,sizeof(txBuf));

	/* 1 组装写请求报文 */
	txBuf[0] = 0xAA;
	txBuf[1] = 0x00; 
	txBuf[2] = 0x00;
	txBuf[16] = 0x7A;
	//有效数据长度
	txBuf[3] = 0x02;
	//CANID
	txBuf[4] = 0x00;
	txBuf[5] = 0x00;
	txBuf[6] = 0x00;
	txBuf[7] = 0x00;

	txBuf[8] = shell;
	txBuf[9] = slave_id;

	/* 2 发送NMT报文给从站 */
	for(;;)
	{
		ret = write(can_sfd, txBuf, sizeof(txBuf));
		if(0>ret)
		{
			perror("write");
			err_count++;
			if(10<=err_count)
			{
				printf("Can't find slave device at nmt request!\n\r");
				break;
			}
		}
		else
		{
			err_count = 0;
			ret = true;
			break;
		}
	}
	
	return ret;
}

/* 同步报文发送 */
bool NVIC_ShellFunc(const int can_sfd)
{
	int ret = true;
	uint8_t txBuf[1+1+1+1+4+8+1];

	/* 1 组装写请求报文 */
	txBuf[0] = 0xAA;
	txBuf[1] = 0x00; 
	txBuf[2] = 0x00;
	txBuf[16] = 0x7A;

	txBuf[3] = 0x00;

	//CAN ID
	txBuf[4] = 0x00;
	txBuf[5] = 0x00;
	txBuf[6] = 0x00;
	txBuf[7] = 0x80u;

	ret = write(can_sfd,txBuf,sizeof(txBuf));
	if(0>ret)
	{
		perror("NVIC_ShellFunc write");
		ret = false;
	}
	
	return ret;
}

#if 0
/* SDO写请求报文收发模块 */
bool SDOWrite_ShellFunc(const int can_sfd,const int slave_id,const short mindex,const char lindex,const unsigned char *data,const int dlen)
{
	int ret = 0;
	if(0>=can_sfd)
	{
		printf("The can device is error at sdo write!\n\r");
		ret = -1;
		return ret;
	}
	
	uint8_t txBuf[1+1+1+1+4+8+1];
	memset(txBuf, 0, sizeof(txBuf));
	int err_count = 0;
	
	/* 1 组装写请求报文 */
	txBuf[0] = 0xAA;
	txBuf[1] = 0x00; 
	txBuf[2] = 0x00;
	txBuf[16] = 0x7A;

	//CAN ID
	txBuf[4] = 0x00;
	txBuf[5] = 0x00;
	txBuf[6] = 0x06;
	txBuf[7] = slave_id&0xFF;

	//有效数据长度
	if(1u>=dlen)
	{
		txBuf[3u] = 4+4; 
		//txBuf[3u] = 4+1; 
		txBuf[8u] = 0x2fu; //写一个字节
	}
	else if(2u>=dlen)
	{
		txBuf[3u] = 4+4; 
		//txBuf[3u] = 4+2; 
		txBuf[8u] = 0x2bu; //写两个字节
	}	
	else if(3u>=dlen)
	{
		txBuf[3u] = 4+4; 
		//txBuf[3u] = 4+3; 
		txBuf[8u] = 0x27u; //写三个字节
	}
	else if(dlen>=4u)
	{
		txBuf[3u] = 4+4; 
		txBuf[8u] = 0x23u; //写四个字节
	}
		
	txBuf[9u] = mindex&0xffu; //小端字节序
	txBuf[10u] = (mindex>>8u)&0xffu;
	txBuf[11u] = lindex;
	memcpy(&(txBuf[12u]),data,dlen);

#if 0
	printf("Send: ");
	for(uint8_t i=0; i<sizeof(txBuf); i++)
	{
		printf("txBuf[%d] = %x,", i, txBuf[i]);
	}
	printf("\r\n");
#endif

	/* 2 发送请求报文 */
	for(;;)
	{
		sem_wait(CANSemID);
		ret = write(can_sfd, txBuf, sizeof(txBuf));
		sem_post(CANSemID);
		if(0>ret)
		{
			perror("write");
			err_count++;
			if(3<=err_count)
			{
				printf("send sdo write req failed!\n\r");
				ret = false;
				return ret;
			}
		}
		else
		{
			err_count = 0;
			break;
		}
	}
	
	/* 3 阻塞等待接收从站回复 */
	uint8_t rxBuf[1+1+1+1+4+8+1];
	memset(rxBuf, 0, sizeof(rxBuf));

	for(;;)
	{
		sem_wait(CANSemID);
		ret = read(can_sfd, rxBuf, sizeof(rxBuf));
		sem_post(CANSemID);

		if(0<=ret)
		{
			uint16_t canID = (rxBuf[6]<<8) | rxBuf[7];

			if((0x580u+slave_id)==canID)
			{
				#if 0
				printf("can_id:%x\r\n", canID);
				
				for(uint8_t i=0; i<rxBuf[3]; i++)
				{
					printf("rxBuf[%d] = %x, ", i+8, rxBuf[i+8]);
				}
				printf("\r\n");
				#endif

				if(0x80u==rxBuf[8])
				{
					printf("SDO write err:%x %x %x %x\n\r",rxBuf[15u],rxBuf[14u],rxBuf[13u],rxBuf[12u]);	
					ret = false;
					break;
				}
				else
				{
					if(mindex==(rxBuf[9u]|(rxBuf[10u]<<8u)))
					{
						if(lindex==rxBuf[11u])
						{
							ret = true;
							break;
						}
					}
				}
			}
		}

		err_count++;

		if(10<=err_count)
		{
			printf("%d time out to wait slave ask sdo write\n\r",slave_id);

			#if 0
			//电机故障暂停保护
			if(slave_id == en_Z)
			{
				MotorRelPosCtrl(true, en_PAUSE, en_O, 0, 0);
				MotorRelPosCtrl(true, en_PAUSE, en_X, 0, 0);
				MotorRelPosCtrl(true, en_PAUSE, en_Y, 0, 0);
			}
			else if(slave_id == en_O)
			{
				MotorRelPosCtrl(true, en_PAUSE, en_Z, 0, 0);
				MotorRelPosCtrl(true, en_PAUSE, en_X, 0, 0);
				MotorRelPosCtrl(true, en_PAUSE, en_Y, 0, 0);				
			}
			else if(slave_id == en_X)
			{
				MotorRelPosCtrl(true, en_PAUSE, en_O, 0, 0);
				MotorRelPosCtrl(true, en_PAUSE, en_Z, 0, 0);
				MotorRelPosCtrl(true, en_PAUSE, en_Y, 0, 0);				
			}
			else if(slave_id == en_Y)
			{
				MotorRelPosCtrl(true, en_PAUSE, en_O, 0, 0);
				MotorRelPosCtrl(true, en_PAUSE, en_Z, 0, 0);
				MotorRelPosCtrl(true, en_PAUSE, en_X, 0, 0);	
			}
			#endif
			
			ret = false;
			break;
		}

	}
	
	return ret;
}

#else

bool SDOWrite_ShellFunc(const int can_sfd, const int slave_id, const short mindex, const char lindex, const unsigned char *data, const int dlen)
{
    // 检查 CAN 设备句柄是否有效
    if (can_sfd <= 0) 
	{
        printf("The can device is error at sdo write!\n\r");
        return false;
    }

    // 初始化发送缓冲区
    uint8_t txBuf[1+1+1+1+4+8+1];
    memset(txBuf, 0, sizeof(txBuf));
    int retries = 3; // 设置重试次数

    /* 1. 组装写请求报文 */
    txBuf[0] = 0xAA;         // 报文起始标志
    txBuf[1] = 0x00;         // 预留字段
    txBuf[2] = 0x00;         // 预留字段
    txBuf[16] = 0x7A;        // 报文结束标志
    txBuf[4] = 0x00;         // 填充字段
    txBuf[5] = 0x00;         // 填充字段
    txBuf[6] = 0x06;         // 固定值
    txBuf[7] = slave_id & 0xFF; // 从设备 ID

    // 根据数据长度设置命令字节和报文长度
    if (dlen <= 1) 
	{
        txBuf[3] = 4 + 4;    // 报文长度
        txBuf[8] = 0x2f;     // 写一个字节
    } 
	else if (dlen <= 2) 
	{
        txBuf[3] = 4 + 4;
        txBuf[8] = 0x2b;     // 写两个字节
    } 
	else if (dlen <= 3) 
	{
        txBuf[3] = 4 + 4;
        txBuf[8] = 0x27;     // 写三个字节
    } 
	else 
	{
        txBuf[3] = 4 + 4;
        txBuf[8] = 0x23;     // 写四个字节
    }

    // 设置主索引和子索引，并复制数据
    txBuf[9] = mindex & 0xffu;         // 主索引低字节
    txBuf[10] = (mindex >> 8u) & 0xffu;// 主索引高字节
    txBuf[11] = lindex;                // 子索引
    memcpy(&txBuf[12], data, dlen);    // 复制数据到缓冲区

	sem_wait(CANSemID); // 获取信号量

    /* 2. 发送请求报文 */
    int ret = -1;
    for (int i = 0; i < retries; i++) 
	{
        
        ret = write(can_sfd, txBuf, sizeof(txBuf));
        
        if (ret > 0) break; // 发送成功，退出循环
        perror("write");    // 打印错误信息
        usleep(10*1000);      // 等待 10ms 后重试
    }

    if (ret <= 0) 
	{
        printf("send sdo write request err after %d retries!\n\r", retries);
		sem_post(CANSemID); // 释放信号量
        return false;       // 发送失败，返回 false
    }

    /* 3. 监听回复报文 */
    uint8_t rxBuf[1+1+1+1+4+8+1];
    memset(rxBuf, 0, sizeof(rxBuf));
    struct timeval tv;
    fd_set readfds;

    for (int i = 0; i < retries; i++) 
	{
        tv.tv_sec = 5;       // 设置 2 秒超时
        tv.tv_usec = 0;
        FD_ZERO(&readfds);   // 清空文件描述符集
        FD_SET(can_sfd, &readfds); // 添加 CAN 文件描述符

        int rv = select(can_sfd + 1, &readfds, NULL, NULL, &tv);
        if (rv > 0 && FD_ISSET(can_sfd, &readfds)) 
		{
            ret = read(can_sfd, rxBuf, sizeof(rxBuf));

            if (ret >= 0) 
			{
                // 检查 CAN ID 是否匹配
                uint16_t canID = (rxBuf[6] << 8) | rxBuf[7];
                if (canID == (0x580u + slave_id))
				 {
                    // 检查是否有错误返回
                    if (rxBuf[8] == 0x80u) 
					{
                        printf("SDO write err:0x%2x 0x%2x 0x%2x 0x%2x\n\r",
                               rxBuf[15], rxBuf[14], rxBuf[13], rxBuf[12]);
						sem_post(CANSemID); // 释放信号量
                        return false;
                    }
                    // 验证主索引和子索引是否一致
                    if (mindex == (rxBuf[9] | (rxBuf[10] << 8u)) && lindex == rxBuf[11]) 
					{
						if(slave_id == en_X)
						{
							alarmCodeVar.removeAlarm(0x30);
						}
						else if(slave_id == en_Y)
						{
							alarmCodeVar.removeAlarm(0x31);
						}
						else if(slave_id == en_Z)
						{
							alarmCodeVar.removeAlarm(0x32);
						}
						else if(slave_id == en_O)
						{
							alarmCodeVar.removeAlarm(0x33);
						}
						sem_post(CANSemID); // 释放信号量
                        return true; // 写入成功
                    }
                }
            }
        }
        usleep(10*1000); // 等待 10ms 后重试
    }

    printf("%d time out write sdo after %d retries!\n\r", slave_id, retries);
	if(slave_id == en_X)
	{
		alarmCodeVar.addAlarm(0x30);
	}
	else if(slave_id == en_Y)
	{
		alarmCodeVar.addAlarm(0x31);
	}
	else if(slave_id == en_Z)
	{
		alarmCodeVar.addAlarm(0x32);
	}
	else if(slave_id == en_O)
	{
		alarmCodeVar.addAlarm(0x33);
	}
	sem_post(CANSemID); // 释放信号量
    return false; // 超时失败
}


#endif

#if 0
/* SDO读请求报文收发模块 */
bool SDORead_ShellFunc(const int can_sfd,const int slave_id,const short mindex,const char lindex,unsigned char *data)
{
	int ret = 0;
	if(0>=can_sfd)
	{
		printf("The can device is error at sdo read!\n\r");
		ret = -1;
		return ret;
	}

	uint8_t txBuf[1+1+1+1+4+8+1];
	memset(txBuf, 0, sizeof(txBuf));
	int err_count = 0;
	

	/* 1 组转请求报文 */
	txBuf[0] = 0xAA;
	txBuf[1] = 0x00; 
	txBuf[2] = 0x00;
	txBuf[16] = 0x7A;

	//CAN ID
	txBuf[4] = 0x00;
	txBuf[5] = 0x00;
	txBuf[6] = 0x06;
	txBuf[7] = slave_id&0xFF;


	txBuf[3] = 8;//串口发送数据长度

	txBuf[8] = 0x40;
	txBuf[9] = mindex&0xffu;
	txBuf[10] = (mindex>>8u)&0xffu;
	txBuf[11] = lindex;

	sem_wait(CANSemID);
	/* 2 发送请求报文 */
	for(;;)
	{
		ret = write(can_sfd, txBuf, sizeof(txBuf));
		
		if(0>ret)
		{
			perror("write");
			err_count++;
			if(3<=err_count)
			{
				printf("send sdo read request err!\n\r");
				ret = false;
				break;
			}
		}
		else
		{
			#if 0
			printf("Send: ");
			for(uint8_t i=0; i<ret; i++)
			{
				printf("SDORead write txBuf[%d] = %x,", i, txBuf[i]);
			}
			printf("\r\n");
			#endif			
			err_count = 0;
			ret = true;
			break;
		}
	}

	/* 3 监听回复报文 */
	uint8_t rxBuf[1+1+1+1+4+8+1];
	memset(rxBuf, 0, sizeof(rxBuf));
	for(;;)
	{

		ret = read(can_sfd, rxBuf, sizeof(rxBuf));

		if(0<=ret)
		{
			uint16_t canID = (rxBuf[6]<<8) | rxBuf[7];

			if((0x580u+slave_id)==canID)
			{
				#if 0
				printf("can_id:%x\r\n",canID);
				#endif

				uint8_t tempLen = 0;
				//读响应字节
				if(rxBuf[8] == 0x4f) //一个字节
				{
					tempLen = 1;
				}
				else if(rxBuf[8] == 0x4b) //二个字节
				{
					tempLen = 2;
				}
				else if(rxBuf[8] == 0x47) //三个字节
				{
					tempLen = 3;
				}
				else if(rxBuf[8] == 0x43) //四个字节
				{
					tempLen = 4;
				}

				#if 0
				printf("can_dlc:%x\r\n",tempLen);
				printf("can_data:");
				for(int i=0; i<sizeof(rxBuf); i++)
				{
					printf("%x  ", rxBuf[i]);
				}
				printf("\r\n");
				#endif

				if(0x80u==rxBuf[8])
				{
					printf("read sdo err:0x%2x 0x%2x 0x%2x 0x%2x\n\r",
							rxBuf[15],rxBuf[14],rxBuf[13],rxBuf[12]);
					ret = false;
					break;
				}
				else
				{
					if(mindex==(rxBuf[9]|(rxBuf[10]<<8u)))
					{
						if(lindex==rxBuf[11])
						{
							memcpy(data,&(rxBuf[12]),4u);
							ret = true;
							break;
						}
					}
				}
			}
		}
		#if 1
		err_count++;
		if(10<=err_count)
		{
			printf("%d time out read sdo!\n\r",slave_id);
			#if 0
			//电机故障暂停保护
			if(slave_id == en_Z)
			{
				MotorRelPosCtrl(true, en_PAUSE, en_O, 0, 0);
				MotorRelPosCtrl(true, en_PAUSE, en_X, 0, 0);
				MotorRelPosCtrl(true, en_PAUSE, en_Y, 0, 0);
			}
			else if(slave_id == en_O)
			{
				MotorRelPosCtrl(true, en_PAUSE, en_Z, 0, 0);
				MotorRelPosCtrl(true, en_PAUSE, en_X, 0, 0);
				MotorRelPosCtrl(true, en_PAUSE, en_Y, 0, 0);				
			}
			else if(slave_id == en_X)
			{
				MotorRelPosCtrl(true, en_PAUSE, en_O, 0, 0);
				MotorRelPosCtrl(true, en_PAUSE, en_Z, 0, 0);
				MotorRelPosCtrl(true, en_PAUSE, en_Y, 0, 0);				
			}
			else if(slave_id == en_Y)
			{
				MotorRelPosCtrl(true, en_PAUSE, en_O, 0, 0);
				MotorRelPosCtrl(true, en_PAUSE, en_Z, 0, 0);
				MotorRelPosCtrl(true, en_PAUSE, en_X, 0, 0);	
			}
			#endif

			ret = false;
			break;
		}
		#endif
	}
	sem_post(CANSemID);
	return ret;
}

#else 
bool SDORead_ShellFunc(const int can_sfd, const int slave_id, const short mindex, const char lindex, unsigned char *data)
{
    if (can_sfd <= 0) 
	{
        printf("The can device is error at sdo read!\n\r");
        return false;
    }

    uint8_t txBuf[1+1+1+1+4+8+1];
    memset(txBuf, 0, sizeof(txBuf));
    int retries = 10;

    /* 1 组装请求报文 */
    txBuf[0] = 0xAA;
    txBuf[1] = 0x00;
    txBuf[2] = 0x00;
    txBuf[16] = 0x7A;
    txBuf[4] = 0x00;
    txBuf[5] = 0x00;
    txBuf[6] = 0x06;
    txBuf[7] = slave_id & 0xFF;
    txBuf[3] = 8;
    txBuf[8] = 0x40;
    txBuf[9] = mindex & 0xffu;
    txBuf[10] = (mindex >> 8u) & 0xffu;
    txBuf[11] = lindex;

	sem_wait(CANSemID);
    /* 2 发送请求报文 */
    int ret = -1;
    for (int i = 0; i < retries; i++) 
	{
        ret = write(can_sfd, txBuf, sizeof(txBuf));
        if (ret > 0) break;
        perror("write");
        usleep(10*1000);
    }

    if (ret <= 0) 
	{
        printf("send sdo read request err after %d retries!\n\r", retries);
		sem_post(CANSemID);
        return false;
    }

    /* 3 监听回复报文 */
    uint8_t rxBuf[1+1+1+1+4+8+1];
    memset(rxBuf, 0, sizeof(rxBuf));
    struct timeval tv;
    fd_set readfds;

    for (int i = 0; i < retries; i++) 
	{
        tv.tv_sec = 3;
        tv.tv_usec = 0;
        FD_ZERO(&readfds);
        FD_SET(can_sfd, &readfds);

        int rv = select(can_sfd + 1, &readfds, NULL, NULL, &tv);
        if (rv > 0 && FD_ISSET(can_sfd, &readfds)) 
		{
            ret = read(can_sfd, rxBuf, sizeof(rxBuf));

            if (ret >= 0) 
			{
                uint16_t canID = (rxBuf[6] << 8) | rxBuf[7];
                if (canID == (0x580u + slave_id)) 
				{
					#if 0
                    if (rxBuf[8] == 0x80u) 
					{
						
                        printf("read sdo err:0x%2x 0x%2x 0x%2x 0x%2x\n\r",
                               rxBuf[15], rxBuf[14], rxBuf[13], rxBuf[12]);
						sem_post(CANSemID);
                        return false;
                    }
					#endif
                    if (mindex == (rxBuf[9] | (rxBuf[10] << 8u)) && lindex == rxBuf[11]) 
					{
                        memcpy(data, &rxBuf[12], 4u);
						sem_post(CANSemID);
                        return true;
                    }
					else
					{
						sem_post(CANSemID);
						return true;
					}
                }
            }
        }
        usleep(10*1000);
    }

    printf("%d time out read sdo after %d retries!\n\r", slave_id, retries);
	sem_post(CANSemID);
    return false;
}


#endif

/* RPDO1映射表配置 */
bool RPDO1_RemapFunc(const int can_sfd,const int slave_id)
{
	bool ret = true;
	unsigned char shell_data[8u] = {0u};
	
	/* 1 RPDO1的重映射 */
	/* 失能1400H0x01u */
	shell_data[0u]=slave_id;
	shell_data[1u]=0x02u;
	shell_data[2u]=0x00u;
	shell_data[3u]=0x80u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1400u,0x01u,shell_data,4u);
	if(false==ret)
	{
		printf("Disable 1400H RPDO1 failed from %d\n\r",slave_id);
		goto end;
	}

	/* 配置为同步循环的传输类型1400H0x02u */
	shell_data[0u] = 0x1u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1400u,0x02u,shell_data,1u);
	if(false==ret)
	{
		printf("Set 1400H RPDO1 sync mode failed from %d\n\r",slave_id);
		goto end;
	}

	/* 配置周期循环同步最小周期时间：0.1ms */
	shell_data[0u] = 0x1u;
	shell_data[1u] = 0x0u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1400u,0x03u,shell_data,2u);
	if(false==ret)
	{
		printf("Set 1400H RPDO1 sync min time failed from %d\n\r",slave_id);
		goto end;
	}
	
	/* 配置定时器触发时间：10ms */
	shell_data[0u] = 0xau;
	shell_data[1u] = 0x0u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1400u,0x05u,shell_data,2u);
	if(false==ret)
	{
		printf("Set 1400H RPDO1 sync ticktok time failed from %d\n\r",slave_id);
		goto end;
	}
	/* 清除1600H有效映射个数 */
	shell_data[0u] = 0x0u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1600u,0x00u,shell_data,1u);
	if(false==ret)
	{
		printf("Clear 1600H RPDO1 remap failed from %d\n\r",slave_id);
		goto end;
	}
	/* RPDO1映射6040H0x00H控制字 */
	shell_data[0u] = 0x10u;
	shell_data[1u] = 0x00u;
	shell_data[2u] = 0x40u;
	shell_data[3u] = 0x60u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1600u,0x01u,shell_data,4u);
	if(false==ret)
	{
		printf("Remap 1600H RPDO1 is 0x6040H in 0x01H failed from %d\n\r",slave_id);
		goto end;
	}
	/* 确认RPDO1有效映射个数为1 */
	shell_data[0u] = 0x1u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1600u,0x00u,shell_data,1u);
	if(false==ret)
	{
		printf("Set 1600H RPDO1 remap number failed from %d\n\r",slave_id);
		goto end;
	}
	/* 使能RPDO1的通道 */
	shell_data[0u]=slave_id;
	shell_data[1u]=0x02u;
	shell_data[2u]=0x00u;
	shell_data[3u]=0x00u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1400u,0x01u,shell_data,4u);
	if(false==ret)
	{
		printf("Enable 1400H RPDO1 failed from %d\n\r",slave_id);
		goto end;
	}
	
end:	
	return ret;
}

/* RPDO2映射表配置 */
bool RPDO2_RemapFunc(const int can_sfd,const int slave_id)
{
	bool ret = true;
	unsigned char shell_data[8u] = {0u};

	/* 2 RPDO2通道重映射 */
	/* 失能1401H0x01u */
	shell_data[0u]=slave_id;
	shell_data[1u]=0x03u;
	shell_data[2u]=0x00u;
	shell_data[3u]=0x80u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1401u,0x01u,shell_data,4u);
	if(false==ret)
	{
		printf("Disable 1401H RPDO2 failed from %d\n\r",slave_id);
		goto end;
	}
	/* 配置为同步循环的传输类型1401H0x02u */
	shell_data[0u] = 0x1u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1401u,0x02u,shell_data,1u);
	if(false==ret)
	{
		printf("Set 1401H RPDO2 sync mode failed from %d\n\r",slave_id);
		goto end;
	}
	/* 配置周期循环同步最小周期时间：0.1ms */
	shell_data[0u] = 0x1u;
	shell_data[1u] = 0x0u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1401u,0x03u,shell_data,2u);
	if(false==ret)
	{
		printf("Set 1401H RPDO2 sync min time failed from %d\n\r",slave_id);
		goto end;
	}
	/* 配置定时器触发时间：10ms */
	shell_data[0u] = 0xau;
	shell_data[1u] = 0x0u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1401u,0x05u,shell_data,2u);
	if(false==ret)
	{
		printf("Set 1401H RPDO2 sync ticktok time failed from %d\n\r",slave_id);
		goto end;
	}
	/* 清除1601H有效映射个数 */
	shell_data[0u] = 0x0u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1601u,0x00u,shell_data,1u);
	if(false==ret)
	{
		printf("Clear 1601H RPDO2 remap failed from %d\n\r",slave_id);
		goto end;
	}
	/* RPDO2映射6083H到01H */
	shell_data[0u] = 0x20u;
	shell_data[1u] = 0x00u;
	shell_data[2u] = 0x83u;
	shell_data[3u] = 0x60u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1601u,0x01u,shell_data,4u);
	if(false==ret)
	{
		printf("Remap 1601H RPDO2 is 0x6083H in 0x01H failed from %d\n\r",slave_id);
		goto end;
	}
	/* RPDO2映射6084H到02H */
	shell_data[0u] = 0x20u;
	shell_data[1u] = 0x00u;
	shell_data[2u] = 0x84u;
	shell_data[3u] = 0x60u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1601u,0x02u,shell_data,4u);
	if(false==ret)
	{
		printf("Remap 1601H RPDO2 is 0x6084H in 0x02H failed from %d\n\r",slave_id);
		goto end;
	}
	/* 确认RPDO2有效映射个数为2 */
	shell_data[0u] = 0x2u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1601u,0x00u,shell_data,1u);
	if(false==ret)
	{
		printf("Set 1601H RPDO2 remap number failed from %d\n\r",slave_id);
		goto end;
	}
	/* 使能RPDO2的通道 */
	shell_data[0u]=slave_id;
	shell_data[1u]=0x03u;
	shell_data[2u]=0x00u;
	shell_data[3u]=0x00u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1401u,0x01u,shell_data,4u);
	if(false==ret)
	{
		printf("Enable 1401H RPDO2 failed from %d\n\r",slave_id);
		goto end;
	}

end:	
	return ret;	
}

/* RPDO3映射表配置 */
bool RPDO3_RemapFunc(const int can_sfd,const int slave_id)
{
	bool ret = true;
	unsigned char shell_data[8u] = {0u};

	/* 3 RPDO3通道重映射 */
	/* 失能1402H0x01u */
	shell_data[0u]=slave_id;
	shell_data[1u]=0x04u;
	shell_data[2u]=0x00u;
	shell_data[3u]=0x80u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1402u,0x01u,shell_data,4u);
	if(false==ret)
	{
		printf("Disable 1402H RPDO3 failed from %d\n\r",slave_id);
		goto end;
	}

	/* 配置为同步循环的传输类型1402H0x02u */
	shell_data[0u] = 0x1u;
	shell_data[1u] = 0x00u;
	shell_data[2u] = 0x00u;
	shell_data[3u] = 0x00u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1402u,0x02u,shell_data,4u);
	if(false==ret)
	{
		printf("Set 1402H RPDO3 sync mode failed from %d\n\r",slave_id);
		goto end;
	}

#if 0
	/* 配置周期循环同步最小周期时间：0.1ms */
	shell_data[0u] = 0x1u;
	shell_data[1u] = 0x0u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1402u,0x03u,shell_data,2u);
	if(false==ret)
	{
		printf("Set 1402H RPDO3 sync min time failed from %d\n\r",slave_id);
		goto end;
	}

	/* 配置定时器触发时间：10ms */
	shell_data[0u] = 0xau;
	shell_data[1u] = 0x0u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1402u,0x05u,shell_data,2u);
	if(false==ret)
	{
		printf("Set 1402H RPDO3 sync ticktok time failed from %d\n\r",slave_id);
		goto end;
	}
#endif	

	/* 清除1602H有效映射个数 */
	shell_data[0u] = 0x0u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1602u,0x00u,shell_data,1u);
	if(false==ret)
	{
		printf("Clear 1602H RPDO3 remap failed from %d\n\r",slave_id);
		goto end;
	}

	/* RPDO3映射607AH到01H */
	/*立迈胜电机映射目标位置*/
	shell_data[0u] = 0x20u;
	shell_data[1u] = 0x00u;
	shell_data[2u] = 0x7au;
	shell_data[3u] = 0x60u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1602u,0x01u,shell_data,4u);
	if(false==ret)
	{
		printf("Remap 1602H RPDO3 is 0x607AH in 0x01H failed from %d\n\r",slave_id);
		goto end;
	}

	/* RPDO3映射6081H到02H */
	/*立迈胜电机映射速度*/
	shell_data[0u] = 0x20u;
	shell_data[1u] = 0x00u;
	shell_data[2u] = 0x81u;
	shell_data[3u] = 0x60u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1602u,0x02u,shell_data,4u);
	if(false==ret)
	{
		printf("Remap 1602H RPDO3 is 0x6081H in 0x02H failed from %d\n\r",slave_id);
		goto end;
	}

	/* 确认RPDO3有效映射个数为2 */
	shell_data[0u] = 0x2u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1602u,0x00u,shell_data,1u);
	if(false==ret)
	{
		printf("Set 1602H RPDO3 remap number failed from %d\n\r",slave_id);
		goto end;
	}

	/* 使能RPDO3的通道 */
	shell_data[0u]=slave_id;
	shell_data[1u]=0x04u;
	shell_data[2u]=0x00u;
	shell_data[3u]=0x00u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1402u,0x01u,shell_data,4u);
	if(false==ret)
	{
		printf("Enable 1402H RPDO3 failed from %d\n\r",slave_id);
		goto end;
	}
	
end:	
	return ret;	
}

/* RPDO4映射表配置 */
bool RPDO4_RemapFunc(const int can_sfd,const int slave_id)
{
	bool ret = true;
	unsigned char shell_data[8u] = {0u};

	/* 3 RPDO4通道重映射 */
	/* 失能1403H0x01u */
	shell_data[0u]=slave_id;
	shell_data[1u]=0x05u;
	shell_data[2u]=0x00u;
	shell_data[3u]=0x80u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1403u,0x01u,shell_data,4u);
	if(false==ret)
	{
		printf("Disable 1402H RPDO3 failed from %d\n\r",slave_id);
		goto end;
	}
	/* 配置为同步循环的传输类型1403H0x02u */
	shell_data[0u] = 0x1u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1403u,0x02u,shell_data,1u);
	if(false==ret)
	{
		printf("Set 1402H RPDO3 sync mode failed from %d\n\r",slave_id);
		goto end;
	}
	/* 配置周期循环同步最小周期时间：0.1ms */
	shell_data[0u] = 0x1u;
	shell_data[1u] = 0x0u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1402u,0x03u,shell_data,2u);
	if(false==ret)
	{
		printf("Set 1402H RPDO3 sync min time failed from %d\n\r",slave_id);
		goto end;
	}
	/* 配置定时器触发时间：10ms */
	shell_data[0u] = 0xau;
	shell_data[1u] = 0x0u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1402u,0x05u,shell_data,2u);
	if(false==ret)
	{
		printf("Set 1402H RPDO3 sync ticktok time failed from %d\n\r",slave_id);
		goto end;
	}
	/* 清除1602H有效映射个数 */
	shell_data[0u] = 0x0u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1602u,0x00u,shell_data,1u);
	if(false==ret)
	{
		printf("Clear 1602H RPDO3 remap failed from %d\n\r",slave_id);
		goto end;
	}
	/* RPDO3映射607AH到01H */
	shell_data[0u] = 0x20u;
	shell_data[1u] = 0x00u;
	shell_data[2u] = 0x7au;
	shell_data[3u] = 0x60u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1602u,0x01u,shell_data,4u);
	if(false==ret)
	{
		printf("Remap 1602H RPDO3 is 0x607AH in 0x01H failed from %d\n\r",slave_id);
		goto end;
	}
	/* RPDO3映射6081H到02H */
	shell_data[0u] = 0x20u;
	shell_data[1u] = 0x00u;
	shell_data[2u] = 0x81u;
	shell_data[3u] = 0x60u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1602u,0x02u,shell_data,4u);
	if(false==ret)
	{
		printf("Remap 1602H RPDO3 is 0x6081H in 0x02H failed from %d\n\r",slave_id);
		goto end;
	}
	/* 确认RPDO3有效映射个数为2 */
	shell_data[0u] = 0x2u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1602u,0x00u,shell_data,1u);
	if(false==ret)
	{
		printf("Set 1602H RPDO3 remap number failed from %d\n\r",slave_id);
		goto end;
	}
	/* 使能RPDO3的通道 */
	shell_data[0u]=slave_id;
	shell_data[1u]=0x05u;
	shell_data[2u]=0x00u;
	shell_data[3u]=0x00u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1403u,0x01u,shell_data,4u);
	if(false==ret)
	{
		printf("Enable 1403H RPDO4 failed from %d\n\r",slave_id);
		goto end;
	}
	
end:	
	return ret;	
}

/* TPDO映射表配置 */
bool TPDO_RemapFunc(const int can_sfd,const int slave_id)
{
	bool ret = true;
	unsigned char shell_data[8u] = {0u};
	
	/* 1 配置TPDO1映射通道 */
	/* 关闭TPDO1通道映射 */
	shell_data[0u] = 0x80u+slave_id;
	shell_data[1u] = 0x01u;
	shell_data[2u] = 0x00u;
	shell_data[3u] = 0xc0u;  //禁止远程触发该TPDO
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1800u,0x01u,shell_data,4u);
	if(false==ret)
	{
		printf("Disable 1800H TPDO1 failed to %d\n\r",slave_id);
		goto end;
	}
	/* 配置TPDO1为同步循环传输类型 */
	shell_data[0u] = 0x1u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1800u,0x02u,shell_data,1u);
	if(false==ret)
	{
		printf("Set 1800H TPDO1 sync mode failed to %d\n\r",slave_id);
		goto end;
	}
	/* 配置同步周期最小时间：0.1ms */
	shell_data[0u] = 0x1u;
	shell_data[1u] = 0x0u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1800u,0x03u,shell_data,2u);
	if(false==ret)
	{
		printf("Set 1800H TPDO1 sync min time failed to %d\n\r",slave_id);
		goto end;
	}
	/* 配置同步周期时间：10ms */
	shell_data[0u] = 0xau;
	shell_data[1u] = 0x0u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1800u,0x05u,shell_data,2u);
	if(false==ret)
	{
		printf("Set 1800H TPDO1 sync time failed to %d\n\r",slave_id);
		goto end;
	}
	/* 清除TPDO1的有效映射个数 */
	shell_data[0u] = 0x0u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1A00u,0x00u,shell_data,1u);
	if(false==ret)
	{
		printf("Clear 1a00H TPDO1 remap failed to %d\n\r",slave_id);
		goto end;
	}
	/* 配置TPDO1映射6041H到01H */
	shell_data[0u] = 0x10u;
	shell_data[1u] = 0x00u;
	shell_data[2u] = 0x41u;
	shell_data[3u] = 0x60u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1A00u,0x01u,shell_data,4u);
	if(false==ret)
	{
		printf("Set 1a00H TPDO1 remap 6041H in 01H failed to %d\n\r",slave_id);
		goto end;
	}
	/* 配置TPDO1有效映射个数为1 */
	shell_data[0u] = 0x1u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1A00u,0x00u,shell_data,1u);
	if(false==ret)
	{
		printf("Set 1a00H TPDO1 remap number falied to %d\n\r",slave_id);
		goto end;
	}
	/* 使能TPDO1通道 */
	shell_data[0u] = 0x80u+slave_id;
	shell_data[1u] = 0x01u;
	shell_data[2u] = 0x00u;
	shell_data[3u] = 0x40u;  //禁止远程触发该TPDO
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1800u,0x01u,shell_data,4u);
	if(false==ret)
	{
		printf("Enable 1800H TPDO1 failed to %d\n\r",slave_id);
		goto end;
	}
	
	/* 2 配置TPDO2通道映射 */
	/* 关闭TPDO2通道映射 */
	shell_data[0u] = 0x80u+slave_id;
	shell_data[1u] = 0x02u;
	shell_data[2u] = 0x00u;
	shell_data[3u] = 0xc0u;  //禁止远程触发该TPDO
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1801u,0x01u,shell_data,4u);
	if(false==ret)
	{
		printf("Disable 1801H TPDO2 failed to %d\n\r",slave_id);
		goto end;
	}
	/* 配置TPDO2为同步循环传输类型 */
	shell_data[0u] = 0x1u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1801u,0x02u,shell_data,1u);
	if(false==ret)
	{
		printf("Set 1801H TPDO2 sync mode failed to %d\n\r",slave_id);
		goto end;
	}
	/* 配置同步周期最小时间：0.1ms */
	shell_data[0u] = 0x1u;
	shell_data[1u] = 0x0u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1801u,0x03u,shell_data,2u);
	if(false==ret)
	{
		printf("Set 1801H TPDO2 sync min time failed to %d\n\r",slave_id);
		goto end;
	}
	/* 配置同步周期时间：10ms */
	shell_data[0u] = 0xau;
	shell_data[1u] = 0x0u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1801u,0x05u,shell_data,2u);
	if(false==ret)
	{
		printf("Set 1801H TPDO2 sync time failed to %d\n\r",slave_id);
		goto end;
	}
	/* 清除TPDO2的有效映射个数 */
	shell_data[0u] = 0x0u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1A01u,0x00u,shell_data,1u);
	if(false==ret)
	{
		printf("Clear 1a01H TPDO2 remap failed to %d\n\r",slave_id);
		goto end;
	}
	/* 配置TPDO2映射6064H到01H */
	shell_data[0u] = 0x20u;
	shell_data[1u] = 0x00u;
	shell_data[2u] = 0x64u;
	shell_data[3u] = 0x60u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1A01u,0x01u,shell_data,4u);
	if(false==ret)
	{
		printf("Set 1a01H TPDO2 remap 6064H in 01H failed to %d\n\r",slave_id);
		goto end;
	}
	/* 配置TPDO2映射606CH到02H */
	shell_data[0u] = 0x20u;
	shell_data[1u] = 0x00u;
	shell_data[2u] = 0x6cu;
	shell_data[3u] = 0x60u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1A01u,0x02u,shell_data,4u);
	if(false==ret)
	{
		printf("Set 1a01H TPDO2 remap 606CH in 02H failed to %d\n\r",slave_id);
		goto end;
	}
	/* 配置TPDO2有效映射个数为2 */
	shell_data[0u] = 0x2u;
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1A01u,0x00u,shell_data,1u);
	if(false==ret)
	{
		printf("Set 1a01H TPDO2 remap number falied to %d\n\r",slave_id);
		goto end;
	}
	/* 使能TPDO2通道 */
	shell_data[0u] = 0x80u+slave_id;
	shell_data[1u] = 0x02u;
	shell_data[2u] = 0x00u;
	shell_data[3u] = 0x40u;  //禁止远程触发该TPDO
	ret = SDOWrite_ShellFunc(can_sfd,slave_id,0x1801u,0x01u,shell_data,4u);
	if(false==ret)
	{
		printf("Enable 1801H TPDO2 failed to %d\n\r",slave_id);
		goto end;
	}
	
end:	
	return ret;
}


//失能TPDO
void TPDO_Disable(const int can_sfd,const int slave_id)
{
	/* 关闭TPDO通道映射 */
	uint32_t data = 0x80000180+slave_id;
	SDOWrite_ShellFunc(can_sfd,slave_id,0x1800u,0x01u,reinterpret_cast<uint8_t*>(&data),4u);
	usleep(10*1000);
	
	data = 0x80000280+slave_id;
	SDOWrite_ShellFunc(can_sfd,slave_id,0x1801u,0x01u,reinterpret_cast<uint8_t*>(&data),4u);
	usleep(10*1000);

	data = 0x80000380+slave_id;
	SDOWrite_ShellFunc(can_sfd,slave_id,0x1802u,0x01u,reinterpret_cast<uint8_t*>(&data),4u);
	usleep(10*1000);

	data = 0x80000480+slave_id;
	SDOWrite_ShellFunc(can_sfd,slave_id,0x1803u,0x01u,reinterpret_cast<uint8_t*>(&data),4u);
	usleep(10*1000);
}


/* RPDO数据发送 */
bool SendRPDO_Func(const int can_sfd,const int slave_id,const int pdo_id,const unsigned char *data, int dlen)
{
	int ret = true;
	int err_count = 0;

	uint8_t txBuf[1+1+1+1+4+8+1];
	memset(txBuf,0,sizeof(txBuf));

	/* 1 组装写请求报文 */
	txBuf[0] = 0xAA;
	txBuf[1] = 0x00; 
	txBuf[2] = 0x00;
	txBuf[16] = 0x7A;

	/* 1 组转数据报文 */
	//CAN ID
	switch(pdo_id)
	{
		case 1:
		{
			txBuf[4] = 0x00;
			txBuf[5] = 0x00;
			txBuf[6] = 0x02;
			txBuf[7] = slave_id;
			break;
		}
		case 2:
		{
			txBuf[4] = 0x00;
			txBuf[5] = 0x00;
			txBuf[6] = 0x03;
			txBuf[7] = slave_id;
			break;
		}
		case 3:
		{
			txBuf[4] = 0x00;
			txBuf[5] = 0x00;
			txBuf[6] = 0x04;
			txBuf[7] = slave_id;
			break;
		}
		case 4:
		{
			txBuf[4] = 0x00;
			txBuf[5] = 0x00;
			txBuf[6] = 0x05;
			txBuf[7] = slave_id;
			break;
		}
	}

	//有效数据长度
	if(1u>=dlen)
	{
		txBuf[3u] = 1; 
	}
	else if(2u>=dlen)
	{
		txBuf[3u] = 2; 
	}
	else if(3u>=dlen)
	{
		txBuf[3u] = 3; 
	}	
	else if(4u>=dlen)
	{
		txBuf[3u] = 4; 
	}
	else if(8u>=dlen)
	{
		txBuf[3u] = 8; 
	}

	memcpy(&txBuf[8],data,dlen);

	/* 2 发送RPDO报文 */
	for(;;)
	{
		sem_wait(CANSemID);
		ret = write(can_sfd, txBuf, sizeof(txBuf));
		
		if(0>ret)
		{
			perror("write");
			err_count++;
			if(10<=err_count)
			{
				printf("Send rpdo shell to %d error!\n\r",slave_id);
				ret = false;
				sem_post(CANSemID);
				goto end;
			}
		}
		else
		{
			
			err_count = 0;
			ret = true;
			sem_post(CANSemID);
			goto end;
		}
	}
	
end:	
	return ret;
}

#if 1
/* TPDO数据接收 */
bool RecvTPDO_Func(const int can_sfd,const int slave_id,const int pdo_id,unsigned char *data)
{
	int ret = 0;
	uint8_t rxBuf[1+1+1+1+4+8+1];
	int err_count = 0;
	memset(rxBuf, 0, sizeof(rxBuf));
	/* 接收TPDO报文 */
	for(;;)
	{
		ret = read(can_sfd, rxBuf,sizeof(rxBuf));

		for(int i=0; i<sizeof(rxBuf); i++)
		{
			printf("%x ",rxBuf[i]);
		}
		printf("\n");
		
		if(0>ret)
		{
			perror("read");
			err_count++;
			if(3<=err_count)
			{
				printf("Recv TPDO from %d error!\n\r",slave_id);
				ret = false;
				goto end;
			}
		}
		else
		{
			uint16_t canID = (rxBuf[6]<<8) | rxBuf[7];

			switch(pdo_id)
			{
				case 1:
				{
					if((0x180u+slave_id)==canID)
					{
						memcpy(data,&rxBuf[8],rxBuf[3]);
						ret = true;
						goto end;
					}
					break;
				}
				case 2:
				{
					if((0x280u+slave_id)==canID)
					{
						memcpy(data,&rxBuf[8],rxBuf[3]);
						ret = true;
						goto end;
					}
					break;
				}
				case 3:
				{
					if((0x380u+slave_id)==canID)
					{
						memcpy(data,&rxBuf[8],rxBuf[3]);
						ret = true;
						goto end;
					}
					break;
				}
				case 4:
				{
					if((0x480u+slave_id)==canID)
					{
						memcpy(data,&rxBuf[8],rxBuf[3]);
						ret = true;
						goto end;
					}
					break;
				}
			}
		}
	}
	
end:	
	return ret;
}

#endif




