#include "MainBoardMCU.h"
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <iomanip>  // 用于 std::setw 和 std::setfill
#include <linux/spi/spidev.h>
#include <time.h>
#include "Public.h"
#include "InternalProtocol.h"
#include "CANOpen.h"
#include "main.h"



// 全局变量
struct timespec mainboard_send_time;                   // 上次发送时间
struct timespec mainboard_last_receive_time;           // 上次接收时间
pthread_mutex_t mainboard_time_mutex = PTHREAD_MUTEX_INITIALIZER;  // 时间变量的互斥锁

struct timespec aging_send_time;                   // 上次发送时间
struct timespec aging_last_receive_time;           // 上次接收时间
pthread_mutex_t aging_time_mutex = PTHREAD_MUTEX_INITIALIZER;  // 时间变量的互斥锁

const int MAINBOARD_TIMEOUT_SECONDS = 60;               // 超时阈值（30秒）



pthread_mutex_t mutexMainBoardTTYS = PTHREAD_MUTEX_INITIALIZER;

int spi_sfd = -1;




//————————————————————————————————————————————
// 监听线程：负责从串口读取MCU发送过来的数据并处理
//————————————————————————————————————————————
void* MCUReceiveThread(void* arg)
{
    UARTHandler delDataHandler;

    while(stop_flag)
    {
        // 定义接收缓冲区
        uint8_t buffer[1024];

        
        ssize_t rlen = read(spi_sfd, buffer, sizeof(buffer));

        #if 1
        printf("接收到老化模块发送的数据\r\n");
        for(ssize_t i = 0; i < rlen; i++)
        {
            printf("%x ", buffer[i]);
        }
        printf("\r\n");
        #endif
        
        pthread_mutex_lock(&mutexMainBoardTTYS);
        // 将读取到的数据逐字节压入队列中
        for (ssize_t i = 0; i < rlen; i++)
        {
            delDataHandler.dataQueue.push(buffer[i]);
        }
        delDataHandler.ProcessData();
        pthread_mutex_unlock(&mutexMainBoardTTYS);


        #if 0
        printf("接收到主板MCU发送的数据\r\n");
        for(ssize_t i = 0; i < rlen; i++)
        {
            printf("%x ", buffer[i]);
        }
        printf("\r\n");
        #endif

        if(buffer[5] == 1)
        {
            // 更新主板MCU接收时间
            pthread_mutex_lock(&mainboard_time_mutex);
            clock_gettime(CLOCK_MONOTONIC, &mainboard_last_receive_time);
            pthread_mutex_unlock(&mainboard_time_mutex);
        }
        else if(buffer[5] == 4)
        {
            // 更新老化模块接收时间
            pthread_mutex_lock(&aging_time_mutex);
            clock_gettime(CLOCK_MONOTONIC, &aging_last_receive_time);
            pthread_mutex_unlock(&aging_time_mutex);            
        }

        
        
        usleep(1000);
    }

    
    pthread_mutex_unlock(&mainboard_time_mutex);

    printf("主板MCU接收线程退出\r\n");
    pthread_exit(NULL);
}

//主板MCU通讯线程
void* MainBoardMCUFunc(void* arg)
{
    // 打开 串口8 设备
    spi_sfd = OpenSerialPort("/dev/ttyS8",57600);
    if (spi_sfd < 0)
    {
        perror("无法打开 UART 设备，线程退出");
        pthread_exit(NULL);
    }

    // 初始化共享内存
    shm_unlink("/MainBoardMCU");
    
    MB_shmfd = shm_open("/MainBoardMCU", O_CREAT | O_RDWR, 0666);
    ftruncate(MB_shmfd, sizeof(SharedData));
    MB_shared = static_cast<SharedData*>(mmap(NULL, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, MB_shmfd, 0));

    sem_unlink("/SEM_MainBoardMCU");
    MB_sem = sem_open("/SEM_MainBoardMCU", O_CREAT, 0666, 1);

    pthread_t receiveThread;
    // 创建监听线程
    if(pthread_create(&receiveThread, NULL, MCUReceiveThread, NULL) != 0)
    {
        perror("创建主板MCU监听线程失败");
        close(spi_sfd);
        pthread_exit(NULL);
    }
    
    while(stop_flag)
    {
        sem_wait(MB_sem);

        if(MB_shared->dataToMCUReady)
        {
            std::vector<uint8_t> txFrame(MB_shared->toMCU, MB_shared->toMCU + MB_shared->toMCU_size);
            MB_shared->dataToMCUReady = false;

            sem_post(MB_sem);

            pthread_mutex_lock(&mutexMainBoardTTYS);
            int ret = write(spi_sfd, txFrame.data(), txFrame.size());
            
            
            #if 1
            printf("向老化模块发送数据\r\n");
            for(int i=0; i<txFrame.size(); i++)
            {
                printf("%x ", txFrame[i]);
            }
            printf("\r\n");
            #endif

            if(ret < 0)
            {
                perror("主板MCU串口8发送失败");
            }
            else
            {
                if(txFrame[3] == 1)
                {
                    // 记录发送时间
                    pthread_mutex_lock(&mainboard_time_mutex);
                    clock_gettime(CLOCK_MONOTONIC, &mainboard_send_time);
                    pthread_mutex_unlock(&mainboard_time_mutex);
                }
                else if(txFrame[3] == 4)
                {
                    // 记录发送时间
                    pthread_mutex_lock(&aging_time_mutex);
                    clock_gettime(CLOCK_MONOTONIC, &aging_send_time);
                    pthread_mutex_unlock(&aging_time_mutex);
                }
            }

            if(txFrame[3] == 1)
            {
                // 检查超时
                pthread_mutex_lock(&mainboard_time_mutex);
                struct timespec now;
                clock_gettime(CLOCK_MONOTONIC, &now);
                //double elapsed = (now.tv_sec - mainboard_send_time.tv_sec) + (now.tv_nsec - mainboard_send_time.tv_nsec) / 1e9;
                double receive_elapsed = (now.tv_sec - mainboard_last_receive_time.tv_sec) + (now.tv_nsec - mainboard_last_receive_time.tv_nsec) / 1e9;
                pthread_mutex_unlock(&mainboard_time_mutex);

                //if(elapsed > MAINBOARD_TIMEOUT_SECONDS && receive_elapsed >= elapsed)
                if(receive_elapsed >= MAINBOARD_TIMEOUT_SECONDS)
                {
                    printf("主板MCU模块通讯超时：超过%d秒未收到MCU响应\r\n", MAINBOARD_TIMEOUT_SECONDS);
                    alarmCodeVar.addAlarm(0x01);
                    // 可选：重试发送或记录日志
                }
                else
                {
                    alarmCodeVar.removeAlarm(0x01);
                }
            }
            else if(txFrame[3] == 4)
            {
                // 检查超时
                pthread_mutex_lock(&aging_time_mutex);
                struct timespec now;
                clock_gettime(CLOCK_MONOTONIC, &now);
                //double elapsed = (now.tv_sec - mainboard_send_time.tv_sec) + (now.tv_nsec - mainboard_send_time.tv_nsec) / 1e9;
                double receive_elapsed = (now.tv_sec - aging_last_receive_time.tv_sec) + (now.tv_nsec - aging_last_receive_time.tv_nsec) / 1e9;
                pthread_mutex_unlock(&aging_time_mutex);

                //if(elapsed > MAINBOARD_TIMEOUT_SECONDS && receive_elapsed >= elapsed)
                if(receive_elapsed >= MAINBOARD_TIMEOUT_SECONDS)
                {
                    printf("老化模块通讯超时：超过%d秒未收到MCU响应\r\n", MAINBOARD_TIMEOUT_SECONDS);
                    alarmCodeVar.addAlarm(0x03);
                    // 可选：重试发送或记录日志
                }
                else
                {
                    alarmCodeVar.removeAlarm(0x03);
                }
            }

            pthread_mutex_unlock(&mutexMainBoardTTYS);
        }
        else
        {
            sem_post(MB_sem);
        }

        usleep(2*1000);
    }

    sem_post(MB_sem);

    pthread_mutex_unlock(&mainboard_time_mutex);
    
    pthread_join(receiveThread, NULL);

    pthread_mutex_destroy(&mutexMainBoardTTYS);
    pthread_mutex_destroy(&mainboard_time_mutex);

    close(spi_sfd);
    printf("主板MCU发送线程退出\r\n");
    pthread_exit(NULL);
}