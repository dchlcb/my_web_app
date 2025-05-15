#include "ZBoardMCU.h"
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
#include <time.h>
#include <sys/ioctl.h>
#include <iomanip>  // 用于 std::setw 和 std::setfill
#include <linux/spi/spidev.h>
#include "Public.h"
#include "InternalProtocol.h"
#include "CANOpen.h"
#include "main.h"
#include "InternalProtocol.h"



int Z_sfd = -1;

// 全局变量
struct timespec zboard_send_time;                   // 上次发送时间
struct timespec zboard_last_receive_time;           // 上次接收时间
pthread_mutex_t zboard_time_mutex = PTHREAD_MUTEX_INITIALIZER;  // 时间变量的互斥锁
const int ZBOARD_TIMEOUT_SECONDS = 60;               // 超时阈值（30秒）


pthread_mutex_t mutexZboardTTYS = PTHREAD_MUTEX_INITIALIZER;


//————————————————————————————————————————————
// 监听线程：负责从串口读取MCU发送过来的数据并处理
//————————————————————————————————————————————
void* ZBoardMCUReceiveThread(void* arg)
{
    UARTHandler delDataHandler;

    while(stop_flag)
    {
        // 定义接收缓冲区
        uint8_t buffer[1024];
        
        ssize_t rlen = read(Z_sfd, buffer, sizeof(buffer));
        
        #if 1
        printf("接收到Z轴模块发送的数据\r\n");
        for(ssize_t i = 0; i < rlen; i++)
        {
            printf("%x ", buffer[i]);
        }
        printf("\r\n");
        #endif

        pthread_mutex_lock(&mutexZboardTTYS);
        // 将读取到的数据逐字节压入队列中
        for (ssize_t i = 0; i < rlen; i++)
        {
            delDataHandler.dataQueue.push(buffer[i]);
        }
        delDataHandler.ProcessData();
        pthread_mutex_unlock(&mutexZboardTTYS);

        #if 0
        pthread_mutex_lock(&serial_mutex);
        printf("接收到孵化模块发送的数据\r\n");
        for(ssize_t i = 0; i < rlen; i++)
        {
            printf("%x ", buffer[i]);
        }
        printf("\r\n");
        pthread_mutex_unlock(&serial_mutex);
        #endif
        

        // 更新接收时间
        pthread_mutex_lock(&zboard_time_mutex);
        clock_gettime(CLOCK_MONOTONIC, &zboard_last_receive_time);
        pthread_mutex_unlock(&zboard_time_mutex);
        
    }

    pthread_mutex_unlock(&zboard_time_mutex);
    printf("Z轴板MCU接收线程退出\r\n");
    pthread_exit(NULL);
}


//Z轴模块MCU通讯线程
void* ZBoardMCUFunc(void* arg)
{
    // 打开 串口4 设备
    Z_sfd = OpenSerialPort("/dev/ttyS4",57600);
    if (Z_sfd < 0)
    {
        perror("Z轴模块无法打开 UART 设备，线程退出");
        pthread_exit(NULL);
    }

    // 初始化共享内存
    shm_unlink("/ZBoard");
    ZB_shmfd = shm_open("/ZBoard", O_CREAT | O_RDWR, 0666);
    ftruncate(ZB_shmfd, sizeof(SharedData));
    ZB_shared = static_cast<SharedData*>(mmap(NULL, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, ZB_shmfd, 0));

    sem_unlink("/SEM_ZBoard");
    ZB_sem = sem_open("/SEM_ZBoard", O_CREAT, 0666, 1);

    pthread_t receiveThread;
    // 创建监听线程
    if(pthread_create(&receiveThread, NULL, ZBoardMCUReceiveThread, NULL) != 0)
    {
        perror("创建Z轴模块MCU监听线程失败");
        close(Z_sfd);
        pthread_exit(NULL);
    }
    
    while(stop_flag)
    {
        sem_wait(ZB_sem);

        if(ZB_shared->dataToMCUReady)
        {
            std::vector<uint8_t> txFrame(ZB_shared->toMCU, ZB_shared->toMCU + ZB_shared->toMCU_size);
            ZB_shared->dataToMCUReady = false;

            sem_post(ZB_sem);
            
            pthread_mutex_lock(&mutexZboardTTYS);
            int ret = write(Z_sfd, txFrame.data(), txFrame.size());
            pthread_mutex_unlock(&mutexZboardTTYS);

            #if 0
            printf("向Z轴模块MCU发送数据\r\n");
            for(int i=0; i<txFrame.size(); i++)
            {
                printf("%x ", txFrame[i]);
            }
            printf("\r\n");
            #endif

            #if 0
            pthread_mutex_lock(&serial_mutex);
            printf("向孵化模块MCU发送数据\r\n");
            for(int i=0; i<txFrame.size(); i++)
            {
                printf("%x ", txFrame[i]);
            }
            printf("\r\n");
            pthread_mutex_unlock(&serial_mutex);
            #endif

            if(ret < 0)
            {
                perror("Z轴MCU串口4发送失败");
            }
            else
            {
                // 记录发送时间
                pthread_mutex_lock(&zboard_time_mutex);
                clock_gettime(CLOCK_MONOTONIC, &zboard_send_time);
                pthread_mutex_unlock(&zboard_time_mutex);
            }

            // 检查超时
            pthread_mutex_lock(&zboard_time_mutex);
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            //double elapsed = (now.tv_sec - zboard_send_time.tv_sec) + (now.tv_nsec - zboard_send_time.tv_nsec) / 1e9;
            double receive_elapsed = (now.tv_sec - zboard_last_receive_time.tv_sec) + (now.tv_nsec - zboard_last_receive_time.tv_nsec) / 1e9;
            pthread_mutex_unlock(&zboard_time_mutex);

            //if(elapsed > ZBOARD_TIMEOUT_SECONDS && receive_elapsed >= elapsed)
            if(receive_elapsed >= ZBOARD_TIMEOUT_SECONDS)
            {
                printf("Z轴模块通讯超时：超过%d秒未收到MCU响应\r\n", ZBOARD_TIMEOUT_SECONDS);
                alarmCodeVar.addAlarm(0x20);
                // 重试发送或记录日志
            }
            else
            {
                alarmCodeVar.removeAlarm(0x20);
            }

        }
        else
        {
            sem_post(ZB_sem);
        }

        usleep(5*1000);
    }

    //结束时释放资源，不然会卡住其他线程
    sem_post(ZB_sem);

    pthread_mutex_unlock(&zboard_time_mutex);


    pthread_join(receiveThread, NULL);

    pthread_mutex_destroy(&mutexZboardTTYS);
    pthread_mutex_destroy(&zboard_time_mutex);

    close(Z_sfd);

    printf("Z轴MCU发送线程退出\r\n");
    pthread_exit(NULL);
}
