#include "IncubationModule.h"
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
#include "InternalProtocol.h"
#include "CANOpen.h"
#include "main.h"


#define SHM_NAME "/shm_incubation"
#define SEM_NAME "/sem_incubation"

// 全局变量
struct timespec incubation_send_time;                   // 上次发送时间
struct timespec incubation_last_receive_time;           // 上次接收时间
pthread_mutex_t incubation_time_mutex = PTHREAD_MUTEX_INITIALIZER;  // 时间变量的互斥锁
const int INCUBATION_TIMEOUT_SECONDS = 60;               // 超时阈值（30秒）


pthread_mutex_t mutexIncubationTTYS = PTHREAD_MUTEX_INITIALIZER;

int IB_sfd = -1;



//————————————————————————————————————————————
// 监听线程：负责从串口读取MCU发送过来的数据并处理
//————————————————————————————————————————————
void* IBBoardMCUReceiveThread(void* arg)
{
    UARTHandler delDataHandler;

    while(stop_flag)
    {
        // 定义接收缓冲区
        uint8_t buffer[1024];
        
        ssize_t rlen = read(IB_sfd, buffer, sizeof(buffer));
        
        #if 1
        printf("接收到孵化模块发送的数据\r\n");
        for(ssize_t i = 0; i < rlen; i++)
        {
            printf("%x ", buffer[i]);
        }
        printf("\r\n");
        #endif

        pthread_mutex_lock(&mutexIncubationTTYS);

        // 将读取到的数据逐字节压入队列中
        for (ssize_t i = 0; i < rlen; i++)
        {
            delDataHandler.dataQueue.push(buffer[i]);
        }
        delDataHandler.ProcessData();

        pthread_mutex_unlock(&mutexIncubationTTYS);


        #if 0
        printf("接收到孵化模块发送的数据\r\n");
        for(ssize_t i = 0; i < rlen; i++)
        {
            printf("%x ", buffer[i]);
        }
        printf("\r\n");
        #endif

        // 更新接收时间
        pthread_mutex_lock(&incubation_time_mutex);
        clock_gettime(CLOCK_MONOTONIC, &incubation_last_receive_time);
        pthread_mutex_unlock(&incubation_time_mutex);
        
    }

    pthread_mutex_unlock(&incubation_time_mutex);
    printf("孵化模块MCU接收线程退出\r\n");
    pthread_exit(NULL);
}


//孵化模块MCU通讯线程
void* IBBoardMCUFunc(void* arg)
{
    // 打开 串口8 设备
    IB_sfd = OpenSerialPort("/dev/ttyS3",57600);
    if (IB_sfd < 0)
    {
        perror("孵化模块无法打开 UART 设备，线程退出");
        pthread_exit(NULL);
    }

    // 初始化共享内存
    shm_unlink(SHM_NAME);
    IB_shmfd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    ftruncate(IB_shmfd, sizeof(SharedData));
    IB_shared = static_cast<SharedData*>(mmap(NULL, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, IB_shmfd, 0));

    sem_unlink(SEM_NAME);
    IB_sem = sem_open(SEM_NAME, O_CREAT, 0666, 1);
    
    pthread_t receiveThread;
    // 创建监听线程
    if(pthread_create(&receiveThread, NULL, IBBoardMCUReceiveThread, NULL) != 0)
    {
        perror("创建孵化模块MCU监听线程失败");
        close(IB_sfd);
        pthread_exit(NULL);
    }

    while(stop_flag)
    {
        sem_wait(IB_sem);

        if(IB_shared->dataToMCUReady)
        {
            std::vector<uint8_t> txFrame(IB_shared->toMCU, IB_shared->toMCU + IB_shared->toMCU_size);
            IB_shared->dataToMCUReady = false;

            sem_post(IB_sem);
            

            pthread_mutex_lock(&mutexIncubationTTYS);
            int ret = write(IB_sfd, txFrame.data(), txFrame.size());
            pthread_mutex_unlock(&mutexIncubationTTYS);

            #if 0
            printf("向孵化模块MCU发送数据\r\n");
            for(int i=0; i<txFrame.size(); i++)
            {
                printf("%x ", txFrame[i]);
            }
            printf("\r\n");
            #endif

            if(ret < 0)
            {
                perror("孵化MCU串口3发送失败");
            }
            else
            {
                // 记录发送时间
                pthread_mutex_lock(&incubation_time_mutex);
                clock_gettime(CLOCK_MONOTONIC, &incubation_send_time);
                pthread_mutex_unlock(&incubation_time_mutex);
            }

            // 检查超时
            pthread_mutex_lock(&incubation_time_mutex);
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            //double elapsed = (now.tv_sec - incubation_send_time.tv_sec) + (now.tv_nsec - incubation_send_time.tv_nsec) / 1e9;
            double receive_elapsed = (now.tv_sec - incubation_last_receive_time.tv_sec) + (now.tv_nsec - incubation_last_receive_time.tv_nsec) / 1e9;
            pthread_mutex_unlock(&incubation_time_mutex);

            //if(elapsed > INCUBATION_TIMEOUT_SECONDS && receive_elapsed >= elapsed)
            if(receive_elapsed >= INCUBATION_TIMEOUT_SECONDS)
            {
                printf("孵化模块通讯超时：超过%d秒未收到MCU响应\r\n", INCUBATION_TIMEOUT_SECONDS);
                alarmCodeVar.addAlarm(0x02);
                // 重试发送或记录日志
            }
            else
            {
                alarmCodeVar.removeAlarm(0x02);
            }

        }
        else
        {
            sem_post(IB_sem);
        }


        usleep(5*1000);
    }

    //结束时释放资源，不然会卡住其他线程
    sem_post(IB_sem);
    pthread_mutex_unlock(&mutexIncubationTTYS);
    pthread_mutex_unlock(&incubation_time_mutex);


    pthread_join(receiveThread, NULL);

    pthread_mutex_destroy(&mutexIncubationTTYS);
    pthread_mutex_destroy(&incubation_time_mutex);

    close(IB_sfd);

    printf("孵化模块MCU发送线程退出\r\n");
    pthread_exit(NULL);
}