#include <sys/ipc.h>
#include <sys/sem.h>
#include <fcntl.h>       // fcntl
#include <stdio.h>    // 标准输入输出
#include "main.h"
#include "CANOpen.h"
#include "soapProcess.h"
#include "ExcuteActions.h"
#include "GetStatusBRequest.h"
#include "Calibrations.h"
#include "LiquidSamplingFlow.h"
#include "MainBoardMCU.h"
#include "IncubationModule.h"
#include "ZBoardMCU.h"
#include "InternalProtocol.h"
#include "GetIncubationRequest.h"
#include "GetZboardRequest.h"
#include "GetMainBoardRequest.h"
#include <sys/resource.h>
#include <dirent.h>

volatile uint8_t stop_flag = 1;

#define PORT 64000

AlarmManager alarmCodeVar;

//共享资源
//CAN信号量
sem_t* CANSemID = NULL;
int can_sfd = -1;

int msgExcuteActionsID = -1; // 创建电机控制消息队列ID
// 创建消息队列ID
int msgCalibrationsID = -1;  //创建校准消息队列ID
// 创建消息队列ID
int msgLiquidSamplingFlowID; //创建流程消息队列ID

// 初始化信号量用于同步
//sem_t* IP_sem = NULL; //创建内部协议处理信号量

//主板MCU共享内存资源创建
int MB_shmfd = -1; // 初始化共享内存
SharedData* MB_shared = NULL;
sem_t* MB_sem = NULL;// 初始化信号量用于同步

//孵化MCU共享内存资源创建
int IB_shmfd = -1;
SharedData* IB_shared = NULL;
// 初始化信号量用于同步
sem_t* IB_sem = NULL;

//Z轴MCU共享内存资源创建
int ZB_shmfd = -1;
SharedData* ZB_shared = NULL;
// 初始化信号量用于同步
sem_t* ZB_sem = NULL;

//电机状态条件变量互斥锁
pthread_mutex_t mutexGetStaBReq = PTHREAD_MUTEX_INITIALIZER;

//孵化模块状态条件变量互斥锁
pthread_mutex_t mutexGetIncubationReq = PTHREAD_MUTEX_INITIALIZER;
//孵化模块状态变量互斥锁
pthread_mutex_t mutexGetIncubationReqVar = PTHREAD_MUTEX_INITIALIZER;
pthread_rwlock_t getIncubationRequestTypeVar_rwlock = PTHREAD_RWLOCK_INITIALIZER;


//老化模块状态条件变量互斥锁
pthread_mutex_t mutexGetMainBoardReq = PTHREAD_MUTEX_INITIALIZER;
//老化模块状态变量互斥锁
pthread_mutex_t mutexMainBoardReqVar = PTHREAD_MUTEX_INITIALIZER;
pthread_rwlock_t getMainBoardRequestTypeVar_rwlock = PTHREAD_RWLOCK_INITIALIZER;


//Z轴模块状态条件变量互斥锁
pthread_mutex_t mutexGetZboardReq = PTHREAD_MUTEX_INITIALIZER;
//Z轴模块状态变量互斥锁
pthread_mutex_t mutexGetZboardReqVar = PTHREAD_MUTEX_INITIALIZER;
pthread_rwlock_t getZboardRequestTypeVar_rwlock = PTHREAD_RWLOCK_INITIALIZER;

//客户端socket
volatile uint32_t socketCount = 0;
int clientSocket = -1;
pthread_mutex_t mutexClientSocket = PTHREAD_MUTEX_INITIALIZER;


pthread_mutex_t serial_mutex = PTHREAD_MUTEX_INITIALIZER;


/* 统计 /proc/self/fd 个数（去掉 . 和 ..） */
static int count_open_fd(void)
{
    long n = 0;
    DIR *d = opendir("/proc/self/fd");
    if (!d) return -1;
    while (readdir(d)) ++n;
    closedir(d);
    return (int)(n - 2);
}

/* 线程函数：每 5 秒打印一次已用 / 上限 */
void *fd_usage_thread(void *arg)
{
    (void)arg;                               /* 未用参数 */

    /* 读取软上限 (ulimit -n) */
    struct rlimit lim;
    if (getrlimit(RLIMIT_NOFILE, &lim) == -1) 
    {
        perror("[FD] getrlimit");
        return NULL;
    }
    int soft = (int)lim.rlim_cur;

    for (;;) 
    {
        int used = count_open_fd();
        if (used >= 0) 
        {
            int pct = used * 100 / soft;
            fprintf(stderr, "[FD] %d / %d  (%d%%)%s\n",
                    used, soft, pct,
                    pct >= 90 ? "  <== WARNING" : "");
            fflush(stderr);
        }
        sleep(5);                            /* 周期：5 秒 */
    }
    return NULL;                             /* 永不返回 */
}



// 线程执行的函数，用于监控内存使用情况
void* memory_monitor(void* arg) 
{
    pid_t pid = getpid();  // 获取当前进程的 PID
    char path[32];
    int32_t tmpData;
    sprintf(path, "/proc/%d/status", pid);  // 构造 /proc/[pid]/status 文件路径

    while (stop_flag) 
    {
        FILE* fp = fopen(path, "r");  // 打开状态文件
        if (fp) 
        {
            char line[256];
            long vm_size = 0, vm_rss = 0, vm_data = 0; // 用于存储解析出的值 (kB)

            // 逐行读取文件内容
            while (fgets(line, sizeof(line), fp)) 
            {
                // 解析包含内存信息的行
                if (strstr(line, "VmSize")) 
                {
                    sscanf(line, "VmSize: %ld kB", &vm_size); // 提取 VmSize 值
                }
                else if (strstr(line, "VmRSS")) 
                {
                    sscanf(line, "VmRSS: %ld kB", &vm_rss);   // 提取 VmRSS 值
                }
                else if (strstr(line, "VmData")) 
                {
                    sscanf(line, "VmData: %ld kB", &vm_data); // 提取 VmData 值
                }
            }
            fclose(fp);  // 关闭文件

            // 转换为 MB 并打印
            //pthread_mutex_lock(&serial_mutex);
            //printf("Memory Usage:\n");
            //printf("  Virtual Memory (VmSize): %.2f MB\n", vm_size / 1024.0);
            printf("  Physical Memory (VmRSS): %.2f MB\n", vm_rss / 1024.0);


            SDORead_ShellFunc(can_sfd, en_Z, 0x1003, 0x01, reinterpret_cast<uint8_t*>(&tmpData));
            printf("Z轴故障值 = %x\r\n",tmpData);
            usleep(15*1000);
        
            SDORead_ShellFunc(can_sfd, en_Y, 0x1003, 0x01, reinterpret_cast<uint8_t*>(&tmpData));
            printf("Y轴故障值 = %x\r\n",tmpData);
            usleep(15*1000);
        
            SDORead_ShellFunc(can_sfd, en_X, 0x1003, 0x01, reinterpret_cast<uint8_t*>(&tmpData));
            printf("X轴故障值 = %x\r\n",tmpData);
            usleep(15*1000);

            //printf("  Physical Memory (VmRSS): %d KB\n", vm_rss);
            //printf("  Data Segment (VmData): %.2f MB\n", vm_data / 1024.0);
            //pthread_mutex_unlock(&serial_mutex);
        } 
        else 
        {
            perror("打开 /proc/[pid]/status 失败");
        }

        sleep(10);  // 每隔 10 秒检查一次
    }

    return NULL;
}


// http通信处理
void *soapCommunication(void *arg) 
{
    int clientSocketTemp = -1;
   // 创建 TCP 套接字
    int serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket == -1)
    {
        std::cerr << "Error creating socket" << std::endl;
        return NULL;
    }

    // 设置 SO_REUSEADDR
    int opt = 1;
    if (setsockopt(serverSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) == -1) 
    {
        perror("setsockopt failed");
        close(serverSocket);
        pthread_exit(NULL);
    }

    struct sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;  // 监听所有可用的接口
    serverAddr.sin_port = htons(PORT);        // 监听指定端口

    // 绑定套接字
    if (bind(serverSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) == -1)
    {
        std::cerr << "Error binding socket" << std::endl;
        close(serverSocket);
        return NULL;
    }

    // 监听连接
    if (listen(serverSocket, 10) == -1)
    {
        std::cerr << "Error listening on socket" << std::endl;
        close(serverSocket);
        return NULL;
    }

    std::cout << "Server is running on port " << PORT << std::endl;

    signal(SIGPIPE, SIG_IGN); // 忽略 SIGPIPE 信号

    // 在使用 serverSocket 前设置其为非阻塞模式
    int flags = fcntl(serverSocket, F_GETFL, 0);
    fcntl(serverSocket, F_SETFL, flags | O_NONBLOCK);

    // 等待并接受客户端连接
    while (stop_flag)
    {
        clientSocketTemp = accept(serverSocket, nullptr, nullptr);

        if (clientSocketTemp == -1)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                // 没有客户端连接，短暂休眠后继续
                usleep(1000);  // 休眠 1ms
                continue;
            }
            else
            {
                std::cerr << "Error accepting connection: " << std::strerror(errno) << std::endl;
                continue;
            }
        }

        clientSocket = clientSocketTemp;

        // 处理请求
        handleRequest(clientSocket);
    }

    close(serverSocket);
    pthread_exit(NULL);

    return NULL;
}


void exitHandler(void)
{
    printf("进程退出！！！\r\n");
    // 这里可以执行你需要的退出时操作
}

void cleanup(int signum) 
{
    printf("关闭进程!!!\r\n");
    stop_flag = 0;
}

//进程入口
int main(int argc, char* argv[]) 
{
    pthread_t soapThread; //SOAP解析线程
    pthread_t excuteActionsThread; //电机控制线程
    pthread_t getStatusBRequestThread; //电机状态应答线程
    pthread_t calibrationsThread; //校准线程
    pthread_t liquidSamplingFlowThread; //进样流程线程
    pthread_t spiMainBoardMCUCommThread; //主板MCU通讯线程
    pthread_t iBBoardMCUCommThread; //孵化模块MCU通讯线程
    pthread_t ZBoardMCUCommThread; //Z轴板模块MCU通讯线程
    //pthread_t InternalProtocolThread; //内部协议通讯线程
    pthread_t getIncubationRequestThread; //孵化模块心跳包线程
    pthread_t getZboardRequestThread; //Z轴模块心跳包线程
    pthread_t getMainBoardRequestThread; //老化(主板)模块心跳包线程
    pthread_t MemoryMonitorThread; //内存使用监控线程

    pthread_t tid;
    


    stop_flag = 1;

    atexit(exitHandler);  // 注册退出回调函数

    //signal(SIGINT, cleanup); // 注册 SIGINT 信号处理函数

    //初始化xml库
    xmlInitParser();

    //设备初始化
    DevInit();

    //创建内存监控线程
    pthread_create(&MemoryMonitorThread, NULL, memory_monitor, NULL);

    // 创建SOAP通信线程
    pthread_create(&soapThread, NULL, soapCommunication, NULL);
    usleep(3000);

    //创建电机控制线程
    pthread_create(&excuteActionsThread, NULL, ExcuteActionsFunc, NULL);
    usleep(3000);

    //创建校准线程
    pthread_create(&calibrationsThread, NULL, CalibrationsFunc, NULL);
    usleep(3000);

    //创建内部协议通讯线程
    //pthread_create(&InternalProtocolThread, NULL, InternalProtocolThreadFunc, NULL);
    //usleep(1000);

    //创建主板MCU通讯线程
    pthread_create(&spiMainBoardMCUCommThread, NULL, MainBoardMCUFunc, NULL);
    usleep(3000);

    //创建孵化模块MCU通讯线程
    pthread_create(&iBBoardMCUCommThread, NULL, IBBoardMCUFunc, NULL);
    usleep(3000);

    //创建Z轴板模块MCU通讯线程
    pthread_create(&ZBoardMCUCommThread, NULL, ZBoardMCUFunc, NULL);
    usleep(3000);

    //创建电机状态应答线程
    pthread_create(&getStatusBRequestThread, NULL, GetStatusBRequestFunc, NULL);
    usleep(3000);

    //创建孵化模块通讯线程
    pthread_create(&getIncubationRequestThread, NULL, GetIncubationRequestFunc, NULL);
    usleep(3000);

    //创建老化模块通讯线程
    pthread_create(&getMainBoardRequestThread, NULL, GetMainBoardRequestFunc, NULL);
    usleep(3000);

    //创建Z轴模块通讯线程
    pthread_create(&getZboardRequestThread, NULL, GetZboardRequestFunc, NULL);
    usleep(3000);

    //创建进样流程线程
    pthread_create(&liquidSamplingFlowThread, NULL, LiquidSamplingFlowFunc, NULL);
    usleep(3000);

    //FD监控线程
    pthread_create(&tid, NULL, fd_usage_thread, NULL);

    // 等待内存监控线程结束
    pthread_join(MemoryMonitorThread, NULL);
    printf("MemoryMonitorThread 退出\r\n");

    // 等待SOAP通讯线程结束
    pthread_join(soapThread, NULL);
    printf("soapThread 退出\r\n");

    //等待电机控制线程结束
    pthread_join(excuteActionsThread, NULL);
    printf("excuteActionsThread 退出\r\n");

    //等待校准线程结束
    pthread_join(calibrationsThread, NULL);
    printf("calibrationsThread 退出\r\n");

    //等待电机响应线程结束
    pthread_join(getStatusBRequestThread, NULL);
    printf("getStatusBRequestThread 退出\r\n");

    //等待进样流程线程结束
    pthread_join(liquidSamplingFlowThread, NULL);
    printf("liquidSamplingFlowThread 退出\r\n");

    //等待主板MCU通讯线程结束
    pthread_join(spiMainBoardMCUCommThread, NULL);
    printf("spiMainBoardMCUCommThread 退出\r\n");
    
    //等待孵化模块MCU通讯线程结束
    pthread_join(iBBoardMCUCommThread, NULL);
    printf("iBBoardMCUCommThread 退出\r\n");
    
    //等待Z轴模块MCU通讯线程结束
    pthread_join(ZBoardMCUCommThread, NULL);

    // 等待孵化模块通讯线程结束
    pthread_join(getIncubationRequestThread, NULL);
    printf("getIncubationRequestThread 退出\r\n");

    // 等待Z轴模块通讯线程结束
    pthread_join(getZboardRequestThread, NULL);

    // 等待老化模块通讯线程结束
    pthread_join(getMainBoardRequestThread, NULL);

    //等待监控线程退出
    pthread_join(tid, NULL);

    //等待内部协议通讯线程结束
    //pthread_join(InternalProtocolThread, NULL);
   //printf("getIncubationRequestThread 退出\r\n");

    printf("清理资源开始\r\n");

    //清理共享资源
    msgctl(msgExcuteActionsID, IPC_RMID, NULL); //电机控制消息队列
    msgctl(msgCalibrationsID, IPC_RMID, NULL); // 校准线程消息队列
    msgctl(msgLiquidSamplingFlowID, IPC_RMID, NULL); //流程线程消息队列

    //sem_close(IP_sem); //内部协议处理线程信号量
    //主板MCU共享内存资源释放
    munmap(MB_shared, sizeof(SharedData)); // 解除共享内存的映射
    close(MB_shmfd); // 关闭共享内存的文件描述符
    sem_close(MB_sem); //关闭主板MCU信号量
    
    //孵化模块MCU共享内存资源释放
    munmap(IB_shared, sizeof(SharedData));// 解除共享内存的映射
    close(IB_shmfd);// 关闭共享内存的文件描述符
    sem_close(IB_sem); //关闭孵化MCU信号量

    //Z轴模块MCU共享内存资源释放
    munmap(ZB_shared, sizeof(SharedData));// 解除共享内存的映射
    close(ZB_shmfd);// 关闭共享内存的文件描述符
    sem_close(ZB_sem); //关闭孵化MCU信号量

    //电机状态资源销毁
    pthread_mutex_destroy(&mutexGetStaBReq);

    //孵化模块状态资源销毁
    pthread_rwlock_destroy(&getIncubationRequestTypeVar_rwlock);
    pthread_mutex_destroy(&mutexGetIncubationReq);

    //Z轴模块状态资源销毁
    pthread_rwlock_destroy(&getZboardRequestTypeVar_rwlock);
    pthread_mutex_destroy(&mutexGetZboardReq);

    //老化模块状态资源销毁
    pthread_rwlock_destroy(&getMainBoardRequestTypeVar_rwlock);
    pthread_mutex_destroy(&mutexGetMainBoardReq);

    // CAN信号量销毁
    sem_close(CANSemID);
    close(can_sfd); //关闭CAN设备

    //客户端套接字互斥锁
    pthread_mutex_destroy(&mutexClientSocket);

    xmlCleanupParser();
    
    if(clientSocket != -1)
    {
        close(clientSocket);
    }
    
    printf("Main函数退出！！！\r\n");

    return 0;
}
