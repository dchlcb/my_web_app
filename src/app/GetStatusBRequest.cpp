#include <unistd.h>
#include <cstring>
#include "CANOpen.h"
#include "Public.h"
#include "GetStatusBRequest.h"
#include "LiquidSamplingFlow.h"
#include "main.h"


std::atomic<int> InjectionReady(0); //是否已进样标志  注射就绪
std::atomic<int>  CanExecuteSample(1); //是否可以进行下一个样品  允许取样
std::atomic<int>  TriggerReady(1); //GC是否已就绪  硬件触发


pthread_rwlock_t getMotorStateVar_rwlock = PTHREAD_RWLOCK_INITIALIZER;
GetMotorStateType getMotorStateVar;


std::atomic<int> IncubationTempStableFlag(0);
std::atomic<int> ConditionTempStableFlag(0);
std::atomic<int> InjectionTempStableFlag(0);


volatile uint8_t request_received = 0;


void* OtherReceiveThreadFunc(void* arg)
{
   
    while(stop_flag)
    {
        pthread_rwlock_wrlock(&getMotorStateVar_rwlock);
        //获取电机状态
        GetMotorStatus(&getMotorStateVar);
        pthread_rwlock_unlock(&getMotorStateVar_rwlock);

        usleep(300*1000);

        //printf("OtherReceiveThreadFunc\r\n");
    }

    sem_post(MB_sem); // 解锁

    sem_post(CANSemID);

    pthread_rwlock_unlock(&getMotorStateVar_rwlock);

    printf("电机状态读取线程退出\r\n");

    pthread_exit(NULL);
}

		
void* GetStatusBRequestFunc(void* arg)
{
    GetMotorStateType motorData;

    pthread_t otherReceiveThread;

    // 创建监听线程
    if(pthread_create(&otherReceiveThread, NULL, OtherReceiveThreadFunc, NULL) != 0)
    {
        perror("创建孵化模块MCU监听线程失败");
        pthread_exit(NULL);
    }

    while(stop_flag)
    {
        #if 0
        pthread_mutex_lock(&mutexGetStaBReq);
        if(request_received)
        {
            request_received = 0;  // 重置条件
            
            pthread_rwlock_rdlock(&getMotorStateVar_rwlock);
            motorData = getMotorStateVar;
            //发送SOAP应答
            getStatusBRequestResponse(&motorData);
            pthread_rwlock_unlock(&getMotorStateVar_rwlock);
        }
        pthread_mutex_unlock(&mutexGetStaBReq);
        #endif
        
        usleep(400*1000);

        //printf("GetStatusBRequestFunc\r\n");
        
    }

    pthread_mutex_unlock(&mutexGetStaBReq);
    pthread_rwlock_unlock(&getMotorStateVar_rwlock);

    //等待状态读取线程结束
    pthread_join(otherReceiveThread, NULL);

    pthread_rwlock_destroy(&getMotorStateVar_rwlock);

    printf("应答线程退出\r\n");
    pthread_exit(NULL);
}


/*获取电机状态*/
void GetMotorStatus(GetMotorStateType* getStaBRes)
{
       
	int32_t tmpData = 0;

	//读取目标绝对位置值
	SDORead_ShellFunc(can_sfd, en_X, 0x6064, 0x00, reinterpret_cast<uint8_t*>(&tmpData));
    //printf("X轴6064 = %d\r\n", tmpData);
	usleep(15*1000);
    
    //printf("GetMotorStatus en_X tmpData = %d\n", tmpData);
	getStaBRes->d_distanceRespone[en_X-1] = ResoConvPos(en_X, abs(tmpData));

    //SDORead_ShellFunc(can_sfd, en_X, 0x6062, 0x00, reinterpret_cast<uint8_t*>(&tmpData));
    //printf("X轴6062 = %d\r\n", tmpData);
	//usleep(15*1000);

	SDORead_ShellFunc(can_sfd, en_Y, 0x6064, 0x00, reinterpret_cast<uint8_t*>(&tmpData));
	usleep(15*1000);
    //printf("GetMotorStatus en_Y tmpData = %d\n", tmpData);
	getStaBRes->d_distanceRespone[en_Y-1] = ResoConvPos(en_Y, abs(tmpData));

	SDORead_ShellFunc(can_sfd, en_Z, 0x6064, 0x00, reinterpret_cast<uint8_t*>(&tmpData));
	usleep(15*1000);
    //printf("GetMotorStatus en_Z tmpData = %d\n", tmpData);
	getStaBRes->d_distanceRespone[en_Z-1] = ResoConvPos(en_Z, abs(tmpData));

    #if 0
    //读取3210的所有参数
    SDORead_ShellFunc(can_sfd, en_X, 0x3210, 0x01, reinterpret_cast<uint8_t*>(&tmpData));
    printf("X轴3210:01 = %x\r\n",tmpData);
	usleep(10*1000);

    SDORead_ShellFunc(can_sfd, en_X, 0x3210, 0x04, reinterpret_cast<uint8_t*>(&tmpData));
    printf("X轴3210:04 = %x\r\n",tmpData);
	usleep(10*1000);
    #endif
    //SDORead_ShellFunc(can_sfd, en_X, 0x3210, 0x02, reinterpret_cast<uint8_t*>(&tmpData));
    //printf("X轴3210:02 = %x\r\n",tmpData);
	//usleep(10*1000);

    //SDORead_ShellFunc(can_sfd, en_X, 0x3210, 0x03, reinterpret_cast<uint8_t*>(&tmpData));
    //printf("X轴3210:03 = %x\r\n",tmpData);
	//usleep(10*1000);
    


    //SDORead_ShellFunc(can_sfd, en_X, 0x3210, 0x05, reinterpret_cast<uint8_t*>(&tmpData));
    //printf("X轴3210:05 = %x\r\n",tmpData);
	//usleep(10*1000);

    //SDORead_ShellFunc(can_sfd, en_X, 0x3210, 0x06, reinterpret_cast<uint8_t*>(&tmpData));
    //printf("X轴3210:06 = %x\r\n",tmpData);
	//usleep(10*1000);

    //SDORead_ShellFunc(can_sfd, en_X, 0x3210, 0x07, reinterpret_cast<uint8_t*>(&tmpData));
    //printf("X轴3210:07 = %x\r\n",tmpData);
	//usleep(10*1000);

    //SDORead_ShellFunc(can_sfd, en_X, 0x3210, 0x08, reinterpret_cast<uint8_t*>(&tmpData));
    //printf("X轴3210:08 = %x\r\n",tmpData);
	//usleep(10*1000);

    //SDORead_ShellFunc(can_sfd, en_X, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&tmpData));
    //printf("X轴2039:05 = %x\r\n",tmpData);
	//usleep(10*1000);


    #if 0
    SDORead_ShellFunc(can_sfd, en_Y, 0x3210, 0x01, reinterpret_cast<uint8_t*>(&tmpData));
    printf("Y轴3210:01 = %x\r\n",tmpData);
	usleep(10*1000);

    SDORead_ShellFunc(can_sfd, en_Y, 0x3210, 0x04, reinterpret_cast<uint8_t*>(&tmpData));
    printf("Y轴3210:04 = %x\r\n",tmpData);
	usleep(10*1000);

    
    SDORead_ShellFunc(can_sfd, en_Y, 0x3210, 0x02, reinterpret_cast<uint8_t*>(&tmpData));
    printf("Y轴3210:02 = %x\r\n",tmpData);
	usleep(10*1000);

    SDORead_ShellFunc(can_sfd, en_Y, 0x3210, 0x03, reinterpret_cast<uint8_t*>(&tmpData));
    printf("Y轴3210:03 = %x\r\n",tmpData);
	usleep(10*1000);

    SDORead_ShellFunc(can_sfd, en_Y, 0x3210, 0x05, reinterpret_cast<uint8_t*>(&tmpData));
    printf("Y轴3210:05 = %x\r\n",tmpData);
	usleep(10*1000);

    SDORead_ShellFunc(can_sfd, en_Y, 0x3210, 0x06, reinterpret_cast<uint8_t*>(&tmpData));
    printf("Y轴3210:06 = %x\r\n",tmpData);
	usleep(10*1000);

    SDORead_ShellFunc(can_sfd, en_Y, 0x3210, 0x07, reinterpret_cast<uint8_t*>(&tmpData));
    printf("Y轴3210:07 = %x\r\n",tmpData);
	usleep(10*1000);

    SDORead_ShellFunc(can_sfd, en_Y, 0x3210, 0x08, reinterpret_cast<uint8_t*>(&tmpData));
    printf("Y轴3210:08 = %x\r\n",tmpData);
	usleep(10*1000);

    SDORead_ShellFunc(can_sfd, en_Y, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&tmpData));
    printf("Y轴2039:05 = %x\r\n",tmpData);
	usleep(10*1000);
    #endif

    #if 0
    SDORead_ShellFunc(can_sfd, en_Z, 0x3210, 0x01, reinterpret_cast<uint8_t*>(&tmpData));
    printf("Z轴3210:01 = %x\r\n",tmpData);
	usleep(10*1000);

    SDORead_ShellFunc(can_sfd, en_Z, 0x3210, 0x04, reinterpret_cast<uint8_t*>(&tmpData));
    printf("Z轴3210:04 = %x\r\n",tmpData);
	usleep(10*1000);

    
    SDORead_ShellFunc(can_sfd, en_Z, 0x3210, 0x02, reinterpret_cast<uint8_t*>(&tmpData));
    printf("Z轴3210:02 = %x\r\n",tmpData);
	usleep(10*1000);

    SDORead_ShellFunc(can_sfd, en_Z, 0x3210, 0x03, reinterpret_cast<uint8_t*>(&tmpData));
    printf("Z轴3210:03 = %x\r\n",tmpData);
	usleep(10*1000);
    
    SDORead_ShellFunc(can_sfd, en_Z, 0x3210, 0x05, reinterpret_cast<uint8_t*>(&tmpData));
    printf("Z轴3210:05 = %x\r\n",tmpData);
	usleep(10*1000);

    SDORead_ShellFunc(can_sfd, en_Z, 0x3210, 0x06, reinterpret_cast<uint8_t*>(&tmpData));
    printf("Z轴3210:06 = %x\r\n",tmpData);
	usleep(10*1000);

    SDORead_ShellFunc(can_sfd, en_Z, 0x3210, 0x07, reinterpret_cast<uint8_t*>(&tmpData));
    printf("Z轴3210:07 = %x\r\n",tmpData);
	usleep(10*1000);

    SDORead_ShellFunc(can_sfd, en_Z, 0x3210, 0x08, reinterpret_cast<uint8_t*>(&tmpData));
    printf("Z轴3210:08 = %x\r\n",tmpData);
	usleep(10*1000);


    

    SDORead_ShellFunc(can_sfd, en_Z, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&tmpData));
    printf("Z轴电流值 = %d\r\n",tmpData);
	usleep(15*1000);

    SDORead_ShellFunc(can_sfd, en_Y, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&tmpData));
    printf("Y轴电流值 = %d\r\n",tmpData);
	usleep(15*1000);

    SDORead_ShellFunc(can_sfd, en_X, 0x2039, 0x05, reinterpret_cast<uint8_t*>(&tmpData));
    printf("X轴电流值 = %d\r\n",tmpData);
	usleep(15*1000);
    #endif

	//读取当前速度值
	//SDORead_ShellFunc(can_sfd, en_X, 0x6069, 0x00, reinterpret_cast<uint8_t*>(&tmpData));
	//usleep(15*1000);
    tmpData = 100;
	getStaBRes->d_speedRespone[en_X-1] = ResoConvPos(en_X, tmpData);

	//SDORead_ShellFunc(can_sfd, en_Y, 0x6069, 0x00, reinterpret_cast<uint8_t*>(&tmpData));
	//usleep(15*1000);
	getStaBRes->d_speedRespone[en_Y-1] = ResoConvPos(en_Y, tmpData);

	//SDORead_ShellFunc(can_sfd, en_Z, 0x6069, 0x00, reinterpret_cast<uint8_t*>(&tmpData));
	//usleep(15*1000);
	getStaBRes->d_speedRespone[en_Z-1] = ResoConvPos(en_Z, tmpData);
    
}


/*状态应答*/
void getStatusBRequestResponse(int sockfd, GetMotorStateType* getStaBRes) 
{
    int err = 0;                              /* 记录步骤是否失败 */
    
    // 创建一个新的 XML 文档
    xmlDocPtr doc = xmlNewDoc(BAD_CAST "1.0");
    
    // 创建 SOAP Envelope 节点
    xmlNodePtr envelope = xmlNewNode(NULL, BAD_CAST "s:Envelope");
    xmlNsPtr soap_ns = xmlNewNs(envelope, BAD_CAST "http://www.w3.org/2003/05/soap-envelope", BAD_CAST "s");
    xmlNsPtr addressing_ns = xmlNewNs(envelope, BAD_CAST "http://www.w3.org/2005/08/addressing", BAD_CAST "a");

    // 将 Envelope 节点添加到文档
    xmlDocSetRootElement(doc, envelope);

    // 创建 Header 节点并添加到 Envelope
    xmlNodePtr header = xmlNewNode(NULL, BAD_CAST "s:Header");
    xmlAddChild(envelope, header);

    // 创建 Body 节点并添加到 Envelope
    xmlNodePtr body = xmlNewNode(NULL, BAD_CAST "s:Body");
    xmlAddChild(envelope, body);

    // 创建 GetStatusBRequestResponse 节点并添加到 Body
    xmlNodePtr response = xmlNewNode(NULL, BAD_CAST "GetStatusBRequestResponse");
    xmlNsPtr ns = xmlNewNs(response, BAD_CAST "http://ctc.ch/pal3/api/v1/", NULL);
    xmlAddChild(body, response);

    char tempBuf[1024];
    memset(tempBuf, 0, sizeof(tempBuf));

    // 创建 DoubleUnitParameter 节点
    xmlNodePtr x_param = xmlNewNode(NULL, BAD_CAST "DoubleUnitParameter");
    xmlNewProp(x_param, BAD_CAST "name", BAD_CAST "x");
    sprintf(tempBuf, "%.1f", getStaBRes->d_distanceRespone[en_X-1]);
    xmlNewProp(x_param, BAD_CAST "value", BAD_CAST tempBuf);
    xmlNewProp(x_param, BAD_CAST "unit", BAD_CAST "mm");
    xmlAddChild(response, x_param);

    xmlNodePtr y_param = xmlNewNode(NULL, BAD_CAST "DoubleUnitParameter");
    xmlNewProp(y_param, BAD_CAST "name", BAD_CAST "y");
    sprintf(tempBuf, "%.1f", getStaBRes->d_distanceRespone[en_Y-1]);
    xmlNewProp(y_param, BAD_CAST "value", BAD_CAST tempBuf);
    xmlNewProp(y_param, BAD_CAST "unit", BAD_CAST "mm");
    xmlAddChild(response, y_param);

    xmlNodePtr z_param = xmlNewNode(NULL, BAD_CAST "DoubleUnitParameter");
    xmlNewProp(z_param, BAD_CAST "name", BAD_CAST "z");
    sprintf(tempBuf, "%.1f", getStaBRes->d_distanceRespone[en_Z-1]);
    xmlNewProp(z_param, BAD_CAST "value", BAD_CAST tempBuf);
    xmlNewProp(z_param, BAD_CAST "unit", BAD_CAST "mm");
    xmlAddChild(response, z_param);

    xmlNodePtr x_speed = xmlNewNode(NULL, BAD_CAST "DoubleUnitParameter");
    xmlNewProp(x_speed, BAD_CAST "name", BAD_CAST "xSpeed");
    sprintf(tempBuf, "%.1f", getStaBRes->d_speedRespone[en_X-1]);
    xmlNewProp(x_speed, BAD_CAST "value", BAD_CAST tempBuf);
    xmlNewProp(x_speed, BAD_CAST "unit", BAD_CAST "mm/s");
    xmlAddChild(response, x_speed);

    xmlNodePtr y_speed = xmlNewNode(NULL, BAD_CAST "DoubleUnitParameter");
    xmlNewProp(y_speed, BAD_CAST "name", BAD_CAST "ySpeed");
    sprintf(tempBuf, "%.1f", getStaBRes->d_speedRespone[en_Y-1]);
    xmlNewProp(y_speed, BAD_CAST "value", BAD_CAST tempBuf);
    xmlNewProp(y_speed, BAD_CAST "unit", BAD_CAST "mm/s");
    xmlAddChild(response, y_speed);

    xmlNodePtr z_speed = xmlNewNode(NULL, BAD_CAST "DoubleUnitParameter");
    xmlNewProp(z_speed, BAD_CAST "name", BAD_CAST "zSpeed");
    sprintf(tempBuf, "%.1f", getStaBRes->d_speedRespone[en_Z-1]);
    xmlNewProp(z_speed, BAD_CAST "value", BAD_CAST tempBuf);
    xmlNewProp(z_speed, BAD_CAST "unit", BAD_CAST "mm/s");
    xmlAddChild(response, z_speed);

    //第一个取样流程是否执行完成标志
    xmlNodePtr p_CanExecuteSample = xmlNewNode(NULL, BAD_CAST "BooleanParameter");
    xmlNewProp(p_CanExecuteSample, BAD_CAST "name", BAD_CAST "CanExecuteSample");
    printf("CanExecuteSample.load() = %d \r\n", CanExecuteSample.load());
    sprintf(tempBuf, "%d", CanExecuteSample.load());
    xmlNewProp(p_CanExecuteSample, BAD_CAST "value", BAD_CAST tempBuf);
    xmlAddChild(response, p_CanExecuteSample);
    
    //是否已注入样品标志
    xmlNodePtr p_InjectionReady = xmlNewNode(NULL, BAD_CAST "BooleanParameter");
    xmlNewProp(p_InjectionReady, BAD_CAST "name", BAD_CAST "InjectionReady");
    sprintf(tempBuf, "%d", InjectionReady.load());
    printf("InjectionReady.load() = %d \r\n", InjectionReady.load());
    xmlNewProp(p_InjectionReady, BAD_CAST "value", BAD_CAST tempBuf);
    xmlAddChild(response, p_InjectionReady); 
    
    //GC是否已就绪
    xmlNodePtr p_TriggerReady = xmlNewNode(NULL, BAD_CAST "BooleanParameter");
    xmlNewProp(p_TriggerReady, BAD_CAST "name", BAD_CAST "TriggerReady");
    sprintf(tempBuf, "%d", TriggerReady.load());
    printf("TriggerReady.load() = %d \r\n", TriggerReady.load());
    xmlNewProp(p_TriggerReady, BAD_CAST "value", BAD_CAST tempBuf);
    xmlAddChild(response, p_TriggerReady);
    
    //孵化器温控稳定标志
    xmlNodePtr IncubationTempStable = xmlNewNode(NULL, BAD_CAST "BooleanParameter");
    xmlNewProp(IncubationTempStable, BAD_CAST "name", BAD_CAST "IncubationTempStableFlag");
    if(IncubationTempStableFlag.load())
    {
        std::strcpy(tempBuf, "True");
    }
    else
    {
        std::strcpy(tempBuf, "False");
    }
    xmlNewProp(IncubationTempStable, BAD_CAST "value", BAD_CAST tempBuf);
    xmlAddChild(response, IncubationTempStable);

    //老化模块温控稳定标志
    xmlNodePtr ConditionTempStable = xmlNewNode(NULL, BAD_CAST "BooleanParameter");
    xmlNewProp(ConditionTempStable, BAD_CAST "name", BAD_CAST "ConditionTempStableFlag");
    if(ConditionTempStableFlag.load())
    {
        std::strcpy(tempBuf, "True");
    }
    else
    {
        std::strcpy(tempBuf, "False");
    }
    xmlNewProp(ConditionTempStable, BAD_CAST "value", BAD_CAST tempBuf);
    xmlAddChild(response, ConditionTempStable);

    //进样工具温控稳定标志
    xmlNodePtr InjectionTempStable = xmlNewNode(NULL, BAD_CAST "BooleanParameter");
    xmlNewProp(InjectionTempStable, BAD_CAST "name", BAD_CAST "InjectionTempStableFlag");
    if(InjectionTempStableFlag.load())
    {
        std::strcpy(tempBuf, "True");
    }
    else
    {
        std::strcpy(tempBuf, "False");
    }
    xmlNewProp(InjectionTempStable, BAD_CAST "value", BAD_CAST tempBuf);
    xmlAddChild(response, InjectionTempStable);


    //报警码
    xmlNodePtr FaultCode = xmlNewNode(NULL, BAD_CAST "StringParameter");
    xmlNewProp(FaultCode, BAD_CAST "name", BAD_CAST "FaultCode");
    //alarmCodeVar.addAlarm(1);
    //alarmCodeVar.addAlarm(2);
    std::strcpy(tempBuf, alarmCodeVar.getAlarmString().c_str());
    printf("报警码字符串 %s\r\n", tempBuf);
    xmlNewProp(FaultCode, BAD_CAST "value", BAD_CAST tempBuf);
    xmlAddChild(response, FaultCode);



    // 序列化 XML 文档为字符串
    xmlChar *xml_buf;
    int xml_size;
    xmlDocDumpFormatMemoryEnc(doc, &xml_buf, &xml_size, "UTF-8", 1);    


    //TCP发送

    const char* res = "HTTP/1.1 200 OK\r\nContent-Type: text/xml\r\n\r\n";
    write(sockfd, res, strlen(res));    
    write(sockfd, xml_buf, xml_size);

    // 清理
    xmlFree(xml_buf);
    xmlFreeDoc(doc);
}