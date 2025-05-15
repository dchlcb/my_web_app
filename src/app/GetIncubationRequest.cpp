#include <unistd.h>
#include <cstring>
#include "CANOpen.h"
#include "Public.h"
#include "GetIncubationRequest.h"
#include "LiquidSamplingFlow.h"
#include "main.h"

std::atomic<int> Pos1Reserved(0); //孵化位置1
std::atomic<int> Pos2Reserved(0); //孵化位置2
std::atomic<int> Pos3Reserved(0); //孵化位置3
std::atomic<int> Pos4Reserved(0); //孵化位置4
std::atomic<int> Pos5Reserved(0); //孵化位置5
std::atomic<int> Pos6Reserved(0); //孵化位置6


GetIncubationRequestType getIncubationRequestTypeVar;

volatile uint8_t incubation_request_received = 0;


void* IncubationReceiveThreadFunc(void* arg)
{
    while(stop_flag)
    {
        GetIncubationHeart();
        printf("孵化模块心跳包\r\n");
        sleep(3);
    }

    sem_post(IB_sem); // 解锁

    printf("孵化模块心跳包线程退出\r\n");

    pthread_exit(NULL);
}

		
void* GetIncubationRequestFunc(void* arg)
{
    GetIncubationRequestType tempData;

    pthread_t IncubationReceiveThread;

    // 创建监听线程
    if(pthread_create(&IncubationReceiveThread, NULL, IncubationReceiveThreadFunc, NULL) != 0)
    {
        perror("创建孵化模块轮询状态线程失败");
        pthread_exit(NULL);
    }

    while(stop_flag)
    {
        pthread_mutex_lock(&mutexGetIncubationReq);
        
        if(incubation_request_received)
        {
            sleep(1);
        }

        //printf("GetIncubationRequestFunc\r\n");
        pthread_mutex_unlock(&mutexGetIncubationReq);

        sleep(5);
    }

    pthread_mutex_unlock(&mutexGetIncubationReq);
    pthread_rwlock_unlock(&getIncubationRequestTypeVar_rwlock);

    //等待状态读取线程结束
    pthread_join(IncubationReceiveThread, NULL);

    printf("孵化模块与PC通讯应答线程退出\r\n");
    pthread_exit(NULL);
}


/*状态应答*/
void getIncubationRequestResponse(int sockfd, GetIncubationRequestType* getStaBRes) 
{
    int err = 0;

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
    xmlNodePtr response = xmlNewNode(NULL, BAD_CAST "GetMachineObjectResponse");
    xmlNsPtr ns = xmlNewNs(response, BAD_CAST "http://ctc.ch/pal3/api/v1/", NULL);
    xmlAddChild(body, response);

    char tempBuf[20];
    memset(tempBuf, 0, sizeof(tempBuf));

    // 创建 DoubleUnitParameter 节点
    xmlNodePtr x_param = xmlNewNode(NULL, BAD_CAST "machineObjectData");
    xmlNewProp(x_param, BAD_CAST "descriptionName", BAD_CAST "AgitatorDescription");
    xmlNewProp(x_param, BAD_CAST "name", BAD_CAST "Agitator 1");
    xmlAddChild(response, x_param);

    xmlNodePtr Children = xmlNewNode(NULL, BAD_CAST "Children");
    xmlAddChild(x_param, Children);

    xmlNodePtr Parameters = xmlNewNode(NULL, BAD_CAST "Parameters");
    xmlAddChild(x_param, Parameters);

    xmlNodePtr y_param = xmlNewNode(NULL, BAD_CAST "IntParameter");
    xmlNewProp(y_param, BAD_CAST "name", BAD_CAST "Speed");
    sprintf(tempBuf, "%d", getStaBRes->Speed);
    xmlNewProp(y_param, BAD_CAST "value", BAD_CAST tempBuf);
    xmlNewProp(y_param, BAD_CAST "unit", BAD_CAST "rpm");
    xmlAddChild(Parameters, y_param);

    xmlNodePtr z_param = xmlNewNode(NULL, BAD_CAST "BooleanParameter");
    xmlNewProp(z_param, BAD_CAST "name", BAD_CAST "HeatAgitator");
    sprintf(tempBuf, "%d", getStaBRes->HeatAgitator);
    xmlNewProp(z_param, BAD_CAST "value", BAD_CAST tempBuf);
    xmlAddChild(Parameters, z_param);

    xmlNodePtr x_speed = xmlNewNode(NULL, BAD_CAST "DoubleUnitParameter");
    xmlNewProp(x_speed, BAD_CAST "name", BAD_CAST "Temperature");
    sprintf(tempBuf, "%.1f", getStaBRes->Temperature);
    xmlNewProp(x_speed, BAD_CAST "value", BAD_CAST tempBuf);
    xmlNewProp(x_speed, BAD_CAST "unit", BAD_CAST "℃");
    xmlAddChild(Parameters, x_speed);

    xmlNodePtr y_speed = xmlNewNode(NULL, BAD_CAST "BooleanParameter");
    xmlNewProp(y_speed, BAD_CAST "name", BAD_CAST "Pos1Reserved");
    if(Pos1Reserved.load())
    {
        std::strcpy(tempBuf, "True");
    }
    else
    {
        std::strcpy(tempBuf, "False");
    }
    //sprintf(tempBuf, "%d", Pos1Reserved.load());
    xmlNewProp(y_speed, BAD_CAST "value", BAD_CAST tempBuf);
    xmlAddChild(Parameters, y_speed);

    xmlNodePtr tempPos2Reserved = xmlNewNode(NULL, BAD_CAST "BooleanParameter");
    xmlNewProp(tempPos2Reserved, BAD_CAST "name", BAD_CAST "Pos2Reserved");
    if(Pos2Reserved.load())
    {
        std::strcpy(tempBuf, "True");
    }
    else
    {
        std::strcpy(tempBuf, "False");
    }
    //sprintf(tempBuf, "%d", Pos2Reserved.load());
    xmlNewProp(tempPos2Reserved, BAD_CAST "value", BAD_CAST tempBuf);
    xmlAddChild(Parameters, tempPos2Reserved);

    xmlNodePtr tempPos3Reserved = xmlNewNode(NULL, BAD_CAST "BooleanParameter");
    xmlNewProp(tempPos3Reserved, BAD_CAST "name", BAD_CAST "Pos3Reserved");
    if(Pos3Reserved.load())
    {
        std::strcpy(tempBuf, "True");
    }
    else
    {
        std::strcpy(tempBuf, "False");
    }
    //sprintf(tempBuf, "%d", Pos3Reserved.load());
    xmlNewProp(tempPos3Reserved, BAD_CAST "value", BAD_CAST tempBuf);
    xmlAddChild(Parameters, tempPos3Reserved);

    xmlNodePtr tempPos4Reserved = xmlNewNode(NULL, BAD_CAST "BooleanParameter");
    xmlNewProp(tempPos4Reserved, BAD_CAST "name", BAD_CAST "Pos4Reserved");
    if(Pos4Reserved.load())
    {
        std::strcpy(tempBuf, "True");
    }
    else
    {
        std::strcpy(tempBuf, "False");
    }
    //sprintf(tempBuf, "%d", Pos4Reserved.load());
    xmlNewProp(tempPos4Reserved, BAD_CAST "value", BAD_CAST tempBuf);
    xmlAddChild(Parameters, tempPos4Reserved);

    xmlNodePtr tempPos5Reserved = xmlNewNode(NULL, BAD_CAST "BooleanParameter");
    xmlNewProp(tempPos5Reserved, BAD_CAST "name", BAD_CAST "Pos5Reserved");
    if(Pos5Reserved.load())
    {
        std::strcpy(tempBuf, "True");
    }
    else
    {
        std::strcpy(tempBuf, "False");
    }
    //sprintf(tempBuf, "%d", Pos5Reserved.load());
    xmlNewProp(tempPos5Reserved, BAD_CAST "value", BAD_CAST tempBuf);
    xmlAddChild(Parameters, tempPos5Reserved);

    xmlNodePtr tempPos6Reserved = xmlNewNode(NULL, BAD_CAST "BooleanParameter");
    xmlNewProp(tempPos6Reserved, BAD_CAST "name", BAD_CAST "Pos6Reserved");
    if(Pos6Reserved.load())
    {
        std::strcpy(tempBuf, "True");
    }
    else
    {
        std::strcpy(tempBuf, "False");
    }
    //sprintf(tempBuf, "%d", Pos6Reserved.load());
    xmlNewProp(tempPos6Reserved, BAD_CAST "value", BAD_CAST tempBuf);
    xmlAddChild(Parameters, tempPos6Reserved);
   
    // 序列化 XML 文档为字符串
    xmlChar *xml_buf;
    int xml_size;
    xmlDocDumpFormatMemoryEnc(doc, &xml_buf, &xml_size, "UTF-8", 1);    

    //TCP发送
    const char* res = "HTTP/1.1 200 OK\r\nContent-Type: text/xml\r\n\r\n";
    write(sockfd, res, strlen(res));    
    write(sockfd, xml_buf, xml_size);
    close(sockfd);

    // 清理
    xmlFree(xml_buf);
    xmlFreeDoc(doc);

}