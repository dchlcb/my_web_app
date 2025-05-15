#include <unistd.h>
#include <cstring>
#include "CANOpen.h"
#include "Public.h"
#include "GetZboardRequest.h"
#include "LiquidSamplingFlow.h"
#include "main.h"


GetZboardRequestType getZboardRequestTypeVar;

volatile uint8_t zboard_request_received = 0;


void* ZboardReceiveThreadFunc(void* arg)
{
    while(stop_flag)
    {
        GetZboardHeart();
        printf("Z轴模块心跳包\r\n");
        sleep(5);
    }

    sem_post(ZB_sem); // 解锁

    printf("Z轴模块心跳包线程退出\r\n");

    pthread_exit(NULL);
}

		
void* GetZboardRequestFunc(void* arg)
{
    GetZboardRequestType tempData;

    pthread_t ZboardReceiveThread;

    // 创建监听线程
    if(pthread_create(&ZboardReceiveThread, NULL, ZboardReceiveThreadFunc, NULL) != 0)
    {
        perror("创建Z轴模块轮询状态线程失败");
        pthread_exit(NULL);
    }

    while(stop_flag)
    {
        pthread_mutex_lock(&mutexGetZboardReq);
        
        //if(zboard_request_received)
        {
            //pthread_rwlock_rdlock(&getZboardRequestTypeVar_rwlock);
            //tempData = getZboardRequestTypeVar;
            //发送SOAP应答
            //getZboardRequestResponse(&tempData);
            //pthread_rwlock_unlock(&getZboardRequestTypeVar_rwlock);

            // 请求条件满足，执行应答消息发送操作
            //zboard_request_received = 0;  // 重置条件
        }

        //printf("GetIncubationRequestFunc\r\n");
        pthread_mutex_unlock(&mutexGetZboardReq);

        sleep(5);
    }

    pthread_mutex_unlock(&mutexGetZboardReq);
    pthread_rwlock_unlock(&getZboardRequestTypeVar_rwlock);

    //等待状态读取线程结束
    pthread_join(ZboardReceiveThread, NULL);

    printf("Z轴模块与PC通讯应答线程退出\r\n");
    pthread_exit(NULL);
}


#if 1
/*状态应答*/
void getZboardRequestResponse(int sockfd, GetZboardRequestType* getStaBRes) 
{
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
    xmlNewProp(x_param, BAD_CAST "descriptionName", BAD_CAST "ToolDescription");
    xmlNewProp(x_param, BAD_CAST "name", BAD_CAST "LIQ 1");
    xmlAddChild(response, x_param);

    xmlNodePtr Children = xmlNewNode(NULL, BAD_CAST "Children");
    xmlAddChild(x_param, Children);

    xmlNodePtr Parameters = xmlNewNode(NULL, BAD_CAST "Parameters");
    xmlAddChild(x_param, Parameters);

    xmlNodePtr z_param = xmlNewNode(NULL, BAD_CAST "BooleanParameter");
    xmlNewProp(z_param, BAD_CAST "name", BAD_CAST "HeatSyringe");
    sprintf(tempBuf, "%d", getStaBRes->HeatAgitator);
    xmlNewProp(z_param, BAD_CAST "value", BAD_CAST tempBuf);
    xmlAddChild(Parameters, z_param);

    xmlNodePtr x_speed = xmlNewNode(NULL, BAD_CAST "DoubleUnitParameter");
    xmlNewProp(x_speed, BAD_CAST "name", BAD_CAST "Temperature");
    sprintf(tempBuf, "%.1f", getStaBRes->Temperature);
    xmlNewProp(x_speed, BAD_CAST "value", BAD_CAST tempBuf);
    xmlNewProp(x_speed, BAD_CAST "unit", BAD_CAST "℃");
    xmlAddChild(Parameters, x_speed);
   
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

#endif