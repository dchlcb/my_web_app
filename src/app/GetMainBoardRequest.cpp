#include <unistd.h>
#include <cstring>
#include "CANOpen.h"
#include "Public.h"
#include "GetMainBoardRequest.h"
#include "LiquidSamplingFlow.h"
#include "main.h"


GetMainBoardRequestType getMainBoardRequestTypeVar;

volatile uint8_t mainboard_request_received = 0;


void* MainBoardReceiveThreadFunc(void* arg)
{
    while(stop_flag)
    {
        //获取GC状态
        GCStateRead();
        printf("主板模块心跳包\r\n");
        sleep(6);
    }

    printf("主板模块心跳包线程退出\r\n");

    pthread_exit(NULL);
}


void* AgingReceiveThreadFunc(void* arg)
{
    while(stop_flag)
    {
        GetAgingHeart();
        printf("老化模块心跳包\r\n");
        sleep(3);
    }

    printf("老化模块心跳包线程退出\r\n");

    pthread_exit(NULL);
}

		
void* GetMainBoardRequestFunc(void* arg)
{
    GetMainBoardRequestType tempData;

    pthread_t MainBoardReceiveThread;
    pthread_t AgingReceiveThread;

    // 创建主板MCU监听线程
    if(pthread_create(&MainBoardReceiveThread, NULL, MainBoardReceiveThreadFunc, NULL) != 0)
    {
        perror("创建主板模块轮询状态线程失败");
        pthread_exit(NULL);
    }

    // 创建老化模块监听线程
    if(pthread_create(&AgingReceiveThread, NULL, AgingReceiveThreadFunc, NULL) != 0)
    {
        perror("创建老化模块轮询状态线程失败");
        pthread_exit(NULL);
    }

    while(stop_flag)
    {
        pthread_mutex_lock(&mutexGetMainBoardReq);
        
        if(mainboard_request_received)
        {
            pthread_rwlock_rdlock(&getMainBoardRequestTypeVar_rwlock);
            //tempData = getIncubationRequestTypeVar;
            //发送SOAP应答
            //getIncubationRequestResponse(&tempData);
            pthread_rwlock_unlock(&getMainBoardRequestTypeVar_rwlock);

            // 请求条件满足，执行应答消息发送操作
            mainboard_request_received = 0;  // 重置条件
        }

        //printf("GetIncubationRequestFunc\r\n");
        pthread_mutex_unlock(&mutexGetMainBoardReq);

        usleep(1000);
    }

    pthread_mutex_unlock(&mutexGetMainBoardReq);
    pthread_rwlock_unlock(&getMainBoardRequestTypeVar_rwlock);

    //等待状态读取线程结束
    pthread_join(MainBoardReceiveThread, NULL);

    pthread_join(AgingReceiveThread, NULL);

    printf("主板模块与PC通讯应答线程退出\r\n");
    pthread_exit(NULL);
}


#if 1
/*状态应答*/
void getAgligRequestResponse(int sockfd, GetMainBoardRequestType* getStaBRes) 
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
    xmlNewProp(x_param, BAD_CAST "descriptionName", BAD_CAST "BurninDescription");
    xmlNewProp(x_param, BAD_CAST "name", BAD_CAST "Burn-In 1");
    xmlAddChild(response, x_param);

    xmlNodePtr Children = xmlNewNode(NULL, BAD_CAST "Children");
    xmlAddChild(x_param, Children);

    xmlNodePtr Parameters = xmlNewNode(NULL, BAD_CAST "Parameters");
    xmlAddChild(x_param, Parameters);

    xmlNodePtr z_param = xmlNewNode(NULL, BAD_CAST "BooleanParameter");
    xmlNewProp(z_param, BAD_CAST "name", BAD_CAST "HeatBurnin");
    sprintf(tempBuf, "%d", getStaBRes->HeatAgitator);
    xmlNewProp(z_param, BAD_CAST "value", BAD_CAST tempBuf);
    xmlAddChild(Parameters, z_param);

    xmlNodePtr v_param = xmlNewNode(NULL, BAD_CAST "BooleanParameter");
    xmlNewProp(v_param, BAD_CAST "name", BAD_CAST "Valve");
    sprintf(tempBuf, "%d", getStaBRes->valCtrlFlg);
    xmlNewProp(v_param, BAD_CAST "value", BAD_CAST tempBuf);
    xmlAddChild(Parameters, v_param);

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