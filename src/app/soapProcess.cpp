#include "soapProcess.h"
#include "CANOpen.h"
#include "ExcuteActions.h"
#include "GetStatusBRequest.h"
#include "Calibrations.h"
#include "LiquidSamplingFlow.h"
#include "GetIncubationRequest.h"
#include "main.h"
#include "GetIncubationRequest.h"
#include "GetZboardRequest.h"
#include "GetMainBoardRequest.h"


// 辅助函数：从 HTTP 请求中提取指定头部的值
std::string getHeaderValue(const std::string &request, const std::string &headerName)
{
    size_t start = request.find(headerName + ": ");
    if (start == std::string::npos)
        return "";

    start += headerName.length() + 2; // 跳过 "headerName: "
    size_t end = request.find("\r\n", start);
    return request.substr(start, end - start);
}

// 解析 HTTP 请求，提取 SOAP 报文体
std::string parseHttpRequest(int clientSocket)
{
    static char buffer[1024 * 10];
    int bytesRead = 0;
    std::string requestBody;

    // 读取 HTTP 请求头部
    while ((bytesRead = read(clientSocket, buffer, sizeof(buffer) - 1)) > 0)
    {
        buffer[bytesRead] = '\0'; // 确保字符串以 null 结束
        requestBody += buffer;

        // 如果读取到了 HTTP 请求的结束标志（\r\n\r\n 表示头部结束）
        if (requestBody.find("\r\n\r\n") != std::string::npos)
            break;
    }

    // 检查是否包含 Expect: 100-continue
    if (requestBody.find("Expect: 100-continue") != std::string::npos)
    {
        // 发送 HTTP/1.1 100 Continue 响应
        const char *continueResponse = "HTTP/1.1 100 Continue\r\n\r\n";
        write(clientSocket, continueResponse, strlen(continueResponse));
    }

    // 查找请求体的起始位置
    size_t bodyStart = requestBody.find("\r\n\r\n") + 4; // 忽略头部
    std::string body = requestBody.substr(bodyStart);

    // 如果请求体不完整，继续读取
    while (body.size() < std::stoi(getHeaderValue(requestBody, "Content-Length")))
    {
        bytesRead = read(clientSocket, buffer, sizeof(buffer) - 1);
        if (bytesRead > 0)
        {
            buffer[bytesRead] = '\0';
            body += buffer;
        }
        else
        {
            break; // 读取结束或出错
        }
    }

    return body;
}

// 处理客户端请求
void handleRequest(int clientSocket)
{
    const char* response = nullptr;

    // 解析请求并获取 SOAP 消息体
    std::string soapMessage = parseHttpRequest(clientSocket);

    // 使用 libxml2 解析 SOAP 报文
    xmlDocPtr doc = xmlReadMemory(soapMessage.c_str(), soapMessage.size(), NULL, NULL, 0);
    if (doc == nullptr)
    {
        std::cerr << "Failed to parse SOAP message" << std::endl;
        close(clientSocket);
        return;
    }

    //获取根节点
    xmlNode* rootElement = xmlDocGetRootElement(doc);
    xmlNode* bodyNode = nullptr;

    // 查找 <s:Body> 节点
    for (xmlNode* node = rootElement->children; node; node = node->next)
    {
        if (node->type == XML_ELEMENT_NODE && xmlStrcmp(node->name, BAD_CAST "Body") == 0)
        {
            bodyNode = node;
            break;
        }
    }

    bool handled = false;     /* 是否已生成自定义响应 */

    // 解析节点
    if (bodyNode)
    {
        for (xmlNode* node = bodyNode->children; node; node = node->next)
        {
            //电机运行控制
            if (node->type == XML_ELEMENT_NODE && xmlStrcmp(node->name, BAD_CAST "ExcuteActions") == 0)
            {
                parseExcuteActions(node->children);
                break;
            }//状态返回
            else if(node->type == XML_ELEMENT_NODE && xmlStrcmp(node->name, BAD_CAST "GetStatusBRequest") == 0)
            {
                GetMotorStateType motor;
                pthread_rwlock_rdlock(&getMotorStateVar_rwlock);
                motor = getMotorStateVar;
                pthread_rwlock_unlock(&getMotorStateVar_rwlock);
                getStatusBRequestResponse(clientSocket,&motor);
                handled = true;
                break;
            }//校准解析
            else if(node->type == XML_ELEMENT_NODE && xmlStrcmp(node->name, BAD_CAST "SaveMachineObject") == 0)
            {
                parseCalibrations(node->children);
                break;
            }//进样流程
            else if(node->type == XML_ELEMENT_NODE && xmlStrcmp(node->name, BAD_CAST "ExecuteSampleBatch") == 0)
            {
                parseLiquidSamplingFlow(node->children);
                break;
            }
            else if(node->type == XML_ELEMENT_NODE && xmlStrcmp(node->name, BAD_CAST "GetMachineObject") == 0)
            {
                parseModuleHeart(clientSocket,node->children);
                handled = true;
                break;
            } //设置模块参数
            else if(node->type == XML_ELEMENT_NODE && xmlStrcmp(node->name, BAD_CAST "SetMachineObject") == 0)
            {

            }
        }
        
    }


    /* 3. 如果前面没特殊内容，就回一个简单 200 OK */
    if (!handled) 
    {
        const char okResp[] =
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: text/xml\r\n"
            "Connection: close\r\n\r\n"
            "<response><status>Success</status></response>\r\n";
        write(clientSocket, okResp, sizeof(okResp)-1);
    }

end:
    // 释放 libxml2 资源
    xmlFreeDoc(doc);
    close(clientSocket);          /* 线程私有 fd 必须也只能关一次 */
    return;
}




// 各模块心跳包
void parseModuleHeart(int clientSocket, xmlNode* node)
{
    xmlNode* cur_node = nullptr;
    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if(cur_node->type == XML_ELEMENT_NODE && xmlStrcmp(cur_node->name, BAD_CAST "reference") == 0)
        {
            xmlChar* descriptionName = xmlGetProp(cur_node, BAD_CAST "descriptionName");

            //孵化器心跳包
            if(xmlStrcmp(descriptionName, BAD_CAST "AgitatorDescription") == 0)
            {
                xmlChar* Name = xmlGetProp(cur_node, BAD_CAST "name");

                if(xmlStrcmp(Name, BAD_CAST "Agitator 1") == 0)
                {
                    GetIncubationRequestType tempData;

                    pthread_rwlock_rdlock(&getIncubationRequestTypeVar_rwlock);
                    tempData = getIncubationRequestTypeVar;
                    pthread_rwlock_unlock(&getIncubationRequestTypeVar_rwlock);
                    //发送SOAP应答
                    getIncubationRequestResponse(clientSocket, &tempData);
                }

                xmlFree(Name);
            }
            else if(xmlStrcmp(descriptionName, BAD_CAST "ToolDescription") == 0) //Z轴板进样工具心跳包 
            {
                //xmlChar* Name = xmlGetProp(cur_node, BAD_CAST "name");

                //if(xmlStrcmp(Name, BAD_CAST "Agitator 1") == 0)
                {
                    
                    GetZboardRequestType tempData;

                    pthread_rwlock_rdlock(&getZboardRequestTypeVar_rwlock);
                    tempData = getZboardRequestTypeVar;
                    pthread_rwlock_unlock(&getZboardRequestTypeVar_rwlock);
                    //发送SOAP应答
                    getZboardRequestResponse(clientSocket, &tempData);
                    
                    //xmlFree(Name);
                    //xmlFree(descriptionName);
                    //break;
                }

                //xmlFree(Name);                
            }
            //老化模块心跳包
            else if(xmlStrcmp(descriptionName, BAD_CAST "BurninDescription") == 0)
            {
                xmlChar* Name = xmlGetProp(cur_node, BAD_CAST "name");

                if(xmlStrcmp(Name, BAD_CAST "Burn-In 1") == 0)
                {
                    GetMainBoardRequestType tempData;

                    pthread_rwlock_rdlock(&getMainBoardRequestTypeVar_rwlock);
                    tempData = getMainBoardRequestTypeVar;
                    pthread_rwlock_unlock(&getMainBoardRequestTypeVar_rwlock);
                    //发送SOAP应答
                    getAgligRequestResponse(clientSocket, &tempData);
                    
                }

                xmlFree(Name);
            }

            xmlFree(descriptionName);
            break;
        }

        parseExcuteActions(cur_node->children);  // 递归解析子节点
    }
}


//设置参数协议解析
void parseSetMachineObject(int clientSocket, xmlNode* node)
{
    xmlNode* cur_node = nullptr;
    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if(cur_node->type == XML_ELEMENT_NODE && xmlStrcmp(cur_node->name, BAD_CAST "machineObjectData") == 0)
        {
            xmlChar* descriptionName = xmlGetProp(cur_node, BAD_CAST "descriptionName");

            //孵化器参数设置
            if(xmlStrcmp(descriptionName, BAD_CAST "AgitatorDescription") == 0)
            {
                xmlChar* Name = xmlGetProp(cur_node, BAD_CAST "name");

                if(xmlStrcmp(Name, BAD_CAST "Agitator 1") == 0)
                {
                    GetIncubationRequestType tempData;

                    pthread_rwlock_rdlock(&getIncubationRequestTypeVar_rwlock);
                    tempData = getIncubationRequestTypeVar;
                    pthread_rwlock_unlock(&getIncubationRequestTypeVar_rwlock);
                    //发送SOAP应答
                    getIncubationRequestResponse(clientSocket, &tempData);
                }

                xmlFree(Name);
            }
            else if(xmlStrcmp(descriptionName, BAD_CAST "ToolDescription") == 0) //Z轴板进样工具心跳包 
            {
                //xmlChar* Name = xmlGetProp(cur_node, BAD_CAST "name");

                //if(xmlStrcmp(Name, BAD_CAST "Agitator 1") == 0)
                {
                    
                    GetZboardRequestType tempData;

                    pthread_rwlock_rdlock(&getZboardRequestTypeVar_rwlock);
                    tempData = getZboardRequestTypeVar;
                    pthread_rwlock_unlock(&getZboardRequestTypeVar_rwlock);
                    //发送SOAP应答
                    getZboardRequestResponse(clientSocket, &tempData);
                    
                    //xmlFree(Name);
                    //xmlFree(descriptionName);
                    //break;
                }

                //xmlFree(Name);                
            }
            //老化模块心跳包
            else if(xmlStrcmp(descriptionName, BAD_CAST "BurninDescription") == 0)
            {
                xmlChar* Name = xmlGetProp(cur_node, BAD_CAST "name");

                if(xmlStrcmp(Name, BAD_CAST "Burn-In 1") == 0)
                {
                    GetMainBoardRequestType tempData;

                    pthread_rwlock_rdlock(&getMainBoardRequestTypeVar_rwlock);
                    tempData = getMainBoardRequestTypeVar;
                    pthread_rwlock_unlock(&getMainBoardRequestTypeVar_rwlock);
                    //发送SOAP应答
                    getAgligRequestResponse(clientSocket, &tempData);
                    
                }

                xmlFree(Name);
            }

            xmlFree(descriptionName);
            break;
        }
        else if(cur_node->type == XML_ELEMENT_NODE && xmlStrcmp(cur_node->name, BAD_CAST "IntParameter") == 0)
        {
            xmlChar* name = xmlGetProp(cur_node, BAD_CAST "name");
            xmlChar* value = xmlGetProp(cur_node, BAD_CAST "value");
            xmlChar* enable = xmlGetProp(cur_node, BAD_CAST "enable");

            if(xmlStrcmp(name, BAD_CAST "Speed") == 0)
            {
                if(xmlStrcmp(enable, BAD_CAST "True") == 0)
                {
                    //电机驱动控制
                    IncubationMotorReset(1, atoi(reinterpret_cast<char*>(value)));
                }
            }

            xmlFree(name);
            xmlFree(value);
            xmlFree(enable);
        }
        else if(cur_node->type == XML_ELEMENT_NODE && xmlStrcmp(cur_node->name, BAD_CAST "BooleanParameter") == 0)
        {
            xmlChar* name = xmlGetProp(cur_node, BAD_CAST "name");
            xmlChar* value = xmlGetProp(cur_node, BAD_CAST "value");
            xmlChar* enable = xmlGetProp(cur_node, BAD_CAST "enable");

            if(xmlStrcmp(name, BAD_CAST "HeatAgitator") == 0) //孵化器加热
            {
                if(xmlStrcmp(enable, BAD_CAST "True") == 0)
                {
                    //开启孵化器加热
                    IncubationMotorReset(1, atoi(reinterpret_cast<char*>(value)));
                }
            }

            xmlFree(name);
            xmlFree(value);
            xmlFree(enable);
        }
        else if(cur_node->type == XML_ELEMENT_NODE && xmlStrcmp(cur_node->name, BAD_CAST "DoubleUnitParameter") == 0)
        {
            xmlChar* name = xmlGetProp(cur_node, BAD_CAST "name");
            xmlChar* value = xmlGetProp(cur_node, BAD_CAST "value");
            xmlChar* enable = xmlGetProp(cur_node, BAD_CAST "enable");

            if(xmlStrcmp(name, BAD_CAST "P") == 0) //温控P
            {
                if(xmlStrcmp(enable, BAD_CAST "True") == 0)
                {
                    //开启孵化器加热
                    IncubationMotorReset(1, atoi(reinterpret_cast<char*>(value)));
                }
            }
            else if(xmlStrcmp(name, BAD_CAST "I") == 0) //温控I
            {
                if(xmlStrcmp(enable, BAD_CAST "True") == 0)
                {
                    //开启孵化器加热
                    IncubationMotorReset(1, atoi(reinterpret_cast<char*>(value)));
                }
            }
            else if(xmlStrcmp(name, BAD_CAST "D") == 0) //温控D
            {
                if(xmlStrcmp(enable, BAD_CAST "True") == 0)
                {
                    //开启孵化器加热
                    IncubationMotorReset(1, atoi(reinterpret_cast<char*>(value)));
                }
            }

            xmlFree(name);
            xmlFree(value);
            xmlFree(enable);
        }

        parseExcuteActions(cur_node->children);  // 递归解析子节点
    }
}



// 电机动作
void parseExcuteActions(xmlNode* node)
{
    static ExcuteActionsType tempData;
    xmlNode* cur_node = nullptr;
    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if(cur_node->type == XML_ELEMENT_NODE && xmlStrcmp(cur_node->name, BAD_CAST "actionsParameter") == 0)
        {
            xmlChar* name = xmlGetProp(cur_node, BAD_CAST "name");

            //电机动作
            if(xmlStrcmp(name, BAD_CAST "DistanceMotion") == 0)
            {
                
                tempData.en_motion = en_MOVE;
            }
            else if(xmlStrcmp(name, BAD_CAST "StopMotion") == 0)
            {
                tempData.en_motion = en_STOP;
            }
            else if(xmlStrcmp(name, BAD_CAST "ResetAgitator") == 0) //孵化器复位
            {

            }
            else if(xmlStrcmp(name, BAD_CAST "Shutdown") == 0) //关机
            {
                std::strcpy(tempData.name,"Shutdown");
                printf("接收关机指令\r\n");

                // 消息队列发送
                msgType temp;
                temp.msgtype = 1;
                //序列化
                serialize(tempData, temp.msgdata, sizeof(temp.msgdata));

                if (msgsnd(msgExcuteActionsID, &temp, sizeof(tempData), IPC_NOWAIT) == -1) //IPC_NOWAIT
                {
                    perror("ExcuteActions msgsnd failed");
                }

                std::memset(reinterpret_cast<char*>(&tempData), 0, sizeof(tempData));

                xmlFree(name);
                break;
            }
            else if(xmlStrcmp(name, BAD_CAST "Standby") == 0) //待机
            {
                std::strcpy(tempData.name,"Standby");

                printf("接收待机指令\r\n");

                // 消息队列发送
                msgType temp;
                temp.msgtype = 1;
                //序列化
                serialize(tempData, temp.msgdata, sizeof(temp.msgdata));

                if (msgsnd(msgExcuteActionsID, &temp, sizeof(tempData), IPC_NOWAIT) == -1) //IPC_NOWAIT
                {
                    perror("ExcuteActions msgsnd failed");
                }

                std::memset(reinterpret_cast<char*>(&tempData), 0, sizeof(tempData));

                xmlFree(name);
                break;
            }
            else if(xmlStrcmp(name, BAD_CAST "EmoStop") == 0) //急停
            {
                std::strcpy(tempData.name,"EmoStop");

                printf("接收急停指令\r\n");

                // 消息队列发送
                msgType temp;
                temp.msgtype = 1;
                //序列化
                serialize(tempData, temp.msgdata, sizeof(temp.msgdata));

                if (msgsnd(msgExcuteActionsID, &temp, sizeof(tempData), IPC_NOWAIT) == -1) //IPC_NOWAIT
                {
                    perror("ExcuteActions msgsnd failed");
                }

                std::memset(reinterpret_cast<char*>(&tempData), 0, sizeof(tempData));

                xmlFree(name);
                break;
            }
            else if(xmlStrcmp(name, BAD_CAST "ChangeTool") == 0) //换针
            {
                std::strcpy(tempData.name,"ChangeTool");

                printf("接收换针指令\r\n");

                // 消息队列发送
                msgType temp;
                temp.msgtype = 1;
                //序列化
                serialize(tempData, temp.msgdata, sizeof(temp.msgdata));

                if (msgsnd(msgExcuteActionsID, &temp, sizeof(tempData), IPC_NOWAIT) == -1) //IPC_NOWAIT
                {
                    perror("ExcuteActions msgsnd failed");
                }

                std::memset(reinterpret_cast<char*>(&tempData), 0, sizeof(tempData));

                xmlFree(name);
                break;
            }
            else if(xmlStrcmp(name, BAD_CAST "ChangeToolEnd") == 0) //换针结束
            {
                std::strcpy(tempData.name,"ChangeToolEnd");

                printf("接收换针结束指令\r\n");

                // 消息队列发送
                msgType temp;
                temp.msgtype = 1;
                //序列化
                serialize(tempData, temp.msgdata, sizeof(temp.msgdata));

                if (msgsnd(msgExcuteActionsID, &temp, sizeof(tempData), IPC_NOWAIT) == -1) //IPC_NOWAIT
                {
                    perror("ExcuteActions msgsnd failed");
                }

                std::memset(reinterpret_cast<char*>(&tempData), 0, sizeof(tempData));

                xmlFree(name);
                break;
            }         

            //std::cout << "action:" << name << std::endl;

            xmlFree(name);
        }
        else if(cur_node->type == XML_ELEMENT_NODE && xmlStrcmp(cur_node->name, BAD_CAST "StringParameter") == 0)
        {
            xmlChar* value = xmlGetProp(cur_node, BAD_CAST "value");

            if(xmlStrcmp(value, BAD_CAST "x") == 0)
            {
                //电机编号
                tempData.en_axis = en_X;
            }
            else if(xmlStrcmp(value, BAD_CAST "y") == 0)
            {
                //电机编号
                tempData.en_axis = en_Y;
            }
            else if(xmlStrcmp(value, BAD_CAST "z") == 0)
            {
                //电机编号
                tempData.en_axis = en_Z;
            }
            else if(xmlStrcmp(value, BAD_CAST "o") == 0)
            {
                //电机编号
                tempData.en_axis = en_O;
            }            

            //std::cout << "axis:" << value << std::endl;

            xmlFree(value);
        }
        else if(cur_node->type == XML_ELEMENT_NODE && xmlStrcmp(cur_node->name, BAD_CAST "DoubleUnitParameter") == 0)
        {
            xmlChar* name = xmlGetProp(cur_node, BAD_CAST "name");
            xmlChar* value = xmlGetProp(cur_node, BAD_CAST "value");

            if(xmlStrcmp(name, BAD_CAST "distance") == 0)
            {
                //移动距离
                tempData.d_distance = atof(reinterpret_cast<char*>(value));
                if(tempData.en_axis == en_X)
                {
                    if(tempData.d_distance >= X_LENGTHS)
                    {
                        tempData.d_distance = X_LENGTHS;
                    }
                    else if(tempData.d_distance <= -X_LENGTHS)
                    {
                        tempData.d_distance = -X_LENGTHS;
                    }
                }
                else if(tempData.en_axis == en_Y)
                {
                    if(tempData.d_distance >= Y_LENGTHS)
                    {
                        tempData.d_distance = Y_LENGTHS;
                    }
                    else if(tempData.d_distance <= -Y_LENGTHS)
                    {
                        tempData.d_distance = -Y_LENGTHS;
                    }
                }
                else if(tempData.en_axis == en_Z)
                {
                    if(tempData.d_distance >= Z_LENGTHS)
                    {
                        tempData.d_distance = Z_LENGTHS;
                    }
                    else if(tempData.d_distance <= -Z_LENGTHS)
                    {
                        tempData.d_distance = -Z_LENGTHS;
                    }
                }

                std::cout << "distance:" << tempData.d_distance << std::endl;
            }
            else if(xmlStrcmp(name, BAD_CAST "speed") == 0)
            {
                //移动速度
                tempData.d_speed = atof(reinterpret_cast<char*>(value));
                //std::cout << "speed:" << tempData.d_speed << std::endl;

                // 消息队列发送
                msgType temp;
                temp.msgtype = 1;
                //序列化
                serialize(tempData, temp.msgdata, sizeof(temp.msgdata));

                if (msgsnd(msgExcuteActionsID, &temp, sizeof(tempData), IPC_NOWAIT) == -1) //IPC_NOWAIT
                {
                    perror("ExcuteActions msgsnd failed");
                }

                std::memset(reinterpret_cast<char*>(&tempData), 0, sizeof(tempData));
            }

            xmlFree(name);
            xmlFree(value);
        }
        else if(cur_node->type == XML_ELEMENT_NODE && xmlStrcmp(cur_node->name, BAD_CAST "IntParameter") == 0)
        {
            xmlChar* name = xmlGetProp(cur_node, BAD_CAST "name");
            xmlChar* value = xmlGetProp(cur_node, BAD_CAST "value");
            xmlChar* enable = xmlGetProp(cur_node, BAD_CAST "enable");

            if(xmlStrcmp(name, BAD_CAST "offset") == 0)
            {
                if(xmlStrcmp(enable, BAD_CAST "True") == 0)
                {
                    IncubationMotorReset(1, atoi(reinterpret_cast<char*>(value)));
                }
            }

            xmlFree(name);
            xmlFree(value);
            xmlFree(enable);
        }

        parseExcuteActions(cur_node->children);  // 递归解析子节点
    }
}


//校准解析
void parseCalibrations(xmlNode* node)
{
    static CalibrationsType tempData;
    static uint8_t step = 0;
    xmlNode* cur_node = nullptr;

    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if(cur_node->type == XML_ELEMENT_NODE && xmlStrcmp(cur_node->name, BAD_CAST "machineObjectData") == 0)
        {
            xmlChar* descriptionName = xmlGetProp(cur_node, BAD_CAST "descriptionName");
            xmlChar* name = xmlGetProp(cur_node, BAD_CAST "name");
            
            if(xmlStrcmp(descriptionName, BAD_CAST "AgitatorDescription") == 0) //孵化模块
            {
                std::strcpy(tempData.RackName, reinterpret_cast<char*>(name));
                printf("AgitatorDescription name = %s\r\n", reinterpret_cast<char*>(name)); 
            }
            else if(xmlStrcmp(descriptionName, BAD_CAST "RackDescription") == 0) //样品盘
            {
                std::strcpy(tempData.RackName, reinterpret_cast<char*>(name));
                printf("RackDescription name = %s\r\n", reinterpret_cast<char*>(name)); 
            }
            else if(xmlStrcmp(descriptionName, BAD_CAST "RinseStationDescription") == 0) //冲洗站
            {
                std::strcpy(tempData.RackName, reinterpret_cast<char*>(name));
                printf("RinseStationDescription name = %s\r\n", reinterpret_cast<char*>(name)); 
            }
            else if(xmlStrcmp(descriptionName, BAD_CAST "InletDescription") == 0) //注入口
            {
                std::strcpy(tempData.RackName, reinterpret_cast<char*>(name));
                printf("InletDescription name = %s\r\n", reinterpret_cast<char*>(name));
            }
            else if(xmlStrcmp(descriptionName, BAD_CAST "BurninDescription") == 0) //老化口
            {
                std::strcpy(tempData.RackName, reinterpret_cast<char*>(name));
                printf("BurninDescription name = %s\r\n", reinterpret_cast<char*>(name));
            }
            else if(xmlStrcmp(descriptionName, BAD_CAST "HomeDescription") == 0) //HOME位
            {
                std::strcpy(tempData.RackName, reinterpret_cast<char*>(name));
                printf("HomeDescription name = %s\r\n", reinterpret_cast<char*>(name));
            }
            else if(xmlStrcmp(descriptionName, BAD_CAST "ChangeToolDescription") == 0) //换针位
            {
                std::strcpy(tempData.RackName, reinterpret_cast<char*>(name));
                printf("ChangeToolDescription name = %s\r\n", reinterpret_cast<char*>(name));
            }
            else if(xmlStrcmp(descriptionName, BAD_CAST "ShutdownDescription") == 0) //关机位
            {
                std::strcpy(tempData.RackName, reinterpret_cast<char*>(name));
                printf("ShutdownDescription name = %s\r\n", reinterpret_cast<char*>(name));
            }
            else if(xmlStrcmp(descriptionName, BAD_CAST "ToolDescription") == 0) //进样工具
            {
                std::strcpy(tempData.RackName, reinterpret_cast<char*>(name));
                printf("ToolDescription name = %s\r\n", reinterpret_cast<char*>(name));
            }

            xmlFree(descriptionName);
            xmlFree(name);
        }
        else if(cur_node->type == XML_ELEMENT_NODE && xmlStrcmp(cur_node->name, BAD_CAST "Reference") == 0)
        {
            xmlChar* value = xmlGetProp(cur_node, BAD_CAST "name");
         
            if(xmlStrcmp(value, BAD_CAST "VT15") == 0)
            {
                //大瓶样品盘
                std::strcpy(tempData.RackType, "VT15");
            }
            else if(xmlStrcmp(value, BAD_CAST "VT54") == 0)
            {
                //小瓶样品盘
                std::strcpy(tempData.RackType, "VT54");
            }
            else if(xmlStrcmp(value, BAD_CAST "RT5") == 0)
            {
                //冲洗站
                std::strcpy(tempData.RackType, "RT5");
            }

            xmlFree(value);
        }
        else if(cur_node->type == XML_ELEMENT_NODE && xmlStrcmp(cur_node->name, BAD_CAST "IntParameter") == 0)
        {
            xmlChar* name = xmlGetProp(cur_node, BAD_CAST "name");

            if(xmlStrcmp(name, BAD_CAST "CapType") == 0)
            {
               xmlChar* value = xmlGetProp(cur_node, BAD_CAST "value");

                //瓶盖类型
                if(xmlStrcmp(value, BAD_CAST "1") == 0)
                {
                    std::strcpy(tempData.CapType, "1");  
                }
                else if(xmlStrcmp(value, BAD_CAST "2") == 0)
                {
                    std::strcpy(tempData.CapType, "2");
                }

                xmlFree(value);
            }

            xmlFree(name);
        }
        else if(cur_node->type == XML_ELEMENT_NODE && xmlStrcmp(cur_node->name, BAD_CAST "DoubleParameter") == 0)
        {
            xmlChar* name = xmlGetProp(cur_node, BAD_CAST "name");

            if(xmlStrcmp(name, BAD_CAST "FiberLength") == 0)
            {
               xmlChar* value = xmlGetProp(cur_node, BAD_CAST "value");
               xmlChar* enable = xmlGetProp(cur_node, BAD_CAST "enable");

                //SPME工具纤维头长度
                if(xmlStrcmp(enable, BAD_CAST "True") == 0)
                {
                    float temp = atof(reinterpret_cast<char*>(value));
                    write_key_value(CHANGE_TOOL_PATH, "FiberLength", temp); 
                }

                xmlFree(enable);
                xmlFree(value);
            }

            xmlFree(name);
        }
        else if(cur_node->type == XML_ELEMENT_NODE && xmlStrcmp(cur_node->name, BAD_CAST "TeachPoints") == 0)
        {
            //冲洗站
            if(std::strcmp(tempData.RackName, "Rinse Station 1") == 0)
            {
                xmlNode *ChildNode = cur_node->children;
                while(ChildNode)
                {
                    if(ChildNode->type == XML_ELEMENT_NODE && xmlStrcmp(ChildNode->name, BAD_CAST "Vector") == 0)
                    {
                        xmlChar* x = xmlGetProp(ChildNode, BAD_CAST "x");
                        xmlChar* y = xmlGetProp(ChildNode, BAD_CAST "y");
                        xmlChar* z = xmlGetProp(ChildNode, BAD_CAST "z");

                        tempData.PointUpLeft.x = atof(reinterpret_cast<char*>(x));
                        tempData.PointUpLeft.y = atof(reinterpret_cast<char*>(y));
                        tempData.PointUpLeft.z = atof(reinterpret_cast<char*>(z));

                        printf("Rinse Station 1 x = %f\r\n", tempData.PointUpLeft.x);
                        printf("Rinse Station 1 y = %f\r\n", tempData.PointUpLeft.y);
                        printf("Rinse Station 1 z = %f\r\n", tempData.PointUpLeft.z);

                        // 消息队列发送
                        msgType temp;
                        temp.msgtype = 1;
                        //序列化
                        serialize(tempData, temp.msgdata, sizeof(temp.msgdata));

                        if (msgsnd(msgCalibrationsID, &temp, sizeof(tempData), 0) == -1) //IPC_NOWAIT
                        {
                            perror("Calibrations msgsnd failed");
                        }
                        
                        std::memset(reinterpret_cast<char*>(&tempData), 0, sizeof(tempData));

                        xmlFree(x);
                        xmlFree(y);
                        xmlFree(z);
                        break;
                    }

                    ChildNode = ChildNode->next; 
                }

            }
            //HOME位
            else if(std::strcmp(tempData.RackName, "Home") == 0)
            {
                xmlNode *ChildNode = cur_node->children;
                while(ChildNode)
                {
                    if(ChildNode->type == XML_ELEMENT_NODE && xmlStrcmp(ChildNode->name, BAD_CAST "Vector") == 0)
                    {
                        xmlChar* x = xmlGetProp(ChildNode, BAD_CAST "x");
                        xmlChar* y = xmlGetProp(ChildNode, BAD_CAST "y");
                        xmlChar* z = xmlGetProp(ChildNode, BAD_CAST "z");

                        tempData.PointUpLeft.x = atof(reinterpret_cast<char*>(x));
                        tempData.PointUpLeft.y = atof(reinterpret_cast<char*>(y));
                        tempData.PointUpLeft.z = atof(reinterpret_cast<char*>(z));

                        printf("Home x = %f\r\n", tempData.PointUpLeft.x);
                        printf("Home y = %f\r\n", tempData.PointUpLeft.y);
                        printf("Home z = %f\r\n", tempData.PointUpLeft.z);

                        // 消息队列发送
                        msgType temp;
                        temp.msgtype = 1;
                        //序列化
                        serialize(tempData, temp.msgdata, sizeof(temp.msgdata));

                        if (msgsnd(msgCalibrationsID, &temp, sizeof(tempData), 0) == -1) //IPC_NOWAIT
                        {
                            perror("Calibrations msgsnd failed");
                        }
                        
                        std::memset(reinterpret_cast<char*>(&tempData), 0, sizeof(tempData));

                        xmlFree(x);
                        xmlFree(y);
                        xmlFree(z);
                        break;
                    }

                    ChildNode = ChildNode->next; 
                }

            }
            //换针位
            else if(std::strcmp(tempData.RackName, "ChangeTool") == 0)
            {
                xmlNode *ChildNode = cur_node->children;
                while(ChildNode)
                {
                    if(ChildNode->type == XML_ELEMENT_NODE && xmlStrcmp(ChildNode->name, BAD_CAST "Vector") == 0)
                    {
                        xmlChar* x = xmlGetProp(ChildNode, BAD_CAST "x");
                        xmlChar* y = xmlGetProp(ChildNode, BAD_CAST "y");
                        xmlChar* z = xmlGetProp(ChildNode, BAD_CAST "z");

                        tempData.PointUpLeft.x = atof(reinterpret_cast<char*>(x));
                        tempData.PointUpLeft.y = atof(reinterpret_cast<char*>(y));
                        tempData.PointUpLeft.z = atof(reinterpret_cast<char*>(z));

                        printf("ChangeTool x = %f\r\n", tempData.PointUpLeft.x);
                        printf("ChangeTool y = %f\r\n", tempData.PointUpLeft.y);
                        printf("ChangeTool z = %f\r\n", tempData.PointUpLeft.z);

                        // 消息队列发送
                        msgType temp;
                        temp.msgtype = 1;
                        //序列化
                        serialize(tempData, temp.msgdata, sizeof(temp.msgdata));

                        if (msgsnd(msgCalibrationsID, &temp, sizeof(tempData), 0) == -1) //IPC_NOWAIT
                        {
                            perror("Calibrations msgsnd failed");
                        }
                        
                        std::memset(reinterpret_cast<char*>(&tempData), 0, sizeof(tempData));

                        xmlFree(x);
                        xmlFree(y);
                        xmlFree(z);
                        break;
                    }

                    ChildNode = ChildNode->next; 
                }

            }
            //关机位
            else if(std::strcmp(tempData.RackName, "Shutdown") == 0)
            {
                xmlNode *ChildNode = cur_node->children;
                while(ChildNode)
                {
                    if(ChildNode->type == XML_ELEMENT_NODE && xmlStrcmp(ChildNode->name, BAD_CAST "Vector") == 0)
                    {
                        xmlChar* x = xmlGetProp(ChildNode, BAD_CAST "x");
                        xmlChar* y = xmlGetProp(ChildNode, BAD_CAST "y");
                        xmlChar* z = xmlGetProp(ChildNode, BAD_CAST "z");

                        tempData.PointUpLeft.x = atof(reinterpret_cast<char*>(x));
                        tempData.PointUpLeft.y = atof(reinterpret_cast<char*>(y));
                        tempData.PointUpLeft.z = atof(reinterpret_cast<char*>(z));

                        printf("Shutdown x = %f\r\n", tempData.PointUpLeft.x);
                        printf("Shutdown y = %f\r\n", tempData.PointUpLeft.y);
                        printf("Shutdown z = %f\r\n", tempData.PointUpLeft.z);

                        // 消息队列发送
                        msgType temp;
                        temp.msgtype = 1;
                        //序列化
                        serialize(tempData, temp.msgdata, sizeof(temp.msgdata));

                        if (msgsnd(msgCalibrationsID, &temp, sizeof(tempData), 0) == -1) //IPC_NOWAIT
                        {
                            perror("Calibrations msgsnd failed");
                        }
                        
                        std::memset(reinterpret_cast<char*>(&tempData), 0, sizeof(tempData));

                        xmlFree(x);
                        xmlFree(y);
                        xmlFree(z);
                        break;
                    }

                    ChildNode = ChildNode->next; 
                }

            }
            //注入口
            else if(std::strcmp(tempData.RackName, "Inlet 1") == 0)
            {
                xmlNode *ChildNode = cur_node->children;
                while(ChildNode)
                {
                    if(ChildNode->type == XML_ELEMENT_NODE && xmlStrcmp(ChildNode->name, BAD_CAST "Vector") == 0)
                    {
                        xmlChar* x = xmlGetProp(ChildNode, BAD_CAST "x");
                        xmlChar* y = xmlGetProp(ChildNode, BAD_CAST "y");
                        xmlChar* z = xmlGetProp(ChildNode, BAD_CAST "z");

                        tempData.PointUpLeft.x = atof(reinterpret_cast<char*>(x));
                        tempData.PointUpLeft.y = atof(reinterpret_cast<char*>(y));
                        tempData.PointUpLeft.z = atof(reinterpret_cast<char*>(z));

                        printf("Inlet 1 x = %f\r\n", tempData.PointUpLeft.x);
                        printf("Inlet 1 y = %f\r\n", tempData.PointUpLeft.y);
                        printf("Inlet 1 z = %f\r\n", tempData.PointUpLeft.z);

                        // 消息队列发送
                        msgType temp;
                        temp.msgtype = 1;
                        //序列化
                        serialize(tempData, temp.msgdata, sizeof(temp.msgdata));

                        if (msgsnd(msgCalibrationsID, &temp, sizeof(tempData), 0) == -1) //IPC_NOWAIT
                        {
                            perror("Calibrations msgsnd failed");
                        }
                        
                        std::memset(reinterpret_cast<char*>(&tempData), 0, sizeof(tempData));

                        xmlFree(x);
                        xmlFree(y);
                        xmlFree(z);
                        break;
                    }

                    ChildNode = ChildNode->next; 
                }
            }
            //孵化器
            else if(std::strcmp(tempData.RackName, "Agitator 1") == 0)
            {
                xmlNode *ChildNode = cur_node->children;
                while(ChildNode)
                {
                    if(ChildNode->type == XML_ELEMENT_NODE && xmlStrcmp(ChildNode->name, BAD_CAST "Vector") == 0)
                    {
                        xmlChar* x = xmlGetProp(ChildNode, BAD_CAST "x");
                        xmlChar* y = xmlGetProp(ChildNode, BAD_CAST "y");
                        xmlChar* z = xmlGetProp(ChildNode, BAD_CAST "z");

                        tempData.PointUpLeft.x = atof(reinterpret_cast<char*>(x));
                        tempData.PointUpLeft.y = atof(reinterpret_cast<char*>(y));
                        tempData.PointUpLeft.z = atof(reinterpret_cast<char*>(z));

                        printf("Agitator 1 x = %f\r\n", tempData.PointUpLeft.x);
                        printf("Agitator 1 y = %f\r\n", tempData.PointUpLeft.y);
                        printf("Agitator 1 z = %f\r\n", tempData.PointUpLeft.z);

                        // 消息队列发送
                        msgType temp;
                        temp.msgtype = 1;
                        //序列化
                        serialize(tempData, temp.msgdata, sizeof(temp.msgdata));

                        if (msgsnd(msgCalibrationsID, &temp, sizeof(tempData), 0) == -1) //IPC_NOWAIT
                        {
                            perror("Agitator 1 msgsnd failed");
                        }
                        
                        std::memset(reinterpret_cast<char*>(&tempData), 0, sizeof(tempData));

                        xmlFree(x);
                        xmlFree(y);
                        xmlFree(z);
                        break;
                    }

                    ChildNode = ChildNode->next; 
                }
            }
            //老化口
            else if(std::strcmp(tempData.RackName, "Burn-In 1") == 0)
            {
                xmlNode *ChildNode = cur_node->children;
                while(ChildNode)
                {
                    if(ChildNode->type == XML_ELEMENT_NODE && xmlStrcmp(ChildNode->name, BAD_CAST "Vector") == 0)
                    {
                        xmlChar* x = xmlGetProp(ChildNode, BAD_CAST "x");
                        xmlChar* y = xmlGetProp(ChildNode, BAD_CAST "y");
                        xmlChar* z = xmlGetProp(ChildNode, BAD_CAST "z");

                        tempData.PointUpLeft.x = atof(reinterpret_cast<char*>(x));
                        tempData.PointUpLeft.y = atof(reinterpret_cast<char*>(y));
                        tempData.PointUpLeft.z = atof(reinterpret_cast<char*>(z));

                        printf("Burn-In 1 x = %f\r\n", tempData.PointUpLeft.x);
                        printf("Burn-In 1 y = %f\r\n", tempData.PointUpLeft.y);
                        printf("Burn-In 1 z = %f\r\n", tempData.PointUpLeft.z);

                        // 消息队列发送
                        msgType temp;
                        temp.msgtype = 1;
                        //序列化
                        serialize(tempData, temp.msgdata, sizeof(temp.msgdata));

                        if (msgsnd(msgCalibrationsID, &temp, sizeof(tempData), 0) == -1) //IPC_NOWAIT
                        {
                            perror("Burn-In 1 msgsnd failed");
                        }    
                        
                        std::memset(reinterpret_cast<char*>(&tempData), 0, sizeof(tempData));

                        xmlFree(x);
                        xmlFree(y);
                        xmlFree(z);
                        break;
                    }

                    ChildNode = ChildNode->next; 
                }
            }
            else if(std::strcmp(tempData.RackName, "Rack 1") == 0)
            {
                xmlNode *vectorNode = cur_node->children;
                while(vectorNode)
                {
                    if(vectorNode->type == XML_ELEMENT_NODE && xmlStrcmp(vectorNode->name, BAD_CAST "Vector") == 0)
                    {
                        xmlChar* x = xmlGetProp(vectorNode, BAD_CAST "x");
                        xmlChar* y = xmlGetProp(vectorNode, BAD_CAST "y");
                        xmlChar* z = xmlGetProp(vectorNode, BAD_CAST "z");
                        if(step == 0)
                        {
                            tempData.PointUpLeft.x = atof(reinterpret_cast<char*>(x));
                            tempData.PointUpLeft.y = atof(reinterpret_cast<char*>(y));
                            tempData.PointUpLeft.z = atof(reinterpret_cast<char*>(z));
                            printf("Vector x = %f\r\n", tempData.PointUpLeft.x);
                            printf("Vector y = %f\r\n", tempData.PointUpLeft.y);
                            printf("Vector z = %f\r\n", tempData.PointUpLeft.z);
                            step++;
                        }
                        else if(step == 1)
                        {
                            tempData.PointLowLeft.x = atof(reinterpret_cast<char*>(x));
                            tempData.PointLowLeft.y = atof(reinterpret_cast<char*>(y));
                            tempData.PointLowLeft.z = atof(reinterpret_cast<char*>(z));
                            step++;
                        }
                        else if(step == 2)
                        {
                            tempData.PointLowRight.x = atof(reinterpret_cast<char*>(x));
                            tempData.PointLowRight.y = atof(reinterpret_cast<char*>(y));
                            tempData.PointLowRight.z = atof(reinterpret_cast<char*>(z));
                            
                            // 消息队列发送
                            msgType temp;
                            temp.msgtype = 1;
                            //序列化
                            serialize(tempData, temp.msgdata, sizeof(temp.msgdata));

                            if (msgsnd(msgCalibrationsID, &temp, sizeof(tempData), 0) == -1) //IPC_NOWAIT
                            {
                                perror("Calibrations msgsnd failed");
                            }

                            std::memset(reinterpret_cast<char*>(&tempData), 0, sizeof(tempData));

                            step = 0;
                        }
                        
                        xmlFree(x);
                        xmlFree(y);
                        xmlFree(z);
                    }
                    vectorNode = vectorNode->next;
                }
            }
            else if(std::strcmp(tempData.RackName, "Rack 2") == 0)
            {
                xmlNode *vectorNode = cur_node->children;
                while(vectorNode)
                {
                    if(vectorNode->type == XML_ELEMENT_NODE && xmlStrcmp(vectorNode->name, BAD_CAST "Vector") == 0)
                    {
                        xmlChar* x = xmlGetProp(vectorNode, BAD_CAST "x");
                        xmlChar* y = xmlGetProp(vectorNode, BAD_CAST "y");
                        xmlChar* z = xmlGetProp(vectorNode, BAD_CAST "z");
                        if(step == 0)
                        {
                            tempData.PointUpLeft.x = atof(reinterpret_cast<char*>(x));
                            tempData.PointUpLeft.y = atof(reinterpret_cast<char*>(y));
                            tempData.PointUpLeft.z = atof(reinterpret_cast<char*>(z));
                            printf("Vector x = %f\r\n", tempData.PointUpLeft.x);
                            printf("Vector y = %f\r\n", tempData.PointUpLeft.y);
                            printf("Vector z = %f\r\n", tempData.PointUpLeft.z);
                            step++;
                        }
                        else if(step == 1)
                        {
                            tempData.PointLowLeft.x = atof(reinterpret_cast<char*>(x));
                            tempData.PointLowLeft.y = atof(reinterpret_cast<char*>(y));
                            tempData.PointLowLeft.z = atof(reinterpret_cast<char*>(z));
                            step++;
                        }
                        else if(step == 2)
                        {
                            tempData.PointLowRight.x = atof(reinterpret_cast<char*>(x));
                            tempData.PointLowRight.y = atof(reinterpret_cast<char*>(y));
                            tempData.PointLowRight.z = atof(reinterpret_cast<char*>(z));
                            
                            // 消息队列发送
                            msgType temp;
                            temp.msgtype = 1;
                            //序列化
                            serialize(tempData, temp.msgdata, sizeof(temp.msgdata));

                            if (msgsnd(msgCalibrationsID, &temp, sizeof(tempData), 0) == -1) //IPC_NOWAIT
                            {
                                perror("Calibrations msgsnd failed");
                            }

                            std::memset(reinterpret_cast<char*>(&tempData), 0, sizeof(tempData));

                            step = 0;
                        }
                        
                        xmlFree(x);
                        xmlFree(y);
                        xmlFree(z);
                    }
                    vectorNode = vectorNode->next;
                }
            }
            else if(std::strcmp(tempData.RackName, "Rack 3") == 0)
            {
                xmlNode *vectorNode = cur_node->children;
                while(vectorNode)
                {
                    if(vectorNode->type == XML_ELEMENT_NODE && xmlStrcmp(vectorNode->name, BAD_CAST "Vector") == 0)
                    {
                        xmlChar* x = xmlGetProp(vectorNode, BAD_CAST "x");
                        xmlChar* y = xmlGetProp(vectorNode, BAD_CAST "y");
                        xmlChar* z = xmlGetProp(vectorNode, BAD_CAST "z");
                        if(step == 0)
                        {
                            tempData.PointUpLeft.x = atof(reinterpret_cast<char*>(x));
                            tempData.PointUpLeft.y = atof(reinterpret_cast<char*>(y));
                            tempData.PointUpLeft.z = atof(reinterpret_cast<char*>(z));
                            printf("Vector x = %f\r\n", tempData.PointUpLeft.x);
                            printf("Vector y = %f\r\n", tempData.PointUpLeft.y);
                            printf("Vector z = %f\r\n", tempData.PointUpLeft.z);
                            step++;
                        }
                        else if(step == 1)
                        {
                            tempData.PointLowLeft.x = atof(reinterpret_cast<char*>(x));
                            tempData.PointLowLeft.y = atof(reinterpret_cast<char*>(y));
                            tempData.PointLowLeft.z = atof(reinterpret_cast<char*>(z));
                            step++;
                        }
                        else if(step == 2)
                        {
                            tempData.PointLowRight.x = atof(reinterpret_cast<char*>(x));
                            tempData.PointLowRight.y = atof(reinterpret_cast<char*>(y));
                            tempData.PointLowRight.z = atof(reinterpret_cast<char*>(z));
                            
                            // 消息队列发送
                            msgType temp;
                            temp.msgtype = 1;
                            //序列化
                            serialize(tempData, temp.msgdata, sizeof(temp.msgdata));

                            if (msgsnd(msgCalibrationsID, &temp, sizeof(tempData), 0) == -1) //IPC_NOWAIT
                            {
                                perror("Calibrations msgsnd failed");
                            }

                            std::memset(reinterpret_cast<char*>(&tempData), 0, sizeof(tempData));

                            step = 0;
                        }
                        
                        xmlFree(x);
                        xmlFree(y);
                        xmlFree(z);
                    }
                    vectorNode = vectorNode->next;
                }
            }   
        }        

        parseCalibrations(cur_node->children);  // 递归解析子节点
    }
}



//进样流程
void parseLiquidSamplingFlow(xmlNode* node)
{
    xmlNode* cur_node = nullptr;
    static LiquidSamplingFlowType tempdata;
    
    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if(cur_node->type == XML_ELEMENT_NODE && xmlStrcmp(cur_node->name, BAD_CAST "ScriptType") == 0)
        {
            xmlChar* methodName = xmlGetProp(cur_node, BAD_CAST "methodName");

            //液体进样流程
            if(xmlStrcmp(methodName, BAD_CAST "LIQ_STD") == 0)
            {
               std::strcpy(tempdata.methodName, "LIQ_STD");
            }//顶空进样流程
            else if(xmlStrcmp(methodName, BAD_CAST "HS_STD") == 0)
            {
                std::strcpy(tempdata.methodName, "HS_STD");
            }//SPME进样流程
            else if(xmlStrcmp(methodName, BAD_CAST "SPME_STD") == 0)
            {
                std::strcpy(tempdata.methodName, "SPME_STD");
            }
           
            xmlFree(methodName);
        }
        else if(cur_node->type == XML_ELEMENT_NODE && xmlStrcmp(cur_node->name, BAD_CAST "Reference") == 0)
        {
            xmlChar* Name = xmlGetProp(cur_node, BAD_CAST "name");
            //样品盘数据处理
            if(xmlStrcmp(Name, BAD_CAST "Rack 1") == 0)
            {
               std::strcpy(tempdata.sampleRack, "Rack 1");
            }
            else if(xmlStrcmp(Name, BAD_CAST "Rack 2") == 0)
            {
                std::strcpy(tempdata.sampleRack, "Rack 2");
            }
            else if(xmlStrcmp(Name, BAD_CAST "Rack 3") == 0)
            {
                std::strcpy(tempdata.sampleRack, "Rack 3");
            }
            else if(xmlStrcmp(Name, BAD_CAST "Inlet 1") == 0)
            {
                printf("1 conditioningPort = %s\r\n", tempdata.conditioningPort);
                if(std::strcmp(tempdata.conditioningPort, "conditioningPort") == 0)
                {
                    std::memset(tempdata.conditioningPort,0,sizeof(tempdata.conditioningPort));
                    std::strcpy(tempdata.conditioningPort, "Inlet 1");
                }
                std::strcpy(tempdata.injector, "Inlet 1");
            }
            else if(xmlStrcmp(Name, BAD_CAST "Burn-In 1") == 0)
            {
                printf("2 conditioningPort = %s\r\n", tempdata.conditioningPort);
                if(std::strcmp(tempdata.conditioningPort, "conditioningPort") == 0)
                {
                    std::memset(tempdata.conditioningPort,0,sizeof(tempdata.conditioningPort));
                    std::strcpy(tempdata.conditioningPort, "Burn-In 1");
                }
            }

            xmlFree(Name);
        }
        else if(cur_node->type == XML_ELEMENT_NODE && xmlStrcmp(cur_node->name, BAD_CAST "ReferenceParameter") == 0)
        {
            xmlChar* name = xmlGetProp(cur_node, BAD_CAST "name");

            if(xmlStrcmp(name, BAD_CAST "conditioningPort") == 0)
            {   
                //样品盘孔位数据处理
                std::strcpy(tempdata.conditioningPort, reinterpret_cast<char *>(name));
                printf("3 conditioningPort = %s\r\n", tempdata.conditioningPort);
            }                                                                

            xmlFree(name);
        }
        else if(cur_node->type == XML_ELEMENT_NODE && xmlStrcmp(cur_node->name, BAD_CAST "IntParameter") == 0)
        {
            xmlChar* name = xmlGetProp(cur_node, BAD_CAST "name");
            xmlChar* value = xmlGetProp(cur_node, BAD_CAST "value");

            if(xmlStrcmp(name, BAD_CAST "sampleIndex") == 0)
            {   
                //样品盘孔位数据处理
                std::strcpy(tempdata.sampleRackNum, reinterpret_cast<char *>(value));
            }
            else if(xmlStrcmp(name, BAD_CAST "fillingStrokesCount") == 0)
            {
                //冲程次数
                tempdata.fillingStrokesCount = atoi(reinterpret_cast<char *>(value));
            }
            else if(xmlStrcmp(name, BAD_CAST "preWashWithSolvent1") == 0)
            {
                tempdata.preWashWithSolvent1 = atoi(reinterpret_cast<char *>(value));
            }
            else if(xmlStrcmp(name, BAD_CAST "preWashWithSolvent2") == 0)
            {
                tempdata.preWashWithSolvent2 = atoi(reinterpret_cast<char *>(value));
            }
            else if(xmlStrcmp(name, BAD_CAST "preWashWithSolvent3") == 0)
            {
                tempdata.preWashWithSolvent3 = atoi(reinterpret_cast<char *>(value));
            }
            else if(xmlStrcmp(name, BAD_CAST "preWashWithSolvent4") == 0)
            {
                tempdata.preWashWithSolvent4 = atoi(reinterpret_cast<char *>(value));
            }             
            else if(xmlStrcmp(name, BAD_CAST "sampleRinseCycles") == 0)
            {
                tempdata.sampleRinseCycles = atoi(reinterpret_cast<char *>(value));
            }
            else if(xmlStrcmp(name, BAD_CAST "postWashWithSolvent1") == 0)
            {
                tempdata.postWashWithSolvent1 = atoi(reinterpret_cast<char *>(value));
            }
            else if(xmlStrcmp(name, BAD_CAST "postWashWithSolvent2") == 0)
            {
                tempdata.postWashWithSolvent2 = atoi(reinterpret_cast<char *>(value));
            }
            else if(xmlStrcmp(name, BAD_CAST "postWashWithSolvent3") == 0)
            {
                tempdata.postWashWithSolvent3 = atoi(reinterpret_cast<char *>(value));
            }
            else if(xmlStrcmp(name, BAD_CAST "postWashWithSolvent4") == 0)
            {
                tempdata.postWashWithSolvent4 = atoi(reinterpret_cast<char *>(value));
            }                                                       

            xmlFree(name);
            xmlFree(value);
        }
        else if(cur_node->type == XML_ELEMENT_NODE && xmlStrcmp(cur_node->name, BAD_CAST "DoubleUnitParameter") == 0)
        {
            xmlChar* name = xmlGetProp(cur_node, BAD_CAST "name");
            xmlChar* value = xmlGetProp(cur_node, BAD_CAST "value");
            xmlChar* time = xmlGetProp(cur_node, BAD_CAST "unit");
            
            if(xmlStrcmp(name, BAD_CAST "sampleVolume") == 0)
            {   
                //样品体积
                tempdata.sampleVolume = atof(reinterpret_cast<char *>(value));
            }
            else if(xmlStrcmp(name, BAD_CAST "washVialDepth") == 0)
            {   
                //清洗扎针深度
                tempdata.washVialDepth = atof(reinterpret_cast<char *>(value));
            }
            else if(xmlStrcmp(name, BAD_CAST "wastePortDepth") == 0)
            {   
                //废液扎针深度
               tempdata.wastePortDepth = atof(reinterpret_cast<char *>(value));
            }
            else if(xmlStrcmp(name, BAD_CAST "analysisTime") == 0)
            {   
                //GC循环时间
                tempdata.analysisTime = atof(reinterpret_cast<char *>(value));
                std::strcpy(tempdata.analysisTimeUnit, reinterpret_cast<char *>(time));
            }
            else if(xmlStrcmp(name, BAD_CAST "preWashSolventVolume") == 0)
            {   
                //填充进样针体积的%多少用于清洗
                tempdata.preWashSolventVolume = atof(reinterpret_cast<char *>(value));
            }
            else if(xmlStrcmp(name, BAD_CAST "preWashAspirateFlowRate") == 0)
            {   
                //清洗抽吸速度
                tempdata.preWashAspirateFlowRate = atof(reinterpret_cast<char *>(value));
            }
            else if(xmlStrcmp(name, BAD_CAST "sampleVialDepth") == 0)
            {   
                //样品瓶扎针深度
                tempdata.sampleVialDepth = atof(reinterpret_cast<char *>(value));
            }
            else if(xmlStrcmp(name, BAD_CAST "sampleVialPenetrationSpeed") == 0)
            {   
                //样品瓶扎针速度
                tempdata.sampleVialPenetrationSpeed = atof(reinterpret_cast<char *>(value));
            }
            else if(xmlStrcmp(name, BAD_CAST "sampleRinseVolume") == 0)
            {   
                //样品清洗抽针体积
                tempdata.sampleRinseVolume = atof(reinterpret_cast<char *>(value));
            }
            else if(xmlStrcmp(name, BAD_CAST "fillingStrokesVolume") == 0)
            {   
                //冲程体积
                tempdata.fillingStrokesVolume = atof(reinterpret_cast<char *>(value));
            }
            else if(xmlStrcmp(name, BAD_CAST "fillingStrokesAspirateFlowRate") == 0)
            {   
                //冲程抽吸速度
                tempdata.fillingStrokesAspirateFlowRate = atof(reinterpret_cast<char *>(value));
            }
            else if(xmlStrcmp(name, BAD_CAST "delayAfterFillingStrokes") == 0)
            {   
                //冲程后等待时间
                tempdata.delayAfterFillingStrokes = atof(reinterpret_cast<char *>(value));
                std::strcpy(tempdata.delayAfterFillingStrokesUnit, reinterpret_cast<char *>(time));
            }
            else if(xmlStrcmp(name, BAD_CAST "sampleAspirateFlowRate") == 0)
            {   
                //样品抽吸速度
                tempdata.sampleAspirateFlowRate = atof(reinterpret_cast<char *>(value));
            }
            else if(xmlStrcmp(name, BAD_CAST "samplePostAspirateDelay") == 0)
            {   
                //采样后抽吸延迟
                tempdata.samplePostAspirateDelay = atof(reinterpret_cast<char *>(value));
                std::strcpy(tempdata.samplePostAspirateDelayUnit, reinterpret_cast<char *>(time));
            }
            else if(xmlStrcmp(name, BAD_CAST "airVolume") == 0)
            {   
                //空气体积
                tempdata.airVolume = atof(reinterpret_cast<char *>(value));
            }
            else if(xmlStrcmp(name, BAD_CAST "injectorPenetrationDepth") == 0)
            {   
                //进样口扎针深度
                tempdata.injectorPenetrationDepth = atof(reinterpret_cast<char *>(value));
            }
            else if(xmlStrcmp(name, BAD_CAST "injectorPenetrationSpeed") == 0)
            {   
                //进样口扎针速度
                tempdata.injectorPenetrationSpeed = atof(reinterpret_cast<char *>(value));
            }
            else if(xmlStrcmp(name, BAD_CAST "preInjectionDwellTime") == 0)
            {   
                //进样前时间延迟
                tempdata.preInjectionDwellTime = atof(reinterpret_cast<char *>(value));
                std::strcpy(tempdata.preInjectionDwellTimeUnit, reinterpret_cast<char *>(time));
            }
            else if(xmlStrcmp(name, BAD_CAST "injectionFlowRate") == 0)
            {   
                //进样速度
                tempdata.injectionFlowRate = atof(reinterpret_cast<char *>(value));
            }
            else if(xmlStrcmp(name, BAD_CAST "postInjectionDwellTime") == 0)
            {   
                //进样后时间延迟
                tempdata.postInjectionDwellTime = atof(reinterpret_cast<char *>(value));
                std::strcpy(tempdata.postInjectionDwellTimeUnit, reinterpret_cast<char *>(time));
            }
            else if(xmlStrcmp(name, BAD_CAST "postWashSolventVolume") == 0)
            {   
                //填充进样针用于清洗
                tempdata.postWashSolventVolume = atof(reinterpret_cast<char *>(value));
            }
            else if(xmlStrcmp(name, BAD_CAST "postWashAspirateFlowRate") == 0)
            {   
                //清洗抽吸速度
                tempdata.postWashAspirateFlowRate = atof(reinterpret_cast<char *>(value));
            }
            else if(xmlStrcmp(name, BAD_CAST "incubationTime") == 0)
            {   
                //孵育时间，单位为分钟（m）
                tempdata.incubationTime = atof(reinterpret_cast<char *>(value));
                std::strcpy(tempdata.incubationTimeUnit, reinterpret_cast<char *>(time));
            }
            else if(xmlStrcmp(name, BAD_CAST "incubationTimeIncrement") == 0)
            {   
                //孵育时间增量，单位为分钟（m）
                tempdata.incubationTimeIncrement = atof(reinterpret_cast<char *>(value));
                std::strcpy(tempdata.incubationTimeIncrementUnit, reinterpret_cast<char *>(time));
            }
            else if(xmlStrcmp(name, BAD_CAST "incubationTemperature") == 0)
            {   
                //孵育温度，单位为摄氏度（℃）
                tempdata.incubationTemperature = atof(reinterpret_cast<char *>(value));
            }
            else if(xmlStrcmp(name, BAD_CAST "syringeTemperature") == 0)
            {   
                //注射器温度，单位为摄氏度（℃）
                tempdata.syringeTemperature = atof(reinterpret_cast<char *>(value));
            }
            else if(xmlStrcmp(name, BAD_CAST "preFillingVolume") == 0)
            {   
                //预填充体积，单位为毫升（mL）
                tempdata.preFillingVolume = atof(reinterpret_cast<char *>(value));
            }
            else if(xmlStrcmp(name, BAD_CAST "postInjectionPurgeTime") == 0)
            {   
                //注射后冲洗时间，单位为秒（s）
                tempdata.postInjectionPurgeTime = atof(reinterpret_cast<char *>(value));
                std::strcpy(tempdata.postInjectionPurgeTimeUnit, reinterpret_cast<char *>(time));
            }
            else if(xmlStrcmp(name, BAD_CAST "preInjectionPurgeTime") == 0)
            {   
                //注射前冲洗时间，单位为秒（s）
                tempdata.preInjectionPurgeTime = atof(reinterpret_cast<char *>(value));
                std::strcpy(tempdata.preInjectionPurgeTimeUnit, reinterpret_cast<char *>(time));
            }
            else if(xmlStrcmp(name, BAD_CAST "agitatorSpeed") == 0)
            {   
                //搅拌器速度，单位为转/分钟（rpm）
                tempdata.agitatorSpeed = atoi(reinterpret_cast<char *>(value));
                
            }
            else if(xmlStrcmp(name, BAD_CAST "agitatorOnTime") == 0)
            {   
                //搅拌器开启时间，单位为秒（s）
                tempdata.agitatorOnTime = atof(reinterpret_cast<char *>(value));
                
            }
            else if(xmlStrcmp(name, BAD_CAST "agitatorOffTime") == 0)
            {   
                //搅拌器关闭时间，单位为秒（s）
                tempdata.agitatorOffTime = atof(reinterpret_cast<char *>(value));

                if(std::strcmp(tempdata.methodName, "HS_STD") == 0)
                {
                    //发送消息队列
                    msgType temp;
                    temp.msgtype = 1;
                    //序列化
                    serialize(tempdata, temp.msgdata, sizeof(temp.msgdata));

                    if (msgsnd(msgLiquidSamplingFlowID, &temp, sizeof(tempdata), IPC_NOWAIT) == -1) //IPC_NOWAIT
                    {
                        perror("LiquidSamplingFlow msgsnd failed");
                    }
                    
                    std::memset(reinterpret_cast<char*>(&tempdata), 0, sizeof(tempdata));
                }

            }
            else if(xmlStrcmp(name, BAD_CAST "spmeIncubationTime") == 0)
            {
                //SPME孵化时间
                tempdata.spmeIncubationTime = atof(reinterpret_cast<char *>(value));
                std::strcpy(tempdata.spmeIncubationTimeUnit, reinterpret_cast<char *>(time));
            }
            else if(xmlStrcmp(name, BAD_CAST "conditioningTemperature") == 0)
            {
                //老化温度
                tempdata.conditioningTemperature = atof(reinterpret_cast<char *>(value));
            }
            else if(xmlStrcmp(name, BAD_CAST "sampleExtractTime") == 0)
            {
                //样品富集时间
                tempdata.sampleExtractTime = atof(reinterpret_cast<char *>(value));
                std::strcpy(tempdata.sampleExtractTimeUnit, reinterpret_cast<char *>(time));
            }
            else if(xmlStrcmp(name, BAD_CAST "sampleDesorbTime") == 0)
            {
                //样品解吸时间
                tempdata.sampleDesorbTime = atof(reinterpret_cast<char *>(value));
                std::strcpy(tempdata.sampleDesorbTimeUnit, reinterpret_cast<char *>(time));
            }
            else if(xmlStrcmp(name, BAD_CAST "preConditioningTime") == 0)
            {
                //取样前老化时间
                tempdata.preConditioningTime = atof(reinterpret_cast<char *>(value));
                std::strcpy(tempdata.preConditioningTimeUnit, reinterpret_cast<char *>(time));
            }
            else if(xmlStrcmp(name, BAD_CAST "postConditioningTime") == 0)
            {
                //取样后老化时间
                tempdata.postConditioningTime = atof(reinterpret_cast<char *>(value));
                std::strcpy(tempdata.postConditioningTimeUnit, reinterpret_cast<char *>(time));
            }
            else if(xmlStrcmp(name, BAD_CAST "conditioningStandbyTemperature") == 0)
            {
                //老化待机温度
                tempdata.conditioningStandbyTemperature = atof(reinterpret_cast<char *>(value));
            }
            else if(xmlStrcmp(name, BAD_CAST "agitatorStandbyTemperature") == 0)
            {
                //孵化模块待机温度
                tempdata.agitatorStandbyTemperature = atof(reinterpret_cast<char *>(value));
            }                                                                                                                                                                                                                    
                        
            xmlFree(name);
            xmlFree(value);
            xmlFree(time);           
        }
        else if(cur_node->type == XML_ELEMENT_NODE && xmlStrcmp(cur_node->name, BAD_CAST "BooleanParameter") == 0)
        {
            xmlChar* name = xmlGetProp(cur_node, BAD_CAST "name");
            xmlChar* value = xmlGetProp(cur_node, BAD_CAST "value");
 
            if(xmlStrcmp(name, BAD_CAST "heatAgitator") == 0)
            {   
                //是否加热搅拌器
                if(xmlStrcmp(value, BAD_CAST "True") == 0)
                {
                    tempdata.heatAgitator = 1;
                }
                else
                {
                    tempdata.heatAgitator = 0;
                }
            } 
            else if(xmlStrcmp(name, BAD_CAST "heatSyringe") == 0)
            {   
                //是否加热进样工具
                if(xmlStrcmp(value, BAD_CAST "True") == 0)
                {
                    tempdata.heatSyringe = 1;
                }
                else
                {
                    tempdata.heatSyringe = 0;
                }
            }
            else if(xmlStrcmp(name, BAD_CAST "enablePreFilling") == 0)
            {   
                //是否启用预填充
                tempdata.enablePreFilling = atoi(reinterpret_cast<char *>(value));
            }
            else if(xmlStrcmp(name, BAD_CAST "continuousPurge") == 0)
            {   
                //是否持续冲洗 
                tempdata.continuousPurge = atoi(reinterpret_cast<char *>(value));
            }
            else if(xmlStrcmp(name, BAD_CAST "doAgitation") == 0)
            {   
                //样品富集时是否开启振摇标志 
                tempdata.doAgitation = atoi(reinterpret_cast<char *>(value));
            }
            else if(xmlStrcmp(name, BAD_CAST "setConditioningStandbyTemperature") == 0)
            {   
                //是否开启老化模块待机温度
                tempdata.setConditioningStandbyTemperature = atoi(reinterpret_cast<char *>(value));
            }
            else if(xmlStrcmp(name, BAD_CAST "setAgitatorStandbyTemperature") == 0)
            {   
                //是否开启孵化模块待机温度
                tempdata.setAgitatorStandbyTemperature = atoi(reinterpret_cast<char *>(value));

                if(std::strcmp(tempdata.methodName, "SPME_STD") == 0)
                {
                    //发送消息队列
                    msgType temp;
                    temp.msgtype = 1;
                    //序列化
                    serialize(tempdata, temp.msgdata, sizeof(temp.msgdata));

                    if (msgsnd(msgLiquidSamplingFlowID, &temp, sizeof(tempdata), IPC_NOWAIT) == -1) //IPC_NOWAIT
                    {
                        perror("LiquidSamplingFlow msgsnd failed");
                    }
                    
                    std::memset(reinterpret_cast<char*>(&tempdata), 0, sizeof(tempdata));
                }
            }        

            xmlFree(name);
            xmlFree(value);

        }        

        parseLiquidSamplingFlow(cur_node->children);  // 递归解析子节点
    }
}


