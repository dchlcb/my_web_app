#ifndef _SOAP_PROCESS_H
#define _SOAP_PROCESS_H


#include <sstream>
#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <libxml/parser.h>
#include <libxml/tree.h>
#include <iostream>
#include <string>
#include <sys/socket.h>
#include "main.h"

enum MethodType
{
    NoneType = 0,
    LiquidType = 1,
    HeadSpaceType = 2,
    SPMEType = 3
};


// 处理客户端请求
void handleRequest(int clientSocket);

// 电机动作解析
void parseExcuteActions(xmlNode* node);

//校准解析
void parseCalibrations(xmlNode* node);

//校准解析测试
void parseCalibrationsTest(xmlNode* node);

//进样流程
void parseLiquidSamplingFlow(xmlNode* node);

// 各模块心跳包
void parseModuleHeart(int clientSocket, xmlNode* node);

#endif