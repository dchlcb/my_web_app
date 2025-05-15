#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <pthread.h>
#include <iostream>
#include <string>
#include <algorithm>
#include "CANOpen.h"
#include "Public.h"
#include "Calibrations.h"
#include "main.h"
#include "LiquidSamplingFlow.h"



//校准线程
void* CalibrationsFunc(void* arg)
{
    key_t CKey;

    msgType* msgData = new msgType;
    msgData->msgtype = 1;

    CKey = ftok(".", 'C');

    // 检查并删除旧消息队列
    msgCalibrationsID = msgget(CKey, 0);  // 尝试获取现有队列
    if (msgCalibrationsID != -1) 
    {
        if (msgctl(msgCalibrationsID, IPC_RMID, NULL) == -1) 
        {
            perror("msgctl IPC_RMID failed");
            delete msgData;
            exit(1);
        }
        printf("成功删除旧消息队列\n");
    } 
    else 
    {
        printf("没有找到旧消息队列\n");
    }

    msgCalibrationsID = msgget(CKey, IPC_CREAT| IPC_EXCL | 0666);
    
    if (msgCalibrationsID == -1) 
    {
        perror("Calibrations msgget failed");
        delete msgData;
        pthread_exit(NULL);
    }  

    while(stop_flag)
    {
        if(msgrcv(msgCalibrationsID, msgData, sizeof(msgData->msgdata), 0, 0) == -1)
        {
            perror("Calibrations msgrcv failed");
            break;  // 其他错误，退出循环
        }
        else 
        {
            CalibrationsType tempData;
            //反序列化
            deserialize(msgData->msgdata, tempData, sizeof(msgData->msgdata));

            //冲洗站
            if(strcmp(tempData.RackName, "Rinse Station 1") == 0)
            {
                printf("REC RinseStation_1 x = %f\r\n",tempData.PointUpLeft.x);
                printf("REC RinseStation_1 y = %f\r\n",tempData.PointUpLeft.y);
                printf("REC RinseStation_1 z = %f\r\n",tempData.PointUpLeft.z);

                write_coordinates("RinseStation_1", tempData.PointUpLeft.x, tempData.PointUpLeft.y, tempData.PointUpLeft.z);

                //校准完成计算每个孔位坐标并存入
                CoordinateCalSave(tempData, "1", RS_R_DIS, RS_C_DIS, RS_MAX);
            }
            //注入口
            else if(strcmp(tempData.RackName, "Inlet 1") == 0)
            {
                printf("REC Inlet 1 x = %f\r\n",tempData.PointUpLeft.x);
                printf("REC Inlet 1 y = %f\r\n",tempData.PointUpLeft.y);
                printf("REC Inlet 1 z = %f\r\n",tempData.PointUpLeft.z);

                //write_coordinates("Inlet 1", tempData.PointUpLeft.x, tempData.PointUpLeft.y, tempData.PointUpLeft.z);

                //校准完成计算每个孔位坐标并存入
                CoordinateCalSave(tempData, "1", 0, 0, 1);
            }
            //孵化器
            else if(strcmp(tempData.RackName, "Agitator 1") == 0)
            {
                printf("REC Agitator 1 x = %f\r\n",tempData.PointUpLeft.x);
                printf("REC Agitator 1 y = %f\r\n",tempData.PointUpLeft.y);
                printf("REC Agitator 1 z = %f\r\n",tempData.PointUpLeft.z);

                //write_coordinates("Inlet 1", tempData.PointUpLeft.x, tempData.PointUpLeft.y, tempData.PointUpLeft.z);

                //校准完成计算每个孔位坐标并存入
                CoordinateCalSave(tempData, "1", AG_R_DIS, AG_C_DIS, AG_MAX);
            }
            //老化口
            else if(strcmp(tempData.RackName, "Burn-In 1") == 0)
            {
                printf("REC Burn-In 1 x = %f\r\n",tempData.PointUpLeft.x);
                printf("REC Burn-In 1 y = %f\r\n",tempData.PointUpLeft.y);
                printf("REC Burn-In 1 z = %f\r\n",tempData.PointUpLeft.z);

                //write_coordinates("Inlet 1", tempData.PointUpLeft.x, tempData.PointUpLeft.y, tempData.PointUpLeft.z);

                //校准完成计算每个孔位坐标并存入
                CoordinateCalSave(tempData, "1", 0, 0, 1);
            }
            //HOME位
            else if(strcmp(tempData.RackName, "Home") == 0)
            {
                printf("REC Home x = %f\r\n",tempData.PointUpLeft.x);
                printf("REC Home y = %f\r\n",tempData.PointUpLeft.y);
                printf("REC Home z = %f\r\n",tempData.PointUpLeft.z);

                //write_coordinates("Home 1", tempData.PointUpLeft.x, tempData.PointUpLeft.y, tempData.PointUpLeft.z);

                //校准完成计算每个孔位坐标并存入
                CoordinateCalSave(tempData, "1", 0, 0, 1);
            }
            //换针位
            else if(strcmp(tempData.RackName, "ChangeTool") == 0)
            {
                printf("REC ChangeTool x = %f\r\n",tempData.PointUpLeft.x);
                printf("REC ChangeTool y = %f\r\n",tempData.PointUpLeft.y);
                printf("REC ChangeTool z = %f\r\n",tempData.PointUpLeft.z);

                //write_coordinates("ChangeTool 1", tempData.PointUpLeft.x, tempData.PointUpLeft.y, tempData.PointUpLeft.z);

                //校准完成计算每个孔位坐标并存入
                CoordinateCalSave(tempData, "1", 0, 0, 1);
            }
            //关机位
            else if(strcmp(tempData.RackName, "Shutdown") == 0)
            {
                printf("REC Shutdown x = %f\r\n",tempData.PointUpLeft.x);
                printf("REC Shutdown y = %f\r\n",tempData.PointUpLeft.y);
                printf("REC Shutdown z = %f\r\n",tempData.PointUpLeft.z);

                //write_coordinates("Shutdown 1", tempData.PointUpLeft.x, tempData.PointUpLeft.y, tempData.PointUpLeft.z);
                
                //校准完成计算每个孔位坐标并存入
                CoordinateCalSave(tempData, "1", 0, 0, 1);
            }             
            //小样品盘 VT54
            else if(strcmp(tempData.RackType, "VT54") == 0)
            {
                printf("REC VT54 x = %f\r\n",tempData.PointUpLeft.x);
                printf("REC VT54 y = %f\r\n",tempData.PointUpLeft.y);
                printf("REC VT54 z = %f\r\n",tempData.PointUpLeft.z);
                
                if(strcmp(tempData.RackName, "Rack 1") == 0)
                {
                    //存入左上角校准坐标
                    //存入左下角校准坐标
                    //write_coordinates("VT54PointLowLeft_1", tempData.PointLowLeft.x, tempData.PointLowLeft.y, tempData.PointLowLeft.z);
                    //存入右下角校准坐标
                    //write_coordinates("VT54PointLowRight_1", tempData.PointLowRight.x, tempData.PointLowRight.y, tempData.PointLowRight.z);

                    //校准完成计算每个孔位坐标并存入
                    CoordinateCalSave(tempData, "1", VT54_R_DIS, VT54_C_DIS, VT54_MAX);
                }
                else if(strcmp(tempData.RackName, "Rack 2") == 0)
                {
                    //存左上角校准坐标
                    //存入左下角校准坐标
                    //write_coordinates("VT54PointLowLeft_2", tempData.PointLowLeft.x, tempData.PointLowLeft.y, tempData.PointLowLeft.z);
                    //存入右下角校准坐标
                    //write_coordinates("VT54PointLowRight_2", tempData.PointLowRight.x, tempData.PointLowRight.y, tempData.PointLowRight.z);

                    //校准完成计算每个孔位坐标并存入
                    CoordinateCalSave(tempData, "1", VT54_R_DIS, VT54_C_DIS, VT54_MAX);
                }
                else if(strcmp(tempData.RackName, "Rack 3") == 0)
                {
                    //存左上角校准坐标
                    //存入左下角校准坐标
                    //write_coordinates("VT54PointLowLeft_3", tempData.PointLowLeft.x, tempData.PointLowLeft.y, tempData.PointLowLeft.z);
                    //存入右下角校准坐标
                    //write_coordinates("VT54PointLowRight_3", tempData.PointLowRight.x, tempData.PointLowRight.y, tempData.PointLowRight.z);

                    //校准完成计算每个孔位坐标并存入
                    CoordinateCalSave(tempData, "1", VT54_R_DIS, VT54_C_DIS, VT54_MAX);
                }

            }//大样品盘 VT15
            else if(strcmp(tempData.RackType, "VT15") == 0)
            {
                if(strcmp(tempData.RackName, "Rack 1") == 0)
                {
                    //存左上角校准坐标
                    //存入左下角校准坐标
                    //write_coordinates("VT15PointLowLeft_1", tempData.PointLowLeft.x, tempData.PointLowLeft.y, tempData.PointLowLeft.z);
                    //存入右下角校准坐标
                    //write_coordinates("VT15PointLowRight_1", tempData.PointLowRight.x, tempData.PointLowRight.y, tempData.PointLowRight.z);

                    //校准完成计算每个孔位坐标并存入
                    CoordinateCalSave(tempData, "1", VT15_R_DIS, VT15_C_DIS, VT15_MAX);

                }
                else if(strcmp(tempData.RackName, "Rack 2") == 0)
                {
                    //存左上角校准坐标
                    //存入左下角校准坐标
                    //write_coordinates("VT15PointLowLeft_2", tempData.PointLowLeft.x, tempData.PointLowLeft.y, tempData.PointLowLeft.z);
                    //存入右下角校准坐标
                    //write_coordinates("VT15PointLowRight_2", tempData.PointLowRight.x, tempData.PointLowRight.y, tempData.PointLowRight.z);

                    //校准完成计算每个孔位坐标并存入
                    CoordinateCalSave(tempData, "1", VT15_R_DIS, VT15_C_DIS, VT15_MAX);
                }
                else if(strcmp(tempData.RackName, "Rack 3") == 0)
                {
                    //存左上角校准坐标
                    //存入左下角校准坐标
                    //write_coordinates("VT15PointLowLeft_3", tempData.PointLowLeft.x, tempData.PointLowLeft.y, tempData.PointLowLeft.z);
                    //存入右下角校准坐标
                    //write_coordinates("VT15PointLowRight_3", tempData.PointLowRight.x, tempData.PointLowRight.y, tempData.PointLowRight.z);

                    //校准完成计算每个孔位坐标并存入
                    CoordinateCalSave(tempData, "1", VT15_R_DIS, VT15_C_DIS, VT15_MAX);
                }
            }

            //ReadCoordinatesMotorExec(HOME_NAME);
        }

    }

    //释放资源
    delete msgData;
    printf("校准线程退出\r\n");
    pthread_exit(NULL);
}

//计算样品盘各孔坐标并存入文件
void CoordinateCalSave(CalibrationsType data, std::string numStr, float r_dis, float c_dis, int numMax)
{
    std::string Name;
    uint8_t flag = 1;
    int8_t factor = 0;
    float tempdata = r_dis;
    int remainder;

    if(std::strcmp(data.RackName,"Rack 1") == 0)
    {
        Name = "Rack 1";
    }
    else if(std::strcmp(data.RackName,"Rack 2") == 0)
    {
        Name = "Rack 2";
    }
    else if(std::strcmp(data.RackName,"Rack 3") == 0)
    {
        Name = "Rack 3";
    }
    else if(std::strcmp(data.RackName,"Rinse Station 1") == 0)
    {
        Name = "Rinse Station 1";
    }
    else if(std::strcmp(data.RackName,"Inlet 1") == 0)
    {
        Name = "Inlet 1";
    }
    else if(std::strcmp(data.RackName,"Agitator 1") == 0)
    {
        Name = "Agitator 1";
        remainder = AG_COL;
        factor = -1;
    }
    else if(std::strcmp(data.RackName,"Burn-In 1") == 0)
    {
        Name = "Burn-In 1";
    }   
    else if(std::strcmp(data.RackName,"Home") == 0)
    {
        Name = "Home 1";
    }
    else if(std::strcmp(data.RackName,"ChangeTool") == 0)
    {
        Name = "ChangeTool 1";
    }
    else if(std::strcmp(data.RackName,"Shutdown") == 0)
    {
        Name = "Shutdown 1";
    }
    else 
    {
        return;
    }

    if(std::strcmp(data.RackType,"VT54") == 0)
    {
        //Name += "VT54";
        remainder = VT54_COL;
    }
    else if(std::strcmp(data.RackType,"VT15") == 0)
    {
        //Name += "VT15";
        remainder = VT15_COL;
    }
    else if(std::strcmp(data.RackType,"RT5") == 0)
    {
        //Name += "RT5";
        factor = -1;
        remainder = RS_COL;
    }

    //删除Name的空格
    Name.erase(std::remove(Name.begin(), Name.end(), ' '), Name.end());
    Name += " ";

    for(int i=std::stoi(numStr); i<(numMax+std::stoi(numStr)); i++)
    {
        std::string temp = Name+std::to_string(i);

        if(flag)
        {
            flag = 0;
            write_coordinates(temp.c_str(), data.PointUpLeft.x, data.PointUpLeft.y, data.PointUpLeft.z);
        }
        else
        {
            write_coordinates(temp.c_str(), data.PointUpLeft.x+tempdata, data.PointUpLeft.y+(c_dis*factor), data.PointUpLeft.z);

            tempdata += r_dis;

            if(i%remainder == 0)
            {
                if(factor >= 0)
                {
                    factor++;
                }
                else
                {
                    factor--;
                }
                tempdata = 0;
            }
        }
    }
}
