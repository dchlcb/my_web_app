#ifndef _CALIBRATIONS_H
#define _CALIBRATIONS_H

#include "TypeDefine.h"

#define CALIBRATIONS_STR_LEN 128

//坐标存入字符串
//小样品盘
#define VT54_NAME " "


//冲洗站
#define SOLVENT1_NAME "RinseStation1 5" //溶剂1
#define SOLVENT2_NAME "RinseStation1 4" //溶剂2
#define SOLVENT3_NAME "RinseStation1 3" //溶剂3
#define SOLVENT4_NAME "RinseStation1 2" //溶剂4
#define WLIB_NAME "RinseStation1 1" //废液瓶名字

//孵化器
#define INCUBATION1_NAME "Agitator1 1" //溶剂1
#define INCUBATION2_NAME "Agitator1 2" //溶剂2
#define INCUBATION3_NAME "Agitator1 3" //溶剂3
#define INCUBATION4_NAME "Agitator1 4" //溶剂4
#define INCUBATION5_NAME "Agitator1 5" //溶剂5
#define INCUBATION6_NAME "Agitator1 6" //溶剂6

//老化口
#define BURNIN_NAME "Burn-In1 1"

//进样口
#define INJECTOR "Inlet1 1"

//HOME位
#define HOME_NAME "Home1 1"

//换针位
#define CHANGETOOL_NAME "ChangeTool1 1"

//关机位
#define SHUTDOWN_NAME "Shutdown1 1"


//小样品盘参数
#define VT54_ROW 6
#define VT54_COL 9
#define VT54_R_DIS 13.9 //横向中心距
#define VT54_C_DIS 13.3 //纵向中心距
#define VT54_MAX  54 //多少孔数


//大样品盘参数
#define VT15_ROW 3
#define VT15_COL 5
#define VT15_R_DIS 25.0 
#define VT15_C_DIS 27.2
#define VT15_MAX 15

//冲洗站参数
#define RS_ROW 5
#define RS_COL 1
#define RS_R_DIS 0.0 
#define RS_C_DIS 28.0
#define RS_MAX 5

//孵化器参数
#define AG_ROW 2
#define AG_COL 3
#define AG_R_DIS 49.0 
#define AG_C_DIS 49.0
#define AG_MAX 6

//三个校准点
typedef struct 
{
	double x; //X轴位置
	double y; //Y轴位置
	double z; //Z轴位置
}CalibrationsPointType;

//校准
typedef struct
{
	char RackName[CALIBRATIONS_STR_LEN]; //样品盘号
	char RackType[CALIBRATIONS_STR_LEN]; //样品盘类型
	char CapType[CALIBRATIONS_STR_LEN]; //盖子类型

	CalibrationsPointType PointUpLeft; //样品盘左上角校准点
	CalibrationsPointType PointLowLeft; //样品盘左下角校准点
	CalibrationsPointType PointLowRight; //样品盘右下角校准点

}CalibrationsType;


//校准线程
void* CalibrationsFunc(void* arg);

//写入一个坐标
int write_coordinates(const char *identifier, float x, float y, float z);

//读取一个坐标
int read_coordinates(const char *identifier, float *x, float *y, float *z);

//计算样品盘各孔坐标并存入文件
void CoordinateCalSave(CalibrationsType data, std::string numStr, float r_dis, float c_dis, int numMax);

#endif