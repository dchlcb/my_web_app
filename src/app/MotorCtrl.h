#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H

#include <cstdlib>
#include <iostream>
#include <time.h>
#include "TypeDefine.h"

//纳诺达克电机 Z 轴 正向往上走 负向往下走   正向回零
//纳诺达克电机 X 轴 人站在机器后面 正向往左走 负向往右走 负向回零
//纳诺达克电机 Y 轴 人站在机器后面 正向往后走 负向往前走 正向回零
//纳诺达克电机 O 轴 正向往下走 负向往上走 正向回零

//立迈胜 O轴和X轴反向

#define NANOTEC

#ifdef NANOTEC 
#define ENCODER_RESOLUTION 4000.0 //纳诺达克编码器分辨率
#define RED_CANID 0x2009 //读取电机ID
#define RED_CANID_SUB 0x00
#define FORWARD_TO_ZERO 0xEF //正向回零
#define INVER_TO_ZERO  0xEE  //反向回零
#define SET_BLOCKING_TORQUE 0x203A //设置堵转电流
    #define SET_BLOCKING_TORQUE_SUB 0x01//设置堵转电流
    #define SET_BLOCKING_TIME_SUB 0x02 //设置堵转检测时间
    #define SET_BLOCKING_DATA_LEN 4
#else 
#define ENCODER_RESOLUTION 10000.0 //立迈胜编码器分辨率
#define RED_CANID 0x200C 
#define RED_CANID_SUB 0x02
#define FORWARD_TO_ZERO 0x25 //正向回零
#define INVER_TO_ZERO  0x26  //反向回零
#define SET_BLOCKING_TORQUE 0x2007 //设置堵转电流
    #define SET_BLOCKING_TORQUE_SUB 0x13 //设置堵转电流
    #define SET_BLOCKING_TIME_SUB 0x15 //设置堵转检测时间
    #define SET_BLOCKING_DATA_LEN 2
#endif

#define X_AXIS_PITCH 62.8  //X轴螺距 单位mm
#define Y_AXIS_PITCH 40.19 //Y轴螺距 单位mm
#define Z_AXIS_PITCH 37.68 //Z轴螺距 单位mm
#define O_AXIS_PITCH 20.00 //进样针轴螺距 单位mm

//各轴移动长度
#define X_LENGTHS 756.19 //mm
#define Y_LENGTHS 253.42 
#define Z_LENGTHS 417.85
#define O_LENGTHS 88.84

#define Y_RETZERO_MOVE 10.0 //Y轴电机回零后移动距离

/*电机初始化*/
void MotorInit(void);

/*获取电机位置*/
double GetMotorPosition(AxisType axis);

/*获取电机速度*/
double GetMotorSpeed(AxisType axis);

/*电机位置运行*/
void MotorRelPosCtrl(bool flg, AcitonType motion, AxisType axis, double position, double speed);

/*电机回零*/
void MotorRetZeroInit(void);

/*电机使能*/
void MotorEnableCtrl(AxisType axis);

/*电机暂停*/
void MotorPauseCtrl(AxisType axis);

/*电机停机*/
void MotorStopCtrl(AxisType axis);

/*电机触底判断*/
bool MotorBottominOutJudgment(AxisType axis, double cur);

/*电机方向控制*/
void MotorDirectionCtrl(AxisType axis, bool dir);

/*电机回零                  轴           堵转转矩       检查时间         回零速度           回零加速度*/
void MotorRetZero(AxisType axis, int32_t torques, int32_t time, uint32_t speed, uint32_t acceSpeed);

/*轮廓位置模式*/
void MotorPositionModle(AxisType axis);

/*位置距离换算编码器分辨率*/
int32_t PosConvReso(AxisType axis, double position);

/*编码器分辨率换算位置距离*/
double ResoConvPos(AxisType axis, int32_t resolution);

/*速度换算编码器分辨率 v/s*/
uint32_t SpedConvReso(AxisType axis, double speed);

/*编码器分辨率换算速度 v/s*/
double ResoConvSped(AxisType axis, int32_t resolution);

void MotorPosCtrl(AcitonType motion, AxisType axis, double position, double speed);

/*电机相对位置运行*/
void MotorMovePosCtrl(AcitonType motion, AxisType axis, double position, double speed);


extern volatile  double xposRecord;
extern volatile  double yposRecord; 
extern volatile  double zposRecord;
extern volatile  double oposRecord;

#endif