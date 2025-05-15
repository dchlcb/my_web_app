#ifndef LIQUID_SAMPLING_FLOW_H
#define LIQUID_SAMPLING_FLOW_H

#include <stdint.h>
#include "TypeDefine.h"
#include <atomic>       // 原子操作支持

typedef enum
{
    GC_START = 0, //启动
    GC_PREPARE = 1 //准备 
}GCCtrlType;

typedef enum
{

}prepAheadType;

typedef struct 
{
    char methodName[128]; //方法名称
    char sampleRack[128]; //样品盘
    char sampleRackNum[128]; //样品盘孔号
    char injector[128]; //注入孔
    
    double sampleVolume; //样品体积
    prepAheadType prepAhead; //系统就绪标志
    double washVialDepth; //清洗扎针深度
    double wastePortDepth; //废液扎针深度
    double analysisTime; //GC循环时间
    char analysisTimeUnit[10]; //单位
    double preWashSolventVolume; //填充进样针体积多少%
    double preWashAspirateFlowRate; //清洗抽吸速度
    int32_t preWashWithSolvent1; //溶剂1的清洗次数
    int32_t preWashWithSolvent2; //溶剂2的清洗次数
    int32_t preWashWithSolvent3; //溶剂3的清洗次数
    int32_t preWashWithSolvent4; //溶剂4的清洗次数
    double sampleVialDepth; //样品瓶扎针深度
    double sampleVialPenetrationSpeed; //样品瓶扎针速度
    int32_t sampleRinseCycles; //样品瓶清洗次数
    double sampleRinseVolume; //样品清洗体积
    int32_t fillingStrokesCount; //冲程次数
    double fillingStrokesVolume; //一次冲程体积
    double fillingStrokesAspirateFlowRate; //冲程抽吸速度
    double delayAfterFillingStrokes; //冲程等待时间
    char delayAfterFillingStrokesUnit[10]; //单位 
    double sampleAspirateFlowRate; //样品抽吸速度
    double samplePostAspirateDelay; //采样后抽吸延迟
    char samplePostAspirateDelayUnit[10]; //单位 
    double airVolume; //空气体积
    double injectorPenetrationDepth; //进样口深度
    double injectorPenetrationSpeed; //进样口速度
    double preInjectionDwellTime; //进样前时间延迟
    char preInjectionDwellTimeUnit[10]; //单位
    double injectionFlowRate; //进样速度
    double postInjectionDwellTime; //进样后时间延迟
    char postInjectionDwellTimeUnit[10]; //单位
    double postWashSolventVolume; //填充进样针用于清洗
    double postWashAspirateFlowRate; //清洗抽吸速度
    int32_t postWashWithSolvent1; //溶剂 1 清洗次数
    int32_t postWashWithSolvent2; //溶剂 2 清洗次数
    int32_t postWashWithSolvent3; //溶剂 3 清洗次数
    int32_t postWashWithSolvent4; //溶剂 4 清洗次数

    //以下HS进样参数
    char agitator[128];  //搅拌器名称
    float incubationTime; //孵育时间，单位为分钟（m）
    char incubationTimeUnit[10]; //单位
    double incubationTimeIncrement; //孵育时间增量，单位为分钟（m）
    char incubationTimeIncrementUnit[10]; //单位
    uint8_t heatAgitator; //是否加热搅拌器
    float incubationTemperature; //孵育温度，单位为摄氏度（℃）
    uint8_t heatSyringe; //是否加热注射器
    double syringeTemperature; //注射器温度，单位为摄氏度（℃）
    uint8_t enablePreFilling; //是否启用预填充
    double preFillingVolume; //预填充体积，单位为毫升（mL）
    double postInjectionPurgeTime; //注射后冲洗时间，单位为秒（s）
    char postInjectionPurgeTimeUnit[10]; //单位
    uint8_t continuousPurge; //是否持续冲洗 
    double preInjectionPurgeTime; //注射前冲洗时间，单位为分钟（m）
    char preInjectionPurgeTimeUnit[10]; //单位
    int32_t agitatorSpeed; //搅拌器速度，单位为转/分钟（rpm）
    float agitatorOnTime; //搅拌器开启时间，单位为秒（s）
    float agitatorOffTime; //搅拌器关闭时间，单位为秒（s）

    //以下是SPME参数
    char spmeAgitator[128];  //搅拌器名称
    char conditioningPort[128];  //老化口名称
    double spmeIncubationTime; //孵化时间
    char spmeIncubationTimeUnit[10]; //单位
    double conditioningTemperature; //老化温度
    double sampleExtractTime; //样品富集时间
    char sampleExtractTimeUnit[10]; //单位
    uint8_t doAgitation; //样品富集时是否开启振摇
    double sampleDesorbTime; //样品解吸时间
    char sampleDesorbTimeUnit[10]; //单位
    double preConditioningTime; //取样前老化时间
    char preConditioningTimeUnit[10]; //单位
    double postConditioningTime; //取样后老化时间
    char postConditioningTimeUnit[10]; //单位
    double conditioningStandbyTemperature; //老化待机温度
    uint8_t setConditioningStandbyTemperature; //是否开启老化待机温度
    double agitatorStandbyTemperature; //孵化模块待机温度
    uint8_t setAgitatorStandbyTemperature; //是否开启孵化模块待机温度
}LiquidSamplingFlowType;


//孵化孔位
typedef struct
{
    uint8_t AgitatorCnt_1;
    uint8_t AgitatorCnt_2;
    uint8_t AgitatorCnt_3;
    uint8_t AgitatorCnt_4;
    uint8_t AgitatorCnt_5;
    uint8_t AgitatorCnt_6;
}AgitatorCntType;




void* LiquidSamplingFlowFunc(void* arg);

//液体进样流程执行
void LiquidProcessExecution(LiquidSamplingFlowType* data);

//HS进样流程
void HSProcessExecution(LiquidSamplingFlowType* data);

//SPME进样流程
void SPMEProcessExecution(LiquidSamplingFlowType* data);

//清洗操作
void WashCtrl(const char* stName, double washDepth, double Volume, double wasteDepth, double washRate, int32_t count);

//冲程操作
//void StrokeCtrl(const char* stName, double Volume, double Delay, double Rate, int32_t count, double Depth);

//冲程操作
void StrokeCtrl(const char* methName, const char* stName, double Volume, double Delay, double Rate, int32_t count, double Depth, double ZRate);

//读取坐标并执行到相应位置
void ReadCoordinatesExec(const char* pointName);

//SPME老化
void SpmeConditioning(double time);

//SPME进样
void SpmePushSampling(double time);

//SPME取样
//SPME取样
void SpmeSampling(double sampleVialDepth, double time, double speed, uint8_t gcCtrl);

//取样
void Sampling(const char* methName,double Volume, double Rate, double Delay);

//进样
void PushSampling(const char* methName, double Volume, double Rate, double Delay, double Depth, double Speed);

//GC状态读取
bool GCStateRead(void);

//孵化模块心跳包
void GetIncubationHeart(void);

//Z轴模块心跳包
void GetZboardHeart(void);

//老化模块心跳包
void GetAgingHeart(void);

//孵化复位
void IncubationMotorReset(uint8_t enable, int32_t offset);

//读取坐标并执行到相应位置
void HSReadCoordinatesExec(const char* pointName);

//读取坐标并执行到相应位置
void ReadCoordinatesMotorExec(const char* pointName);

void OMotorExec(double dis);

extern int msgLiquidSamplingFlowID;

extern std::atomic<int> incubationComp;

#endif