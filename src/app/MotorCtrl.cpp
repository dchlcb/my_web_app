#include <unistd.h>
#include <pthread.h>
#include "Public.h"
#include "MotorCtrl.h"
#include "CANOpen.h"
#include "main.h"
#include "Calibrations.h"
#include "LiquidSamplingFlow.h"

//电机移动位置记录
volatile  double xposRecord = 0.0;
volatile  double yposRecord = 0.0; 
volatile  double zposRecord = 0.0;
volatile  double oposRecord = 0.0;

// CAN总线处理
void MotorInit(AxisType xyz) 
{
	uint32_t data;
	bool ret;

    //读取电机ID
    std::cout << "读取电机ID" << std::endl;
    uint8_t ReadData[8] = {0};
    ret = SDORead_ShellFunc(can_sfd, xyz, RED_CANID, RED_CANID_SUB, ReadData);
    if( ret != true)
    {
        return;
    }
    usleep(10*1000);
    
	#if 1
     //清除错误
     std::cout << "清除错误" << std::endl;
     data = 0x80;
     ret = SDOWrite_ShellFunc(can_sfd, xyz, 0x6040, 0x00, reinterpret_cast<uint8_t*>(&data), 2);
     if(ret != true)
     {
        //std::cout << "error:" << "SDOWrite_ShellFunc(can_sfd, 1, 0x6040, 0x00, &data, 1)" << std::endl;
     }
     usleep(10*1000);
	#endif

#if 0
     //更改X电机的RPDO2为位置速度
     data = 0x80000300 + xyz; //无效RPDO2
     ret = SDOWrite_ShellFunc(can_sfd, xyz, 0x1400+xyz, 0x01, reinterpret_cast<uint8_t*>(&data), 4);
	 std::cout << "无效RPDO2成功" << std::endl;
     usleep(10*1000);

	 //清除原来映射
	 data = 0; 
     ret = SDOWrite_ShellFunc(can_sfd, xyz, 0x1601, 0x00, reinterpret_cast<uint8_t*>(&data), 4);
     usleep(10*1000);
	 std::cout << "清除原来映射成功" << std::endl;

	 //重新写入要映射的参数
	 data = 0x607A0020; //轮廓目标位置 
     ret = SDOWrite_ShellFunc(can_sfd, xyz, 0x1601, 0x01, reinterpret_cast<uint8_t*>(&data), 4);
     usleep(10*1000);
	 std::cout << "重新写入要映射的参数成功" << std::endl;

	 data = 0x60810020; //轮廓目标速度 
     ret = SDOWrite_ShellFunc(can_sfd, xyz, 0x1601, 0x02, reinterpret_cast<uint8_t*>(&data), 4);
     usleep(10*1000);
	 std::cout << "轮廓目标速度成功" << std::endl;

	 //设置映射参数个数
	 data = 0x02; 
     ret = SDOWrite_ShellFunc(can_sfd, xyz, 0x1601, 0x00, reinterpret_cast<uint8_t*>(&data), 4);
     usleep(10*1000);
	 std::cout << "设置映射参数个数成功" << std::endl;

	 //生效RPDO2
     data = 0x00000300 + xyz; 
     ret = SDOWrite_ShellFunc(can_sfd, xyz, 0x1400+xyz, 0x01, reinterpret_cast<uint8_t*>(&data), 4);
     usleep(10*1000);
	 std::cout << "生效RPDO2成功" << std::endl;

#endif

	//电机使能
	MotorEnableCtrl(xyz);
	     
    return;
}


/*电机回零初始化*/
void MotorRetZeroInit(void)
{
	uint32_t stateData = 0;
	uint32_t o_stateData = 0;
	int32_t o_reso = 0;
	uint32_t sendData = 0;
	double o_pos;
	bool ret = false;
	uint8_t stateFlag = 0;
	
	while(true)
	{
		switch (stateFlag)
		{
		case 0:
			//X电机
			MotorInit(en_X);
			//Y电机
			MotorInit(en_Y);
			//进样针电机
			MotorInit(en_O);
			//Z轴电机
			MotorInit(en_Z); 
			stateFlag++;
			//goto end;
			break;

		case 1: //Z轴与进样针回零
			#ifdef NANOTEC
			MotorRetZero(en_Z, 1800, 500, 100, 100);
			#else
			MotorRetZero(en_Z, 700, 500, 100, 100);
			MotorRetZero(en_O, 700, 500, 100, 100);
			#endif
			stateFlag++;
			break;

		case 2: //Z轴回零完成
			stateData = 0;
			while(!(((stateData>>12)&1) && ((stateData>>10)&1)))
			{
				SDORead_ShellFunc(can_sfd,en_Z,0x6041,0x00,reinterpret_cast<uint8_t*>(&stateData));
				usleep(100*1000);
			}
			zposRecord = 0;
			stateFlag++;
			break;

		case 3: //X轴回零
			#ifdef NANOTEC
			MotorRetZero(en_X, 1800, 300, 100, 100);
			#else
			MotorRetZero(en_X, 500, 500, 200, 300);
			#endif
			stateFlag++;
			break;

        case 4: //X轴回零完成
			stateData = 0;
			while(!(((stateData>>12)&1) && ((stateData>>10)&1)))
			{
				SDORead_ShellFunc(can_sfd,en_X,0x6041,0x00,reinterpret_cast<uint8_t*>(&stateData));
				usleep(100*1000);
			}
			xposRecord = 0;
			stateFlag++;
            break;

        case 5: //Y轴回零
			#ifdef NANOTEC
			MotorRetZero(en_Y, 1800, 300, 100, 100);
			#else
            MotorRetZero(en_Y, 500, 500, 100, 100);
			#endif
			stateFlag++;
            break;

		case 6: //Y轴回零完成
			stateData = 0;
			while(!(((stateData>>12)&1) && ((stateData>>10)&1)))
			{
				SDORead_ShellFunc(can_sfd,en_Y,0x6041,0x00,reinterpret_cast<uint8_t*>(&stateData));
				usleep(100*1000);
			}
			yposRecord = 0;
			stateFlag++;
			break;

		case 7: //编码器清零
			sendData = 35;
			SDOWrite_ShellFunc(can_sfd, en_X, 0x6098, 0x00, reinterpret_cast<uint8_t*>(&sendData), 1);
			usleep(15*1000);
			sendData = 0x0f;
			SDOWrite_ShellFunc(can_sfd, en_X, 0x6040, 0x00, reinterpret_cast<uint8_t*>(&sendData), 2);
			usleep(15*1000);
			sendData = 0x1f;
			SDOWrite_ShellFunc(can_sfd, en_X, 0x6040, 0x00, reinterpret_cast<uint8_t*>(&sendData), 2);
			usleep(15*1000);
			sendData = 35;
			SDOWrite_ShellFunc(can_sfd, en_Y, 0x6098, 0x00, reinterpret_cast<uint8_t*>(&sendData), 1);
			usleep(15*1000);
			sendData = 0x0f;
			SDOWrite_ShellFunc(can_sfd, en_Y, 0x6040, 0x00, reinterpret_cast<uint8_t*>(&sendData), 2);
			usleep(15*1000);
			sendData = 0x1f;
			SDOWrite_ShellFunc(can_sfd, en_Y, 0x6040, 0x00, reinterpret_cast<uint8_t*>(&sendData), 2);
			usleep(15*1000);			
			sendData = 35;
			SDOWrite_ShellFunc(can_sfd, en_Z, 0x6098, 0x00, reinterpret_cast<uint8_t*>(&sendData), 1);
			usleep(15*1000);
			sendData = 0x0f;
			SDOWrite_ShellFunc(can_sfd, en_Z, 0x6040, 0x00, reinterpret_cast<uint8_t*>(&sendData), 2);
			usleep(15*1000);
			sendData = 0x1f;
			SDOWrite_ShellFunc(can_sfd, en_Z, 0x6040, 0x00, reinterpret_cast<uint8_t*>(&sendData), 2);
			usleep(15*1000);
			stateFlag++;
			break;

		case 8:
			sendData = 0x0f;
			SDOWrite_ShellFunc(can_sfd, en_X, 0x6040, 0x00, reinterpret_cast<uint8_t*>(&sendData), 2);
			usleep(15*1000);
			SDOWrite_ShellFunc(can_sfd, en_Y, 0x6040, 0x00, reinterpret_cast<uint8_t*>(&sendData), 2);
			usleep(15*1000);
			SDOWrite_ShellFunc(can_sfd, en_Z, 0x6040, 0x00, reinterpret_cast<uint8_t*>(&sendData), 2);
			usleep(15*1000);
			SDOWrite_ShellFunc(can_sfd, en_O, 0x6040, 0x00, reinterpret_cast<uint8_t*>(&sendData), 2);
			usleep(15*1000);
			stateFlag++;						
			break;

		case 9:
			/*轮廓位置模式*/
			MotorPositionModle(en_X);
			MotorPositionModle(en_Y);
			MotorPositionModle(en_Z);
			stateFlag++;	
			break;

		case 10: //O轴切换轮廓位置模块
			MotorPositionModle(en_O);
			stateFlag++;
			break;

		case 11: //设置PI参数并保存
			stateData = 7000;
			SDOWrite_ShellFunc(can_sfd, en_X, 0x3210, 0x01, reinterpret_cast<uint8_t*>(&stateData), 4);
			usleep(15*1000);
			stateData = 4000; //位置环
			SDOWrite_ShellFunc(can_sfd, en_Y, 0x3210, 0x01, reinterpret_cast<uint8_t*>(&stateData), 4);
			usleep(15*1000);
			stateData = 5000;
			SDOWrite_ShellFunc(can_sfd, en_Z, 0x3210, 0x01, reinterpret_cast<uint8_t*>(&stateData), 4);
			usleep(15*1000);
			SDOWrite_ShellFunc(can_sfd, en_O, 0x3210, 0x01, reinterpret_cast<uint8_t*>(&stateData), 4);
			usleep(15*1000);
			stateData = 9;  //电机刚性
			SDOWrite_ShellFunc(can_sfd, en_X, 0x3210, 0x04, reinterpret_cast<uint8_t*>(&stateData), 4);
			usleep(15*1000);
			SDOWrite_ShellFunc(can_sfd, en_Y, 0x3210, 0x04, reinterpret_cast<uint8_t*>(&stateData), 4);
			usleep(15*1000);
			SDOWrite_ShellFunc(can_sfd, en_Z, 0x3210, 0x04, reinterpret_cast<uint8_t*>(&stateData), 4);
			usleep(15*1000);
			SDOWrite_ShellFunc(can_sfd, en_O, 0x3210, 0x04, reinterpret_cast<uint8_t*>(&stateData), 4);
			usleep(15*1000);
			stateData = 700;
			SDOWrite_ShellFunc(can_sfd, en_X, 0x6083, 0x00, reinterpret_cast<uint8_t*>(&stateData), 4);
			usleep(15*1000);
			stateData = 400;
			SDOWrite_ShellFunc(can_sfd, en_X, 0x6084, 0x00, reinterpret_cast<uint8_t*>(&stateData), 4);
			usleep(15*1000);
			stateData = 40;
			SDOWrite_ShellFunc(can_sfd, en_X, 0x6067, 0x00, reinterpret_cast<uint8_t*>(&stateData), 4);
			usleep(15*1000);
			stateFlag++;
			break;

		case 12:
			//进入操作状态
			NMT_ShellFunc(can_sfd, 0, 0x01);
			usleep(5);
			stateFlag++;
		break;

		case 13:
			MotorMovePosCtrl(en_MOVE, en_O, -20, 50.0);
			stateFlag++;
			break;

		case 14:
			stateData = 0;
			while(!((stateData>>10)&0x01))
			{
				
				SDORead_ShellFunc(can_sfd, en_O, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&stateData));
				
				usleep(100*1000);
			}
			stateFlag++;			
			break;

		case 15: //回零
			MotorRetZero(en_O, 500, 500, 100, 100);
			stateFlag++;
			break;

		case 16:
			stateData = 0;
			while(!(((stateData>>12)&1) && ((stateData>>10)&1)))
			{
				SDORead_ShellFunc(can_sfd,en_O,0x6041,0x00,reinterpret_cast<uint8_t*>(&stateData));
				usleep(100*1000);
			}
			oposRecord = 0;
			stateFlag++;
			break;

		case 17:
			sendData = 3000;
			SDOWrite_ShellFunc(can_sfd, en_O, 0x6083, 0x00, reinterpret_cast<uint8_t*>(&sendData), 4);
			usleep(15*1000);
			SDOWrite_ShellFunc(can_sfd, en_O, 0x6084, 0x00, reinterpret_cast<uint8_t*>(&sendData), 4);
			usleep(15*1000);
			stateFlag++;
			break;

		case 18: //编码器清零
			sendData = 35;
			SDOWrite_ShellFunc(can_sfd, en_O, 0x6098, 0x00, reinterpret_cast<uint8_t*>(&sendData), 1);
			usleep(15*1000);
			sendData = 0x0f;
			SDOWrite_ShellFunc(can_sfd, en_O, 0x6040, 0x00, reinterpret_cast<uint8_t*>(&sendData), 2);
			usleep(15*1000);
			sendData = 0x1f;
			SDOWrite_ShellFunc(can_sfd, en_O, 0x6040, 0x00, reinterpret_cast<uint8_t*>(&sendData), 2);
			usleep(15*1000);
			sendData = 0x0f;
			SDOWrite_ShellFunc(can_sfd, en_O, 0x6040, 0x00, reinterpret_cast<uint8_t*>(&sendData), 2);
			usleep(15*1000);
			stateFlag++;    
			break;

		case 19:
			MotorPositionModle(en_O);
			stateFlag++;
			break;

		case 20:
			MotorRelPosCtrl(true, en_MOVE, en_O, 5.4, 50);
			
			stateData = 0;
			while(!((stateData>>10)&0x01))
			{
				
				SDORead_ShellFunc(can_sfd, en_O, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&stateData));
				
				usleep(100*1000);
			}
			stateFlag++;
			break;

		case 21:
			MotorRelPosCtrl(true, en_MOVE, en_O, -5.0, 50);
			
			stateData = 0;
			while(!((stateData>>10)&0x01))
			{
				
				SDORead_ShellFunc(can_sfd, en_O, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&stateData));
				
				usleep(100*1000);
			}
			stateFlag++;
			break;

		case 22: //Y轴前推50mm
			MotorRelPosCtrl(true, en_MOVE, en_Y, Y_RETZERO_MOVE, 20.0);
			stateFlag++;
			break;

		case 23:
			stateData = 0;
			while(!((stateData>>10)&0x01))
			{
				
				SDORead_ShellFunc(can_sfd, en_Y, 0x6041, 0x00, reinterpret_cast<uint8_t*>(&stateData));
				
				usleep(100*1000);
			}
			stateFlag++;
			break;

		case 24: //home位回归
			//读取坐标并执行到相应位置
			ReadCoordinatesMotorExec(HOME_NAME);
			goto end;
		break;

		default:
			break;
		}
	}

end:
	return;
}


/*电机位置运行*/
void MotorPosCtrl(AcitonType motion, AxisType axis, double position, double speed)
{
	uint32_t tmpData = 0;

	volatile static double xPosRecord = 0.0;
	volatile static double yPosRecord = 0.0; 
	volatile static double zPosRecord = 0.0;
	volatile static double oPosRecord = 0.0;
	int32_t dist;
	   
	   
    if(motion == en_MOVE)
    {

		if(axis == en_X)
		{
			xPosRecord += position;

			if(xPosRecord < 0) xPosRecord=0;

			//设置目标位置
			dist = PosConvReso(axis, xPosRecord);

			//printf("%d  距离 = %f 编码器值 = %d Record = %f\r\n", axis, position, dist, xPosRecord);
		}
		else if(axis == en_Y)
		{
			yPosRecord += position;

			if(yPosRecord > 0) yPosRecord=0;

			//设置目标位置
			dist = PosConvReso(axis, yPosRecord);

			//printf("%d  距离 = %f 编码器值 = %d Record = %f\r\n", axis, position, dist, yPosRecord);
		}
		else if(axis == en_Z)
		{
			zPosRecord += position;

			if(zPosRecord > 0) zPosRecord=0;

			//设置目标位置
			dist = PosConvReso(axis, zPosRecord);

			//printf("%d  距离 = %f 编码器值 = %d Record = %f\r\n", axis, position, dist, zPosRecord);
		}
		else
		{
			oPosRecord += position;

			if(oPosRecord > 0) oPosRecord=0;

			//设置目标位置
			dist = PosConvReso(axis, oPosRecord);

			//printf("%d  距离 = %f 编码器值 = %d Record = %f\r\n", axis, position, dist, oPosRecord);
		}


		//std::cout << "dist = " << dist << std::endl;
		uint32_t sped = SpedConvReso(axis, speed);
		//std::cout << "sped = " << dist << std::endl;
		uint8_t tempbuf[8];
		memset(tempbuf,0,sizeof(tempbuf));
		memcpy(tempbuf,reinterpret_cast<uint8_t*>(&dist),4);
		memcpy(&tempbuf[4],reinterpret_cast<uint8_t*>(&sped),4);
		SendRPDO_Func(can_sfd, axis, 2, tempbuf, 8);
		usleep(5*1000);

		tmpData = 0x2F; //绝对位移
		//tmpData = 0x6F; //相对位移立即执行
		
		SendRPDO_Func(can_sfd, axis, 1, reinterpret_cast<uint8_t*>(&tmpData), 2);
		usleep(5*1000);

		tmpData = 0x3F; //绝对位移
		//tmpData = 0x7F;
		
		SendRPDO_Func(can_sfd, axis, 1, reinterpret_cast<uint8_t*>(&tmpData), 2);
		usleep(5*1000);
    }

	usleep(10*1000);
}


/*电机相对位置运行*/
void MotorMovePosCtrl(AcitonType motion, AxisType axis, double position, double speed)
{
	uint32_t tmpData = 0;

	int32_t dist;
	   

    if(motion == en_MOVE)
    {

		if(axis == en_X)
		{

			//设置目标位置
			dist = PosConvReso(axis, position);

			//printf("%d  距离 = %f 编码器值 = %d Record = %f\r\n", axis, position, dist, xposRecord);
		}
		else if(axis == en_Y)
		{

			//设置目标位置
			dist = PosConvReso(axis, position);

			//printf("%d  距离 = %f 编码器值 = %d Record = %f\r\n", axis, position, dist, yposRecord);
		}
		else if(axis == en_Z)
		{

			//设置目标位置
			dist = PosConvReso(axis, position);

			//printf("%d  距离 = %f 编码器值 = %d Record = %f\r\n", axis, position, dist, zposRecord);
		}
		else
		{

			//设置目标位置
			dist = PosConvReso(axis, position);

			//printf("%d  距离 = %f 编码器值 = %d Record = %f\r\n", axis, position, dist, oposRecord);
		}


		//std::cout << "dist = " << dist << std::endl;
		uint32_t sped = SpedConvReso(axis, speed);
		//std::cout << "sped = " << dist << std::endl;
		uint8_t tempbuf[8];
		memcpy(tempbuf,reinterpret_cast<uint8_t*>(&dist),4);
		memcpy(&tempbuf[4],reinterpret_cast<uint8_t*>(&sped),4);
		SendRPDO_Func(can_sfd, axis, 2, tempbuf, 8);
		usleep(5*1000);


		tmpData = 0x6F; //相对位移立即执行
		

        
		SendRPDO_Func(can_sfd, axis, 1, reinterpret_cast<uint8_t*>(&tmpData), 2);
		usleep(5*1000);



		tmpData = 0x7F;
		
		SendRPDO_Func(can_sfd, axis, 1, reinterpret_cast<uint8_t*>(&tmpData), 2);
		usleep(5*1000);
    }

	usleep(10*1000);
}


/*电机相对位置运行*/
void MotorRelPosCtrl(bool flg, AcitonType motion, AxisType axis, double position, double speed)
{
	uint32_t tmpData = 0;
    static bool stopFlag = false;
	static bool pauseFlag = false;

	int32_t dist;
	   
	   
	//脱机不带锁轴
    if(stopFlag)
    {
        stopFlag = false;

		uint16_t data = 0x07;  
		
		SendRPDO_Func(can_sfd, axis, 1, reinterpret_cast<uint8_t*>(&data), 2);
		usleep(5*1000);
		
    }

	if(pauseFlag)
	{
		pauseFlag = false;
		uint32_t redata;

		SDORead_ShellFunc(can_sfd, axis, 0x6040, 0x00, reinterpret_cast<uint8_t*>(&redata));
		usleep(15*1000);

		redata = redata&0xFFFFFEFF;
		
		SendRPDO_Func(can_sfd, axis, 1, reinterpret_cast<uint8_t*>(&redata), 2);
		usleep(5*1000);
	}

    if(motion == en_MOVE)
    {

		if(axis == en_X)
		{
			xposRecord += position;

			if(xposRecord < 0) xposRecord=0;

			//设置目标位置
			dist = PosConvReso(axis, xposRecord);
			printf("xposRecord = %f\r\n",  xposRecord);

			//printf("%d  距离 = %f 编码器值 = %d Record = %f\r\n", axis, position, dist, xposRecord);
		}
		else if(axis == en_Y)
		{
			yposRecord += position;

			if(yposRecord < 0) yposRecord=0;

			//设置目标位置
			dist = PosConvReso(axis, yposRecord);
			printf("yposRecord = %f\r\n",  yposRecord);

			//printf("%d  距离 = %f 编码器值 = %d Record = %f\r\n", axis, position, dist, yposRecord);
		}
		else if(axis == en_Z)
		{
			zposRecord += position;

			if(zposRecord < 0) zposRecord=0;

			//设置目标位置
			dist = PosConvReso(axis, zposRecord);

			printf("zposRecord = %f\r\n",  zposRecord);

			//printf("%d  距离 = %f 编码器值 = %d Record = %f\r\n", axis, position, dist, zposRecord);
		}
		else
		{
			oposRecord += position;

			//设置目标位置
			dist = PosConvReso(axis, oposRecord);

			printf("oposRecord = %f\r\n",  oposRecord);

			//printf("%d  距离 = %f 编码器值 = %d Record = %f\r\n", axis, position, dist, oposRecord);
		}

		if(flg)
		{
			if(axis == en_Z) dist = -dist;
			if(axis == en_Y) dist = -dist;
			if(axis == en_O) dist = -dist;
		}


		//std::cout << "dist = " << dist << std::endl;
		uint32_t sped = SpedConvReso(axis, speed);
		//std::cout << "sped = " << dist << std::endl;
		uint8_t tempbuf[8];
		memcpy(tempbuf,reinterpret_cast<uint8_t*>(&dist),4);
		memcpy(&tempbuf[4],reinterpret_cast<uint8_t*>(&sped),4);
		SendRPDO_Func(can_sfd, axis, 2, tempbuf, 8);
		usleep(5*1000);


		tmpData = 0xF; //绝对位移
		//tmpData = 0x6F; //相对位移立即执行
		

        
		SendRPDO_Func(can_sfd, axis, 1, reinterpret_cast<uint8_t*>(&tmpData), 2);
		usleep(5*1000);


		tmpData = 0x1F; //绝对位移
		//tmpData = 0x7F;
		
		SendRPDO_Func(can_sfd, axis, 1, reinterpret_cast<uint8_t*>(&tmpData), 2);
		usleep(5*1000);
    }
    else if(motion == en_STOP)
    {
        MotorStopCtrl(axis);
        stopFlag = true;
    }
	else if(motion == en_PAUSE)
	{
        MotorPauseCtrl(axis);
        pauseFlag = true;
	}

	usleep(10*1000);
}


/*电机触底判断*/
bool MotorBottominOutJudgment(AxisType axis, double cur)
{
    bool ret = false;
	int32_t tmpData = 0;
	int16_t data;

	sem_wait(CANSemID);

	SDORead_ShellFunc(can_sfd, axis, 0x6078, 0x00, reinterpret_cast<uint8_t*>(&tmpData));
	usleep(15*1000);

	sem_post(CANSemID);

	data = cur/1.3*1000;

	if(tmpData >= data)
	{
		ret = true;
	}
	else 
	{
		ret = false;
	}

	return ret;	

}

/*获取电机位置*/
double GetMotorPosition(AxisType axis)
{
	double pos;
	int32_t tmpData = 0;

	SDORead_ShellFunc(can_sfd, axis, 0x6064, 0x00, reinterpret_cast<uint8_t*>(&tmpData));
	usleep(30*1000);
	pos = ResoConvPos(axis, tmpData);

	return pos;
}

/*获取电机速度*/
double GetMotorSpeed(AxisType axis)
{
	double speed;
	int32_t tmpData = 0;

	SDORead_ShellFunc(can_sfd, axis, 0x6069, 0x00, reinterpret_cast<uint8_t*>(&tmpData));
	usleep(30*1000);
	speed = ResoConvPos(axis, tmpData);

	return speed;
}

/*电机方向控制*/
void MotorDirectionCtrl(AxisType axis, bool dir)
{
	uint8_t temp = 0;

	if(dir) //反转
	{
		temp = 1;
		SDOWrite_ShellFunc(can_sfd, axis, 0x607E, 0x00, &temp, 1); 
	}
	else //正转
	{
		temp = 0;
		SDOWrite_ShellFunc(can_sfd, axis, 0x607E, 0x00, &temp, 1); 
	}

	usleep(30*1000);
}

/*电机使能*/
void MotorEnableCtrl(AxisType axis)
{
	bool ret = false;
	uint16_t data;

	data = 0x06;
	ret = SDOWrite_ShellFunc(can_sfd, axis, 0x6040, 0x00, reinterpret_cast<uint8_t*>(&data), 2);
	usleep(15*1000);

	data = 0x07;
	ret = SDOWrite_ShellFunc(can_sfd, axis, 0x6040, 0x00, reinterpret_cast<uint8_t*>(&data), 2);
	usleep(15*1000);

	data = 0x0F;
	ret = SDOWrite_ShellFunc(can_sfd, axis, 0x6040, 0x00, reinterpret_cast<uint8_t*>(&data), 2);
	usleep(15*1000);

	 std::string axisType;
	 if(axis == en_X)
	 {
		axisType = "X";
	 }
	 else if(axis == en_Y)
	 {
		axisType = "Y";
	 }
	 else if(axis == en_Z)
	 {
		axisType = "Z";
	 }
	 else if(axis == en_O)
	 {
		axisType = "O";
	 }

     if(ret)
     {
       std::cout<< axisType << "电机使能成功" << std::endl;
     }
	 else
	 {
		std::cout<< axisType << "电机使能失败" << std::endl;
	 }
}


/*电机停机*/
void MotorStopCtrl(AxisType axis)
{
#if 1
		uint16_t data = 0x06;
		SendRPDO_Func(can_sfd, axis, 1, reinterpret_cast<uint8_t*>(&data), 2);
		usleep(5*1000);
#else 
	uint16_t data = 0x06;
	uint32_t statedata = 0;

	SDOWrite_ShellFunc(can_sfd, axis, 0x6040, 0x00, reinterpret_cast<uint8_t*>(&data), 2);
	usleep(10*1000);
	
	while((statedata&0x0001) != 0x0001)
	{
		SDORead_ShellFunc(can_sfd,axis,0x6041,0x00,reinterpret_cast<uint8_t*>(&statedata));
		usleep(10*1000);
		std::cout << "MotorStopCtrl stateData&0x0001 = " << (statedata&0x0001) << std::endl; 
	}
#endif
}


/*电机暂停*/
void MotorPauseCtrl(AxisType axis)
{
		uint32_t reData;
		uint16_t data = 0x06;

		SDORead_ShellFunc(can_sfd,axis,0x6040,0x00,reinterpret_cast<uint8_t*>(&reData));
		usleep(15*1000);

		reData = reData|0x00010000;

		SendRPDO_Func(can_sfd, axis, 1, reinterpret_cast<uint8_t*>(&reData), 2);
		usleep(5*1000);
}


/*电机回零                  轴           堵转转矩       检查时间         回零速度           回零加速度*/
void MotorRetZero(AxisType axis, int32_t torques, int32_t time, uint32_t speed, uint32_t acceSpeed)
{

	int32_t data = 0;

	#ifdef NANOTEC

	data = 6;
	SDOWrite_ShellFunc(can_sfd, axis, 0x6060, 0x00, reinterpret_cast<uint8_t*>(&data), 1);
	usleep(15*1000);

	#else

	//设置CIA402模式
	SDOWrite_ShellFunc(can_sfd, axis, 0x2002, 0x01, reinterpret_cast<uint8_t*>(&data), 2);
	usleep(15*1000);

	//设置原点回归模式
	data = 6;
	SDOWrite_ShellFunc(can_sfd, axis, 0x6060, 0x00, reinterpret_cast<uint8_t*>(&data), 1);
	usleep(15*1000);
	#endif

	//设置堵转原点回归模式
	if(axis == en_X)
	{
		data = FORWARD_TO_ZERO; //正向回零
	}
	else if(axis == en_O)
	{
		data = INVER_TO_ZERO; //负向回零
	}
	else if(axis == en_Y)
	{
		data = INVER_TO_ZERO; //负向回零
	}
	else if(axis == en_Z)
	{
		data = INVER_TO_ZERO; //反向回零
	}
	SDOWrite_ShellFunc(can_sfd, axis, 0x6098, 0x00, reinterpret_cast<uint8_t*>(&data), 1);
	usleep(15*1000);	

	//设置堵转转矩检测时间
	printf("设置堵转转矩检测时间 data = %d\r\n", time);
	SDOWrite_ShellFunc(can_sfd, axis, SET_BLOCKING_TORQUE, SET_BLOCKING_TIME_SUB, reinterpret_cast<uint8_t*>(&time), SET_BLOCKING_DATA_LEN);
	usleep(15*1000);


	//设置堵转转矩
	printf("设置堵转转矩 data = %d\r\n", torques);
	SDOWrite_ShellFunc(can_sfd, axis, SET_BLOCKING_TORQUE, SET_BLOCKING_TORQUE_SUB, reinterpret_cast<uint8_t*>(&torques), SET_BLOCKING_DATA_LEN);
	usleep(15*1000);


	//设置寻找原点速度
	uint32_t tempData;
	tempData = SpedConvReso(axis, speed);
	printf("设置寻找原点速度 data = %d\r\n", tempData);
	SDOWrite_ShellFunc(can_sfd, axis, 0x6099, 0x02, reinterpret_cast<uint8_t*>(&tempData), 4);
	usleep(15*1000);

	//设置寻找原点加速度
	tempData = SpedConvReso(axis, acceSpeed);
	printf("设置寻找原点加速度 data = %d\r\n", tempData);
	SDOWrite_ShellFunc(can_sfd, axis, 0x609A, 0x00, reinterpret_cast<uint8_t*>(&tempData), 4);
	usleep(15*1000);

	//使能电机	
	data = 6;
	SDOWrite_ShellFunc(can_sfd, axis, 0x6040, 0x00, reinterpret_cast<uint8_t*>(&data), 2);
	usleep(15*1000);

	data = 7;
	SDOWrite_ShellFunc(can_sfd, axis, 0x6040, 0x00, reinterpret_cast<uint8_t*>(&data), 2);
	usleep(15*1000);

	data = 0x0f;
	SDOWrite_ShellFunc(can_sfd, axis, 0x6040, 0x00, reinterpret_cast<uint8_t*>(&data), 2);
	usleep(15*1000);

	data = 0x1f;
	SDOWrite_ShellFunc(can_sfd, axis, 0x6040, 0x00, reinterpret_cast<uint8_t*>(&data), 2);
	usleep(15*1000);			
}


/*轮廓位置模式*/
void MotorPositionModle(AxisType axis)
{
	uint16_t data;
	bool ret = false;

	#ifdef NANOTEC

	data = 1;
	ret = SDOWrite_ShellFunc(can_sfd, axis, 0x6060, 0x00, reinterpret_cast<uint8_t*>(&data), 1);
	usleep(15*1000);

	#else

	 //设置CIA402模式
     data = 0;
     ret = SDOWrite_ShellFunc(can_sfd, axis, 0x2002, 0x01, reinterpret_cast<uint8_t*>(&data), 2);
     usleep(15*1000);

     //设置轮廓位置模式
     data = 0x01;
     ret = SDOWrite_ShellFunc(can_sfd, axis, 0x6060, 0x00, reinterpret_cast<uint8_t*>(&data), 1);
	 usleep(15*1000);

	 #endif

	 std::string axisType;
	 if(axis == en_X)
	 {
		axisType = "X";
	 }
	 else if(axis == en_Y)
	 {
		axisType = "Y";
	 }
	 else if(axis == en_Z)
	 {
		axisType = "Z";
	 }
	 else if(axis == en_O)
	 {
		axisType = "O";
	 }

     if(ret)
     {
       std::cout << axisType << "切换轮廓位置模式成功" << std::endl;
     }
	 else
	 {
		std::cout << axisType << "切换轮廓位置模式失败" << std::endl;
	 }
}


/*位置距离换算编码器分辨率*/
int32_t PosConvReso(AxisType axis, double position)
{
	int32_t ret;

	switch (axis)
	{
	case en_X:
		ret = position / X_AXIS_PITCH * ENCODER_RESOLUTION;
		break;

	case en_Y:
		ret = position / Y_AXIS_PITCH * ENCODER_RESOLUTION;
		break;

	case en_Z:
		ret = position / Z_AXIS_PITCH * ENCODER_RESOLUTION;
		break;

	case en_O:
		ret = position / O_AXIS_PITCH * ENCODER_RESOLUTION;
		break;

	default:
		break;
	}

	return ret;
}

/*编码器分辨率换算位置距离*/
double ResoConvPos(AxisType axis, int32_t resolution)
{
	double ret;

	switch (axis)
	{
	case en_X:
		ret = (static_cast<double>(resolution) / ENCODER_RESOLUTION) * X_AXIS_PITCH;
		break;

	case en_Y:
		ret = (static_cast<double>(resolution) / ENCODER_RESOLUTION) * Y_AXIS_PITCH;
		break;

	case en_Z:
		ret = (static_cast<double>(resolution) / ENCODER_RESOLUTION) * Z_AXIS_PITCH;
		break;	

	case en_O:
		ret = (static_cast<double>(resolution) / ENCODER_RESOLUTION) * O_AXIS_PITCH;
		break;			

	default:
		break;
	}

	return ret;
}

/*速度换算编码器分辨率 v/s*/
uint32_t SpedConvReso(AxisType axis, double speed)
{
	uint32_t ret;

	#ifndef NANOTEC
	ret = speed / 60.0  * ENCODER_RESOLUTION;
	#else
	ret = static_cast<uint32_t>(speed);
	#endif

	return ret;
}


/*编码器分辨率换算速度 v/s*/
double ResoConvSped(AxisType axis, int32_t resolution)
{
	double ret;

	resolution = abs(resolution);

	#ifndef NANOTEC
	ret = static_cast<double>(resolution) / ENCODER_RESOLUTION * 60;
	#else 
	ret = static_cast<double>(resolution);
	#endif

	return ret;
}

#if 0
     //配置TPDO3为非循环同步模式
     std::cout << "配置TPDO3为非循环同步模式" << std::endl;
     data = 0x80000380 + 1; //无效PDO3
     ret = SDOWrite_ShellFunc(can_sfd, xyz, 0x1802, 0x01, reinterpret_cast<uint8_t*>(&data), 4);
     if(ret != true)
     {
        //std::cout << "error:" << "SDOWrite_ShellFunc(can_sfd, 1, 0x2002, 0x01, reinterpret_cast<uint8_t*>(&data), 2)" << std::endl;
     }
     usleep(10*1000);
     data = 0x00; //非循环同步
     ret = SDOWrite_ShellFunc(can_sfd, xyz, 0x1802, 0x02, reinterpret_cast<uint8_t*>(&data), 1);
     if(ret != true)
     {
        //std::cout << "error:" << "SDOWrite_ShellFunc(can_sfd, 1, 0x2002, 0x01, reinterpret_cast<uint8_t*>(&data), 2)" << std::endl;
     }
     usleep(10*1000);     
     data = 0x00000380 + 1; //生效PDO3
     ret = SDOWrite_ShellFunc(can_sfd, xyz, 0x1802, 0x01, reinterpret_cast<uint8_t*>(&data), 4);
     if(ret != true)
     {
        //std::cout << "error:" << "SDOWrite_ShellFunc(can_sfd, 1, 0x2002, 0x01, reinterpret_cast<uint8_t*>(&data), 2)" << std::endl;
     }
     usleep(10*1000);

     //配置TPDO4为非循环同步模式
     std::cout << "配置TPDO4为非循环同步模式" << std::endl;
     data = 0x80000480 + 1; //无效PDO4
     ret = SDOWrite_ShellFunc(can_sfd, xyz, 0x1803, 0x01, reinterpret_cast<uint8_t*>(&data), 4);
     if(ret != true)
     {
        //std::cout << "error:" << "SDOWrite_ShellFunc(can_sfd, 1, 0x2002, 0x01, reinterpret_cast<uint8_t*>(&data), 2)" << std::endl;
     }
     usleep(10*1000);
     data = 0x00; //非循环同步
     ret = SDOWrite_ShellFunc(can_sfd, xyz, 0x1803, 0x02, reinterpret_cast<uint8_t*>(&data), 1);
     if(ret != true)
     {
        //std::cout << "error:" << "SDOWrite_ShellFunc(can_sfd, 1, 0x2002, 0x01, reinterpret_cast<uint8_t*>(&data), 2)" << std::endl;
     }
     usleep(10*1000);     
     data = 0x00000480 + 1; //生效PDO4
     ret = SDOWrite_ShellFunc(can_sfd, xyz, 0x1803, 0x01, reinterpret_cast<uint8_t*>(&data), 4);
     if(ret != true)
     {
        //std::cout << "error:" << "SDOWrite_ShellFunc(can_sfd, 1, 0x2002, 0x01, reinterpret_cast<uint8_t*>(&data), 2)" << std::endl;
     }
     usleep(10*1000);
#endif