#ifndef EXCUTE_ACTION_H
#define EXCUTE_ACTION_H

#include <unistd.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <errno.h>
#include "TypeDefine.h"
#include "CANOpen.h"



//电机动作
typedef struct
{
	char name[128];
	AcitonType en_motion; //动作
	AxisType   en_axis; //轴
	double 	   d_distance; //距离
	double     d_speed; //速度	
}ExcuteActionsType;

/*电机控制线程*/
void* ExcuteActionsFunc(void* arg);


#endif