#ifndef _INCUBATION_MODULE_H
#define _INCUBATION_MODULE_H

#include "Public.h"
#include <semaphore.h>
#include <sys/mman.h>


//孵化模块MCU通讯线程
void* IBBoardMCUFunc(void* arg);

extern int IB_sfd;




#endif