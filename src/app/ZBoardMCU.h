#ifndef _ZBOARD_MCU_H
#define _ZBOARD_MCU_H

#include "Public.h"
#include <semaphore.h>
#include <sys/mman.h>

//孵化模块MCU通讯线程
void* ZBoardMCUFunc(void* arg);

extern int Z_sfd;



#endif