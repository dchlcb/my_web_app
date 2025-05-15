#ifndef MAIN_BOARD_MCU_H
#define MAIN_BOARD_MCU_H

#include "Public.h"
#include <semaphore.h>
#include <sys/mman.h>


//主板MCU通讯线程
void* MainBoardMCUFunc(void* arg);

extern int spi_sfd;



#endif