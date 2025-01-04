/**
 * @file platform_sys.h
 * @brief 
 * @author wpc (dchlcb@163.com)
 * @version 1.0
 * @date 2025-01-04
 * 
 * @copyright Copyright (c) 2025, Dchlcb Development Team
 * 
 * @logs:
 * Date           Version     Author      Description
 * 2025-01-04     v1.0        wpc         the first version
 */


#ifndef __PLATFORM_SYS_H__
#define __PLATFORM_SYS_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>


#ifdef __cplusplus
extern "C" {
#endif

void delay_ms(unsigned int milliseconds);
void delay_us(unsigned int microseconds);



#ifdef __cplusplus
}
#endif


#endif

