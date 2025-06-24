#ifndef _IAP_916_H_
#define _IAP_916_H_

#include <stdint.h>
#include "IAP_FLASH_MAP.H"



extern void Uart_Send_Complete_Check(void);
void IAP_Init(void);
void JumpToApp(uint32_t addr);

#endif


