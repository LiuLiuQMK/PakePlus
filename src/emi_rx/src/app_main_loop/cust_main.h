#ifndef _CUSTOMER_MAIN_H_
#define _CUSTOMER_MAIN_H_

#include <stdint.h>
#include <stdio.h>

void custom_main_init(void);
uint32_t custom_main_loop(void);
void custom_sys_wakeup_callback(void);
void custom_sys_sleep_prepare(void);


#endif

