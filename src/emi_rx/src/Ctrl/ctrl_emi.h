/* $(license) */

#ifndef __CTRL_EMI_H__
#define __CTRL_EMI_H__

#include "ctrl_pch.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "ingsoc.h"
#include "platform_api.h"



typedef bool (*emi_sweep_timer_cb_t)(void);
void EMI_Timer_Init(void);
void EMI_Timer_Start(uint8_t sec, emi_sweep_timer_cb_t cb);
void EMI_Timer_Stop(void);



typedef enum{
    EMI_RESULT_SUCCESS = 0,
} EMI_Result_t;

EMI_Result_t EMI_Dispatch(uint8_t *data, uint16_t len);

uint8_t emi_init(void);
uint8_t emi_loop(void);

/**
 * @brief Enable or disable the EMI module.
 *
 * @param enable A boolean value indicating whether to enable (true) or disable (false) 
 *               the EMI module.
 * 
 * @note When `g_active_flag` is 0, the `emi_loop` function will exit directly.
 */
void emi_enable(bool enable);

#endif