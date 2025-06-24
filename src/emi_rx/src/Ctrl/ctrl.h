/* $(license) */

#ifndef __CTRL_CTRL_H__
#define __CTRL_CTRL_H__

#include "ctrl_pch.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "ingsoc.h"
#include "platform_api.h"

uint8_t ucm_init(void);
uint8_t ucm_loop(void);
void ucm_emi_enable(bool enable);

#endif
