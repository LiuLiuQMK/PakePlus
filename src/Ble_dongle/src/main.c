#include <stdio.h>
#include <string.h>
#include "profile.h"
#include "ingsoc.h"
#include "platform_api.h"
#include "FreeRTOS.h"
#include "task.h"
#include "trace.h"
#include "kv_storage.h"
#include "eflash.h"
#include "bsp_usb_hid_iap.h"

static uint32_t cb_hard_fault(hard_fault_info_t *info, void *_)
{
    platform_printf("HARDFAULT:\nPC : 0x%08X\nLR : 0x%08X\nPSR: 0x%08X\n"
                    "R0 : 0x%08X\nR1 : 0x%08X\nR2 : 0x%08X\nR3 : 0x%08X\n"
                    "R12: 0x%08X\n",
                    info->pc, info->lr, info->psr,
                    info->r0, info->r1, info->r2, info->r3, info->r12);
    for (;;);
}

static uint32_t cb_assertion(assertion_info_t *info, void *_)
{
    platform_printf("[ASSERTION] @ %s:%d\n",
                    info->file_name,
                    info->line_no);
    for (;;);
}

static uint32_t cb_heap_out_of_mem(uint32_t tag, void *_)
{
    platform_printf("[OOM] @ %d\n", tag);
    for (;;);
}

#define PRINT_PORT    APB_UART0

uint32_t cb_putc(char *c, void *dummy)
{
    while (apUART_Check_TXFIFO_FULL(PRINT_PORT) == 1);
    UART_SendData(PRINT_PORT, (uint8_t)*c);
    return 0;
}

int fputc(int ch, FILE *f)
{
    cb_putc((char *)&ch, NULL);
    return ch;
}

void config_uart(uint32_t freq, uint32_t baud)
{
    UART_sStateStruct config;

    config.word_length       = UART_WLEN_8_BITS;
    config.parity            = UART_PARITY_NOT_CHECK;
    config.fifo_enable       = 1;
    config.two_stop_bits     = 0;
    config.receive_en        = 1;
    config.transmit_en       = 1;
    config.UART_en           = 1;
    config.cts_en            = 0;
    config.rts_en            = 0;
    config.rxfifo_waterlevel = 1;
    config.txfifo_waterlevel = 1;
    config.ClockFrequency    = freq;
    config.BaudRate          = baud;

    apUART_Initialize(PRINT_PORT, &config, 0);
}


static uint32_t IRQHandler_TIMER2_INT(void* user_data)
{
    static int8_t led_sta = 0;

    TMR_IntClr(APB_TMR2, 0, 0x1);
    
    if(led_sta){
        GIO_WriteValue(LED_3, LED_OFF);
        led_sta = 0;
    } else {
        GIO_WriteValue(LED_3, LED_ON);
        led_sta = 1;
    }
    
    TMR_Enable(APB_TMR2, 0, 0x00);
    switch(ble_state_get()){
        case KB_BLE_STATE_SCAN:
            TMR_SetReload(APB_TMR2, 0, 1000000);   // 10us 
            TMR_Enable(APB_TMR2, 0, 1);
            break;
        case KB_BLE_STATE_CONNECT:
            TMR_SetReload(APB_TMR2, 0, 3000000);   // 10us 
            TMR_Enable(APB_TMR2, 0, 1);
            break;
        case KB_BLE_STATE_COMMUNICATING:
            GIO_WriteValue(LED_3, LED_ON);
            break;
        default:
            log_printf("[ERR]: unknown state:%d\n", ble_state_get());
    }
//    log_printf("111\n");
    return 0;
}

void setup_peripherals(void)
{
    config_uart(OSC_CLK_FREQ, 460800);
    bsp_usb_disable();
    bsp_usb_init();
	
    SYSCTRL_ClearClkGateMulti(  (1 << SYSCTRL_ClkGate_APB_GPIO0)
                      | (1 << SYSCTRL_ClkGate_APB_PinCtrl)
                      | (1 << SYSCTRL_ITEM_APB_SysCtrl)
                      | (1 << SYSCTRL_ITEM_APB_TMR2)
                      | (1 << SYSCTRL_ClkGate_APB_GPIO1));
	
	/*³õÊ¼»¯ µÆ¹â IO*/
    PINCTRL_SetPadMux(LED_1, IO_SOURCE_GPIO);
	GIO_WriteValue(LED_1, LED_ON);
    GIO_SetDirection(LED_1, GIO_DIR_OUTPUT);
    GIO_WriteValue(LED_1, LED_ON);

    PINCTRL_SetPadMux(LED_2, IO_SOURCE_GPIO);
    GIO_WriteValue(LED_2, LED_ON);
	GIO_SetDirection(LED_2, GIO_DIR_OUTPUT);
    GIO_WriteValue(LED_2, LED_ON);
	
    PINCTRL_SetPadMux(LED_3, IO_SOURCE_GPIO);
    GIO_WriteValue(LED_3, LED_ON);
	GIO_SetDirection(LED_3, GIO_DIR_OUTPUT);
    GIO_WriteValue(LED_3, LED_ON);
    
    TMR_SetOpMode(APB_TMR2, 0, TMR_CTL_OP_MODE_32BIT_TIMER_x1, TMR_CLK_MODE_EXTERNAL, 0);//timer mode set
    TMR_IntEnable(APB_TMR2, 0, 0x01);
    TMR_Enable(APB_TMR2, 0, 0x01);
    TMR_SetReload(APB_TMR2, 0, 600000);
    platform_set_irq_callback(PLATFORM_CB_IRQ_TIMER2, IRQHandler_TIMER2_INT, 0);
}

uint32_t on_lle_init(void *dummy, void *user_data)
{
    (void)(dummy);
    (void)(user_data);
    return 0;
}

trace_rtt_t trace_ctx = {0};


static const platform_evt_cb_table_t evt_cb_table =
{
    .callbacks = {
        [PLATFORM_CB_EVT_HARD_FAULT] = {
            .f = (f_platform_evt_cb)cb_hard_fault,
        },
        [PLATFORM_CB_EVT_ASSERTION] = {
            .f = (f_platform_evt_cb)cb_assertion,
        },
        [PLATFORM_CB_EVT_HEAP_OOM] = {
            .f = (f_platform_evt_cb)cb_heap_out_of_mem,
        },
        [PLATFORM_CB_EVT_PROFILE_INIT] = {
            .f = setup_profile,
        },
        [PLATFORM_CB_EVT_LLE_INIT] = {
            .f = on_lle_init,
        },
        [PLATFORM_CB_EVT_PUTC] = {
            .f = (f_platform_evt_cb)cb_putc,
        },
        [PLATFORM_CB_EVT_TRACE] = {
            .f = (f_platform_evt_cb)cb_trace_rtt,
            .user_data = &trace_ctx,
        },
    }
};

#define DB_FLASH_ADDRESS  0x2042000

int db_write_to_flash(const void *db, const int size)
{
    log_printf("write to flash, size = %d\n", size);
    program_flash(DB_FLASH_ADDRESS, (const uint8_t *)db, size);
    return KV_OK;
}

int read_from_flash(void *db, const int max_size)
{
    memcpy(db, (void *)DB_FLASH_ADDRESS, max_size);
    return KV_OK;
}

int app_main()
{
    SYSCTRL_Init();
    // setup event handlers
    platform_set_evt_callback_table(&evt_cb_table);

    setup_peripherals();
    // kv_init(db_write_to_flash, read_from_flash);

    trace_rtt_init(&trace_ctx);
    // TODO: config trace mask
    platform_config(PLATFORM_CFG_TRACE_MASK, 0);

    return 0;
}

