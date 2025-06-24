#include <stdio.h>
#include <string.h>
#include "profile.h"
#include "ingsoc.h"
#include "platform_api.h"
#include "port_gen_os_driver.h"
#include "trace.h"
#include "ing_2p4g.h"
#include "uart_console.h"
#include "IAP_916.h"

static uint32_t cb_hard_fault(hard_fault_info_t *info, void *_)
{
    platform_printf("HARDFAULT:\nPC : 0x%08X\nLR : 0x%08X\nPSR: 0x%08X\n"
                    "R0 : 0x%08X\nR1 : 0x%08X\nR2 : 0x%08X\nP3 : 0x%08X\n"
                    "R12: 0x%08X\n",
                    info->pc, info->lr, info->psr,
                    info->r0, info->r1, info->r2, info->r3, info->r12);
    
    #if RELEASE_VER
		platform_reset();
    #else
		trace_full_dump2(puts, 56, 8);
    #endif
    for (;;);
}

static uint32_t cb_assertion(assertion_info_t *info, void *_)
{
    platform_printf("[ASSERTION] @ %s:%d\n",
                    info->file_name,
                    info->line_no);
    
    #if RELEASE_VER
		platform_reset();
    #else
		trace_full_dump2(puts, 56, 8);
    #endif
    for (;;);
}

static uint32_t cb_heap_out_of_mem(uint32_t tag, void *_)
{
    platform_printf("[OOM] @ %d\n", tag);
    
    for (;;);
    #if RELEASE_VER
		platform_reset();
    #else
		trace_full_dump2(puts, 56, 8);
    #endif
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

    #if RELEASE_VER
		apUART_Initialize(PRINT_PORT, &config, 0);
    #else
		apUART_Initialize(PRINT_PORT, &config, (1 << bsUART_RECEIVE_INTENAB) | (1 << bsUART_TIMEOUT_INTENAB));
    #endif
}

void setup_peripherals(void)
{
    config_uart(OSC_CLK_FREQ, 460800);
    
    SYSCTRL_ClearClkGateMulti(  (1 << SYSCTRL_ClkGate_APB_GPIO0)
                          | (1 << SYSCTRL_ClkGate_APB_PinCtrl)
                          | (1 << SYSCTRL_ClkGate_APB_TMR0)
                          | (1 << SYSCTRL_ClkGate_APB_GPIO1));
}

uint32_t on_lle_init(void *dummy, void *user_data)
{
    (void)(dummy);
    (void)(user_data);
    return 0;
}

uint32_t on_deep_sleep_wakeup(void *dummy, void *user_data)
{
    (void)(dummy);
    (void)(user_data);
    setup_peripherals();
    return 0;
}

uint32_t query_deep_sleep_allowed(void *dummy, void *user_data)
{
    (void)(dummy);
    (void)(user_data);
    // TODO: return 0 if deep sleep is not allowed now; else deep sleep is allowed
    return 0;
}

trace_rtt_t trace_ctx = {0};

uint32_t uart_isr(void *user_data)
{
    uint32_t status;

    while(1)
    {
        status = apUART_Get_all_raw_int_stat(APB_UART0);
        if (status == 0)
            break;

        APB_UART0->IntClear = status;

        while (apUART_Check_RXFIFO_EMPTY(APB_UART0) != 1)
        {
            char c = APB_UART0->DataRead;
            console_rx_data(&c, 1);
//            log_printf("%c ", c);
        }
    }
    return 0;
}

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
        [PLATFORM_CB_EVT_ON_DEEP_SLEEP_WAKEUP] = {
            .f = (f_platform_evt_cb)on_deep_sleep_wakeup,
        },
        [PLATFORM_CB_EVT_QUERY_DEEP_SLEEP_ALLOWED] = {
            .f = query_deep_sleep_allowed,
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

// TODO: add RTOS source code to the project.
uintptr_t app_main()
{
    ing2p4g_init_dual_mode();
    // setup event handlers
    platform_set_evt_callback_table(&evt_cb_table);

    platform_config(PLATFORM_CFG_POWER_SAVING, PLATFORM_CFG_ENABLE);    
    
	SYSCTRL_Init();
    
	setup_peripherals();

    platform_set_irq_callback(PLATFORM_CB_IRQ_UART0, uart_isr, NULL);
    
	trace_rtt_init(&trace_ctx);
    
	// TODO: config trace mask
    platform_config(PLATFORM_CFG_TRACE_MASK, 0);

    return (uintptr_t)os_impl_get_driver();
}

