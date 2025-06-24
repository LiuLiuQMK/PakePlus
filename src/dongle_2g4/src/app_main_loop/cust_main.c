#include "cust_main.h"
#include "platform_api.h"
#include "uart_console.h"
#include "app_2g4.h"
#include "bsp_usb_hid_iap.h"
#include "profile.h"
/********************************************************************************
 ****************************** 用户main文件 *************************************
 ********************************************************************************/
static uint32_t app_busy_flag = 0;

// 此函数在系统启动时初始化一次
void custom_main_init(void) {
    return;
}

// 此函数为主循环函数，相当于它在一个while(1)中被循环调用
// 当返回值为0时，系统会尝试进入睡眠状态，请在所有循环任务执行完毕时再将睡眠状态设置为0；
uint32_t custom_main_loop(void) {
    app_2g4_loop();
	
	app_mode_loop();
	
    loop_uart_console();

    app_busy_flag = 0xFFFFFFFF; // keep wake up

    return app_busy_flag; //当有任务忙的时候，系统不会进入睡眠状态；
}

// 此函数在系统唤醒时回调一次
void custom_sys_wakeup_callback(void) {
    return;
}

// 此函数在系统真正睡眠前调用一次
// 特别注意：睡眠前如果调用printf或log_printf打印了串口信息，则睡眠系统会等待串口FIFO数据发送完成后才会真正进入睡眠
void custom_sys_sleep_prepare(void) {
    return;
}
