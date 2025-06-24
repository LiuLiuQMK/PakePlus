#include "cust_main.h"
#include "platform_api.h"
#include "bsp_usb_hid_iap.h"
#include "ctrl.h"
/********************************************************************************
 ****************************** 用户main文件 *************************************
 ********************************************************************************/

static uint32_t app_busy_flag = 0;

// 测试函数
static uint32_t tick_test = 0;
static void print_test_loop(void)
{
    if(tick_test == 0)
        tick_test = platform_get_us_time();
    
    // 测试循环，每隔一秒打印一次。
    if(platform_get_us_time() - tick_test >= 10000)
    { 
        tick_test = platform_get_us_time();
        platform_printf("main loop\n");
    }
}

// 硬件初始化函数
static void hardware_init(void)
{

}

// 此函数在系统启动时初始化一次
void custom_main_init(void)
{
    hardware_init();
}

// 此函数为主循环函数，相当于它在一个while(1)中被循环调用
// 当返回值为0时，系统会尝试进入睡眠状态，请在所有循环任务执行完毕时再将睡眠状态设置为0；
uint32_t custom_main_loop(void)
{
    // 主循环测试函数。
    // print_test_loop();
    
    ucm_loop(); //EMI 测试相关

    app_busy_flag = 0xFFFFFFFF; // keep wake up
    // app_busy_flag = 0; // try to sleep
    return app_busy_flag; //当有任务忙的时候，系统不会进入睡眠状态；
}

// 此函数在系统唤醒时回调一次
void custom_sys_wakeup_callback(void)
{
    hardware_init();
}

// 此函数在系统真正睡眠前调用一次
// 特别注意：睡眠前如果调用printf或platform_printf打印了串口信息，则睡眠系统会等待串口FIFO数据发送完成后才会真正进入睡眠
void custom_sys_sleep_prepare(void)
{

}
