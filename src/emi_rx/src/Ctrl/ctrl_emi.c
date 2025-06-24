/* $(license) */

/* 
 * This file is used to handle the logic related to the EMI module.
 */
 

#include "ctrl_emi.h"
#include "gap.h"
#include "btstack_event.h"

#include "bsp_usb_hid_iap.h"
#include "profile.h"

#include "peripheral_timer.h"
#include "TEST_emi.h"

#define LOG_ERR platform_printf
#define LOG_INFO platform_printf
#define LOG_DEBUG platform_printf

#define EMI_Unexpected_Occurence() do {\
    LOG_DEBUG("Unexpected in: %s line:%d\n", __FILE__, __LINE__);\
    while(1);\
}while(0);




typedef enum {
    STATE_NORMAL,
    STATE_SWEEP,
} EMI_Sweep_State_e;

typedef enum {
    STATE_IDLE,
    STATE_STARTUP,
    STATE_IN_PROGRESS,
    STATE_SHUTDOWN,
} EMI_State_e;

typedef enum {
    STATE_BEGIN,
    STATE_STOPING,
    STATE_STARTING,
    STATE_START_SWEEP_TIMER,
    STATE_END,
} EMI_Start_Test_Process_State_e;

typedef enum {
    STATE_STOP_BEGIN,
    STATE_STOP_SWEEP,
    STATE_STOP_STOPING_RUNNING_TEST,
    STATE_STOP_END,
} EMI_Stop_Test_Process_State_e;

typedef enum {
    STATE_FORWARD_BEGIN,
    STATE_FORWARD_STOPING,
    STATE_FORWARD_STARTING,
    STATE_FORWARD_ITERATION,
    STATE_FORWARD_END,
} EMI_Forward_Test_Process_State_e;

typedef enum {
    STATE_REQUEST_IDLE,
    STATE_REQUEST_IN_PROGRESS,
} EMI_Request_State_e;


typedef enum {
    START_TX_SINGLE_CARRIER    = 0xb0,
    START_TX_SINGLE_PAYLOAD    = 0xb1,
    START_TX_SWEEP_CARRIER     = 0xb2,
    START_TX_SWEEP_PAYLOAD     = 0xb3,
    START_TX_SWEEP_PAYLOAD_V2  = 0xb4,
    START_TX_SWEEP_24G_PAYLOAD = 0xb5,
    START_TX_SWEEP_24G_CARRIER = 0xb6,
    START_RX                   = 0xb7,
    STOP_TEST                  = 0xb8,
    SWEEP_TX_V4_FORWARD        = 0xb9,
    SWEEP_CARRIER_FORWARD      = 0xba,
    CUS_SWEEP_BLE_TX_V4_FORWARD = 0xbb,
    CUS_SWEEP_24G_TX_V4_FORWARD = 0xbc,
    CUS_SWEEP_24G_CARRIER_FORWARD  = 0xbd,
} EMI_Test_Request_e;

typedef enum {
    TASK_TYPE_NONE                 = 0,
    TASK_TYPE_TX_V4                = 1,
    TASK_TYPE_RX_V2                = 2,
    TASK_TYPE_SWEEP_TX_V4          = 3,
    TASK_TYPE_CW                   = 4,
    TASK_TYPE_SWEEP_CW             = 5,
    TASK_TYPE_CUS_SWEEP_TX_V4      = 6,
    TASK_TYPE_CUS_SWEEP_24G_TX_V4  = 7,
    TASK_TYPE_CUS_SWEEP_24G_CW     = 8,
} EMI_Task_Type_e;

typedef enum {
    TX_POWER_0DB               = 0x00,
    TX_POWER_2DB               = 0x01,
    TX_POWER_3DB               = 0x02,
    TX_POWER_4DB               = 0x03,
    TX_POWER_5DB               = 0x04,
    TX_POWER_6DB               = 0x05,
    TX_POWER_NAG2DB            = 0x06,
    TX_POWER_NAG4DB            = 0x07,
    TX_POWER_NAG8DB            = 0x08,
    TX_POWER_NAG12DB           = 0x09,
    TX_POWER_NAG16DB           = 0x0A,
    TX_POWER_NAG10DB           = 0x0B,
    TX_POWER_NAG20DB           = 0x0C,
    TX_POWER_NAG25DB           = 0x0D,
    TX_POWER_NAG63DB           = 0x0E,
} EMI_Tx_Power_e;

typedef enum {
    EMI_PHY_1M                 = 0x00,
    EMI_PHY_2M                 = 0x01,
    EMI_PHY_CODED_S8           = 0x02,
    EMI_PHY_CODED_S2           = 0x03,
} EMI_Phy_e;

typedef enum {
    WAVE_CARRIER               = 0x00,
    WAVE_PRBS9                 = 0x01,
    WAVE_REPEAT10101010        = 0x02,
    WAVE_REPEAT11110000        = 0x03,
} EMI_Wave_Type_e;

typedef enum {
    RF_TEST_MODE_SINGLE        = 0x00,
    RF_TEST_MODE_SWEEP         = 0x01,
    RF_TEST_MODE_OTHER_SWEEP   = 0x02,
} EMI_Test_Mode_e;

typedef enum {
    EMI_CUS_SWEEP_BLE        = 0x00,
    EMI_CUS_SWEEP_24G        = 0x01,
} EMI_Custom_sweep_type_e;

#define FREQ_TO_TEST4(v) (((v) >> 1) - 1)
#define FREQ_TO_CW(v) (2400 + (v))

#pragma pack (push, 1)

typedef struct
{
    uint8_t channel;
    uint8_t channel1;
    uint8_t reserve;
    uint8_t reserve1;
} usb_rf_single_mode_t;

typedef struct
{
    uint8_t channel_min;
    uint8_t channel_max;
    uint8_t reserve;
    uint8_t sweep_time;
} usb_rf_sweep_mode_t;

typedef struct
{
    uint8_t type;
    uint8_t reserve;
    uint8_t reserve1;
    uint8_t sweep_time;
} usb_rf_custom_sweep_mode_t;

typedef struct
{
    uint8_t head;
    uint8_t packetCode;
    uint8_t is_start;
    uint8_t is_rx;
    uint8_t payload_model;
    uint8_t payload_len;
    uint8_t mode;
    union {
        usb_rf_single_mode_t single;
        usb_rf_sweep_mode_t sweep;
        usb_rf_custom_sweep_mode_t custom_sweep;
    } mode_data;
    uint8_t tx_power;
    uint8_t phy;
} usb_rf_cmd_t; /* 19byte */

#pragma pack (pop)


typedef struct {
    uint16_t freq_min;
    uint16_t freq_max;
    uint16_t freq_current;
    
    int8_t tx_power_level;
} sweep_carrier_task_context_t;

typedef struct {
    uint8_t channel_min;
    uint8_t channel_max;
    uint8_t channel_current;
    
    uint8_t test_data_length;
    uint8_t packet_payload;
    uint8_t phy;
    int8_t tx_power_level;
} sweep_tx_v4_task_context_t;

typedef struct {
    uint16_t channel_current;
    uint8_t test_data_length;
    uint8_t packet_payload;
    uint8_t phy;
    int8_t tx_power_level;
    uint8_t step;
} custom_sweep_task_context_t;

static int g_req;
static usb_rf_cmd_t g_cmd;
static sweep_carrier_task_context_t sweep_carrier_task_ctx;
static sweep_tx_v4_task_context_t sweep_tx_v4_task_ctx;
static custom_sweep_task_context_t custom_sweep_task_ctx;
static bool g_active_flag = false;
static volatile EMI_Sweep_State_e g_sweep_state;
static volatile EMI_State_e g_state;
static volatile EMI_State_e g_state_prev;
static volatile EMI_Request_State_e g_state_request;
static volatile EMI_Stop_Test_Process_State_e g_stop_process_state;
static volatile EMI_Start_Test_Process_State_e g_start_process_state;
static volatile EMI_Forward_Test_Process_State_e g_forward_process_state;
static volatile EMI_Task_Type_e running_task = TASK_TYPE_NONE;
static int rx_received_packet_num = 0;
static volatile bool command_completed = false;
static emi_sweep_timer_cb_t emi_sweep_timer_cb;

#define EMI_TEST_START_TIMEOUT_US 200000
uint64_t timeout_timepoint;
uint64_t current_time;

void emi_test_set_timeout(uint64_t timeout)
{
    timeout_timepoint = platform_get_us_time() + timeout;
}

/**
 * @brief Waits for a condition to be true or until a timeout occurs.
 * 
 * This function waits for a condition to be true or until a specified timeout occurs.
 * If the condition becomes true before the timeout, the function returns true.
 * If the timeout occurs before the condition becomes true, the function returns true.
 * 
 * @param condition A boolean value indicating the condition to wait for.
 * @return true if the condition becomes true or if a timeout occurs, otherwise false.
 */
bool emi_test_wait_timeout(bool condition)
{
    if (condition)
        return true;
    
    return current_time >= timeout_timepoint;
}

typedef struct
{
    int req;
    usb_rf_cmd_t content;
}EMI_Request_t;
#define REQUEST_QUEUE_DEPTH 8
static EMI_Request_t queue[REQUEST_QUEUE_DEPTH];
static volatile int queue_rp = 0;
static volatile int queue_wp = 0;

static bool queue_is_full() {
    int tmp = queue_rp + 1;
    if (tmp >= REQUEST_QUEUE_DEPTH)
        tmp = 0;
    return queue_wp == tmp;
}

static bool queue_is_empty() {
    return queue_wp == queue_rp;
}

static void inqueue_request(int request, usb_rf_cmd_t content) {
    EMI_Request_t r = {.req = request, .content = content};
    queue[queue_wp++] = r;
    if (queue_wp >= REQUEST_QUEUE_DEPTH) queue_wp = 0;
}

static EMI_Request_t dequeue_request() {
    EMI_Request_t r = queue[queue_rp++];
    if (queue_rp >= REQUEST_QUEUE_DEPTH) queue_rp = 0;
    return r;
}

static EMI_Request_t queue_top() {
    return queue[queue_rp];
}



uint32_t TIMER1_IrqHandler(void *user_data)
{
    TMR_IntClr(APB_TMR1, 0, 0xf);
    if(emi_sweep_timer_cb)
    {
        emi_sweep_timer_cb();
    }
    return 0;
}

void EMI_Timer_Init(void) {
    platform_set_irq_callback(PLATFORM_CB_IRQ_TIMER1, TIMER1_IrqHandler, NULL);
    SYSCTRL_ClearClkGateMulti(1 << SYSCTRL_ITEM_APB_TMR1);
    TMR_SetOpMode(APB_TMR1, 0, TMR_CTL_OP_MODE_32BIT_TIMER_x1, TMR_CLK_MODE_EXTERNAL, 0);
    TMR_IntEnable(APB_TMR1, 0, 0xf);
}

void EMI_Timer_Start(uint8_t sec, emi_sweep_timer_cb_t cb) {
    if(cb == NULL)
    {
        return;
    }
    emi_sweep_timer_cb = cb;
    uint32_t interval = TMR_GetClk(APB_TMR1, 0) * sec;
    TMR_SetReload(APB_TMR1, 0, interval);
    TMR_PauseEnable(APB_TMR1, 0);
    TMR_Enable(APB_TMR1, 0, 0xf);
}

void EMI_Timer_Stop(void) {
    TMR_PauseEnable(APB_TMR1, 1);
}

static void setPrevState(EMI_State_e state) {
    g_state_prev = state;
}
static void setState(EMI_State_e state) {
    g_state = state;
}


static int8_t convert_tx_power(uint8_t v) {
    switch(v) {
    case TX_POWER_0DB    : return 0;
    case TX_POWER_2DB    : return 2;
    case TX_POWER_3DB    : return 3;
    case TX_POWER_4DB    : return 4;
    case TX_POWER_5DB    : return 5;
    case TX_POWER_6DB    : return 6;
    case TX_POWER_NAG2DB : return -2;
    case TX_POWER_NAG4DB : return -4;
    case TX_POWER_NAG8DB : return -8;
    case TX_POWER_NAG12DB: return -12;
    case TX_POWER_NAG16DB: return -16;
    case TX_POWER_NAG10DB: return -10;
    case TX_POWER_NAG20DB: return -20;
    case TX_POWER_NAG25DB: return -25;
    case TX_POWER_NAG63DB: return -63;
    default:
        LOG_DEBUG("[Error]: unexpected tx_power: %d", v);
        return 0;
    }
}


static int convert_phy(uint8_t v) {
    switch (v) {
    case EMI_PHY_1M:
        return 0;
    case EMI_PHY_2M:
        return 1;
    case EMI_PHY_CODED_S8:
        return 2;
    case EMI_PHY_CODED_S2:
        return 3;
    default:
        LOG_DEBUG("[Error]: unexpected phy: %d", v);
        return 0;
    }
}

static int convert_payload_model(uint8_t v) {
    switch (v) {
    case WAVE_PRBS9:
        return 0;
    case WAVE_REPEAT10101010:
        return 1;
    case WAVE_REPEAT11110000:
        return 2;
    default:
        LOG_DEBUG("[Error]: unexpected payload_mode: %d", v);
        return 0;
    }
}

static uint8_t find_tx_power_index_cw(uint8_t tx_power) {
    switch (tx_power) {
    case TX_POWER_0DB    : return 22;
    case TX_POWER_2DB    : return 29;
    case TX_POWER_3DB    : return 24;
    case TX_POWER_4DB    : return 40;
    case TX_POWER_5DB    : return 49;
    case TX_POWER_6DB    : return 63;
    case TX_POWER_NAG2DB : return 17;
    case TX_POWER_NAG4DB : return 14;
    case TX_POWER_NAG8DB : return 8;
    case TX_POWER_NAG12DB: return 5;
    case TX_POWER_NAG16DB: return 3;
    case TX_POWER_NAG10DB: return 6;
    case TX_POWER_NAG20DB: return 2;
    case TX_POWER_NAG25DB: return 1;
    case TX_POWER_NAG63DB: return 0;
    default              : return 22;
    }
}

static uint8_t find_tx_power_index_rf_test(uint8_t tx_power) {
    switch (tx_power) {
    case TX_POWER_0DB    : return 39;
    case TX_POWER_2DB    : return 45;
    case TX_POWER_3DB    : return 50;
    case TX_POWER_4DB    : return 55;
    case TX_POWER_5DB    : return 60;
    case TX_POWER_6DB    : return 63;
    case TX_POWER_NAG2DB : return 30;
    case TX_POWER_NAG4DB : return 22;
    case TX_POWER_NAG8DB : return 14;
    case TX_POWER_NAG12DB: return 8;
    case TX_POWER_NAG16DB: return 5;
    case TX_POWER_NAG10DB: return 11;
    case TX_POWER_NAG20DB: return 3;
    case TX_POWER_NAG25DB: return 2;
    case TX_POWER_NAG63DB: return 0;
    default              : return 22;
    }
}

static uint8_t buffer[32];

static uint8_t calc_sum_8(uint8_t *data, uint16_t len) {
    uint8_t sum;
    while (len-- > 0)
        sum += *data++;
    return sum;
}

static void bsp_usb_send_success()
{
    LOG_INFO("success!\n");
    memset(buffer, 0, 32);
    buffer[0] = UCM_HEADER;
    buffer[1] = UCM_EMI_CODE;
    buffer[2] = UCM_CODE_SUCCESS;
    *(uint16_t *)(buffer + 3) = rx_received_packet_num;
    buffer[31] = calc_sum_8(buffer, 31);
    bsp_usb_hid_ucm_send(buffer, 32);
}

static void bsp_usb_send_failed()
{
    LOG_INFO("failed!\n");
    memset(buffer, 0, 32);
    buffer[0] = UCM_HEADER;
    buffer[1] = UCM_EMI_CODE;
    buffer[2] = UCM_CODE_SUCCESS;
    buffer[31] = calc_sum_8(buffer, 31);
    bsp_usb_hid_ucm_send(buffer, 32);
}

static bool sweep_carrier_task()
{
    usb_rf_cmd_t _ = {0};
    inqueue_request(SWEEP_CARRIER_FORWARD, _);
    LOG_DEBUG("[EMI]Sweep carrier forward.\n");
    
    return true;
}

static bool sweep_tx_v4_task()
{
    usb_rf_cmd_t _ = {0};
    inqueue_request(SWEEP_TX_V4_FORWARD, _);
    LOG_DEBUG("[EMI]Sweep tx_v4 forward.\n");
    
    return true;
}

static bool custom_sweep_tx_v4_task()
{
    usb_rf_cmd_t _ = {0};
    inqueue_request(CUS_SWEEP_BLE_TX_V4_FORWARD, _);
    LOG_DEBUG("[EMI]Custom sweep tx_v4 forward.\n");
    
    return true;
}

static bool custom_sweep_24g_tx_v4_task()
{
    usb_rf_cmd_t _ = {0};
    inqueue_request(CUS_SWEEP_24G_TX_V4_FORWARD, _);
    LOG_DEBUG("[EMI]Custom sweep 24G tx_v4 forward.\n");
    
    return true;
}

static bool custom_sweep_24g_carrier_task()
{
    usb_rf_cmd_t _ = {0};
    inqueue_request(CUS_SWEEP_24G_CARRIER_FORWARD, _);
    LOG_DEBUG("[EMI]Custom sweep 24G carrier forward.\n");
    
    return true;
}


static bool start_tx_v4(uint8_t tx_channel, uint8_t test_data_length,
                uint8_t packet_payload, uint8_t phy,
                int8_t tx_power_level)
{
    LOG_DEBUG("[EMI]Start tx_v4, channel : %d.\n", tx_channel);

    RFTestTx(tx_channel, phy, test_data_length, packet_payload, tx_power_level);

    // if (gap_tx_test_v4(tx_channel, test_data_length, 
    //         packet_payload, phy, 0, 0, 0, NULL, tx_power_level)){
    //     return false;
    // }
    return true;
}
static bool start_cw(uint8_t power_level_index, uint16_t freq)
{
    LOG_DEBUG("[EMI]Start carrier.\n");
    if (gap_vendor_tx_continuous_wave(1, power_level_index, freq)) {
        return false;
    }
    return true;
}
static bool start_rx_v2(uint8_t rx_channel, uint8_t phy)
{
    LOG_DEBUG("[EMI]Start rx_v2.\n");
    if (gap_rx_test_v2(rx_channel, phy, 0))
        return false;
    return true;
}
static bool start_sweep_tx_v4(uint8_t tx_channel_min, uint8_t tx_channel_max, 
                uint8_t sweep_time, uint8_t test_data_length,
                uint8_t packet_payload, uint8_t phy,
                int8_t tx_power_level)
{
    LOG_DEBUG("[EMI]Start sweep tx_v4.\n");
    memset(&sweep_tx_v4_task_ctx, 0, sizeof(sweep_tx_v4_task_ctx));
    sweep_tx_v4_task_ctx.channel_min = tx_channel_min;
    sweep_tx_v4_task_ctx.channel_max = tx_channel_max;
    sweep_tx_v4_task_ctx.channel_current = tx_channel_min;
    sweep_tx_v4_task_ctx.test_data_length = test_data_length;
    sweep_tx_v4_task_ctx.packet_payload = packet_payload;
    sweep_tx_v4_task_ctx.phy = phy;
    sweep_tx_v4_task_ctx.tx_power_level = tx_power_level;
    
    EMI_Timer_Start(sweep_time, sweep_tx_v4_task);
    g_sweep_state = STATE_SWEEP;
    
    return true;
}
static bool start_sweep_carrier(uint16_t tx_freq_min, uint16_t tx_freq_max, 
                uint8_t sweep_time, int8_t tx_power_level)
{
    LOG_DEBUG("[EMI]Start sweep carrier.\n");
    memset(&sweep_carrier_task_ctx, 0, sizeof(sweep_carrier_task_ctx));
    sweep_carrier_task_ctx.freq_min = tx_freq_min;
    sweep_carrier_task_ctx.freq_max = tx_freq_max;
    sweep_carrier_task_ctx.freq_current = tx_freq_min;
    sweep_carrier_task_ctx.tx_power_level = tx_power_level;
    
    EMI_Timer_Start(sweep_time, sweep_carrier_task);
    g_sweep_state = STATE_SWEEP;
    
    return true;
}
static bool start_custom_sweep_tx_v4(uint8_t sweep_time, uint8_t test_data_length,
                uint8_t packet_payload, uint8_t phy,
                int8_t tx_power_level)
{
    LOG_DEBUG("[EMI]Start other sweep BLE tx_v4.\n");
    memset(&custom_sweep_task_ctx, 0, sizeof(custom_sweep_task_ctx));
    custom_sweep_task_ctx.channel_current = 0;
    custom_sweep_task_ctx.test_data_length = test_data_length;
    custom_sweep_task_ctx.packet_payload = packet_payload;
    custom_sweep_task_ctx.phy = phy;
    custom_sweep_task_ctx.tx_power_level = tx_power_level;
    custom_sweep_task_ctx.step = 0;
    
    EMI_Timer_Start(sweep_time, custom_sweep_tx_v4_task);
    g_sweep_state = STATE_SWEEP;
    
    return true;
}
static bool start_custom_sweep_24g_tx_v4(uint8_t sweep_time, uint8_t test_data_length,
                uint8_t packet_payload, uint8_t phy,
                int8_t tx_power_level)
{
    LOG_DEBUG("[EMI]Start other sweep BLE tx_v4.\n");
    memset(&custom_sweep_task_ctx, 0, sizeof(custom_sweep_task_ctx));
    custom_sweep_task_ctx.channel_current = 2;
    custom_sweep_task_ctx.test_data_length = test_data_length;
    custom_sweep_task_ctx.packet_payload = packet_payload;
    custom_sweep_task_ctx.phy = phy;
    custom_sweep_task_ctx.tx_power_level = tx_power_level;
    custom_sweep_task_ctx.step = 0;
    
    EMI_Timer_Start(sweep_time, custom_sweep_24g_tx_v4_task);
    g_sweep_state = STATE_SWEEP;
    
    return true;
}
static bool start_custom_sweep_24g_carrier(uint8_t sweep_time, int8_t tx_power_level)
{
    LOG_DEBUG("[EMI]Start other sweep BLE tx_v4.\n");
    memset(&custom_sweep_task_ctx, 0, sizeof(custom_sweep_task_ctx));
    custom_sweep_task_ctx.channel_current = 2402;
    custom_sweep_task_ctx.tx_power_level = tx_power_level;
    
    EMI_Timer_Start(sweep_time, custom_sweep_24g_carrier_task);
    g_sweep_state = STATE_SWEEP;
    
    return true;
}

/**
 * @brief Checks and stops the ongoing HCI test if needed.
 * 
 * @return true if the test is successfully stopped or if there's no ongoing test,
 *         false if there's an error during test termination.
 */
static bool emi_test_stop_hci_test()
{
    if (running_task == TASK_TYPE_CW ||
        running_task == TASK_TYPE_SWEEP_CW || running_task == TASK_TYPE_CUS_SWEEP_24G_CW) {
        if (gap_vendor_tx_continuous_wave(0, 0, 0)) {
            return false;
        }
    } 
    if(running_task == TASK_TYPE_CUS_SWEEP_24G_TX_V4) {
        LLE_TX_STOP();
    }
    else {
        if (gap_test_end()) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Starts a test according to the specified test mode.
 * 
 * This function starts a test based on the provided test mode. It switches between different test modes 
 * and performs the corresponding actions accordingly. Each test mode has its own set of parameters 
 * required for starting the test.
 *
 * For START_TX_SWEEP_PAYLOAD and START_TX_SWEEP_CARRIER, they only initiate timers and do not require
 * waiting for HCI command complete events.
 * 
 * @return true if the test was successfully started, false otherwise.
 */
static bool start_test_dif_case()
{
    LOG_DEBUG("[EMI]Test mode: 0x%02X.\n", g_req);
    switch (g_req)
    {
        case START_TX_SINGLE_CARRIER:
        {
            if (start_cw(find_tx_power_index_cw(g_cmd.tx_power), 
                        FREQ_TO_CW(g_cmd.mode_data.single.channel))) {
                return true;
            }
            break;
        }
        case START_TX_SINGLE_PAYLOAD:
        {
            // if (start_tx_v4(FREQ_TO_TEST4(g_cmd.mode_data.single.channel), 
            //             g_cmd.payload_len + 1, 
            //             convert_payload_model(g_cmd.payload_model), 
            //             g_cmd.phy + 1,
            //             convert_tx_power(g_cmd.tx_power))) {
            //     return true;
            // }
            if (start_tx_v4(g_cmd.mode_data.single.channel, 
                        g_cmd.payload_len + 1, 
                        convert_payload_model(g_cmd.payload_model), 
                        g_cmd.phy + 1,
                        find_tx_power_index_rf_test(g_cmd.tx_power))) {
                return true;
            }
            break;
        }
        case START_TX_SWEEP_PAYLOAD:
        {
            if (start_sweep_tx_v4(
                        FREQ_TO_TEST4(g_cmd.mode_data.sweep.channel_min), 
                        FREQ_TO_TEST4(g_cmd.mode_data.sweep.channel_max), 
                        g_cmd.mode_data.sweep.sweep_time + 1,
                        g_cmd.payload_len + 1, 
                        convert_payload_model(g_cmd.payload_model), 
                        g_cmd.phy + 1,
                        convert_tx_power(g_cmd.tx_power))) {
                return true;
            }
            break;
        }
        case START_TX_SWEEP_CARRIER:
        {
            if (start_sweep_carrier(
                        FREQ_TO_CW(g_cmd.mode_data.sweep.channel_min), 
                        FREQ_TO_CW(g_cmd.mode_data.sweep.channel_max), 
                        g_cmd.mode_data.sweep.sweep_time + 1,
                        find_tx_power_index_cw(g_cmd.tx_power))) {
                return true;
            }
            break;
        }
        case START_TX_SWEEP_PAYLOAD_V2:
        {
            if(start_custom_sweep_tx_v4(
                        g_cmd.mode_data.custom_sweep.sweep_time + 1,
                        g_cmd.payload_len + 1, 
                        convert_payload_model(g_cmd.payload_model), 
                        g_cmd.phy + 1,
                        convert_tx_power(g_cmd.tx_power)))
            {
                return true;
            }
            break;
        }
        case START_TX_SWEEP_24G_PAYLOAD:
        {
            if(start_custom_sweep_24g_tx_v4(
                        g_cmd.mode_data.custom_sweep.sweep_time + 1,
                        g_cmd.payload_len + 1, 
                        convert_payload_model(g_cmd.payload_model), 
                        g_cmd.phy + 1,
                        find_tx_power_index_rf_test(g_cmd.tx_power)))
            {
                return true;
            }
            break;
        }
        case START_TX_SWEEP_24G_CARRIER:
        {
            if(start_custom_sweep_24g_carrier(g_cmd.mode_data.custom_sweep.sweep_time + 1, find_tx_power_index_cw(g_cmd.tx_power)))
            {
                return true;
            }
            break;
        }
        case START_RX:
        {
            if (start_rx_v2(FREQ_TO_TEST4(g_cmd.mode_data.single.channel), g_cmd.phy + 1)) {
                return true;
            }
            break;
        }
        case SWEEP_TX_V4_FORWARD:
        {
            LOG_DEBUG("[EMI]Current Freq :%dMHz\n", 2400 + (sweep_tx_v4_task_ctx.channel_current + 1) * 2);
            
            if (!gap_tx_test_v4(sweep_tx_v4_task_ctx.channel_current, 
                        sweep_tx_v4_task_ctx.test_data_length, 
                        sweep_tx_v4_task_ctx.packet_payload,
                        sweep_tx_v4_task_ctx.phy, 0, 0, 0, NULL, 
                        sweep_tx_v4_task_ctx.tx_power_level)) {
                return true;
            }
            break;
        }
        case SWEEP_CARRIER_FORWARD:
        {
            LOG_DEBUG("[EMI]Current Freq :%dMHz\n", sweep_carrier_task_ctx.freq_current);
            
            if (!gap_vendor_tx_continuous_wave(1, 
                        sweep_carrier_task_ctx.tx_power_level, 
                        sweep_carrier_task_ctx.freq_current)) {
                return true;
            }
            break;
        }

        case CUS_SWEEP_BLE_TX_V4_FORWARD:
        {
            LOG_DEBUG("[EMI]Current Freq :%dMHz\n", 2400 + (custom_sweep_task_ctx.channel_current + 1) * 2);
            if (!gap_tx_test_v4(custom_sweep_task_ctx.channel_current, 
                        custom_sweep_task_ctx.test_data_length, 
                        custom_sweep_task_ctx.packet_payload,
                        custom_sweep_task_ctx.phy, 0, 0, 0, NULL, 
                        custom_sweep_task_ctx.tx_power_level)) {
                return true;
            }
            break;
        }
        case CUS_SWEEP_24G_TX_V4_FORWARD:
        {
            LOG_DEBUG("[EMI]Current Freq :%dMHz\n", 2400 + custom_sweep_task_ctx.channel_current);
            RFTestTx(custom_sweep_task_ctx.channel_current, 
                        custom_sweep_task_ctx.phy, 
                        custom_sweep_task_ctx.test_data_length, 
                        custom_sweep_task_ctx.packet_payload, 
                        custom_sweep_task_ctx.tx_power_level);
            return true;
            break;
        }
        case CUS_SWEEP_24G_CARRIER_FORWARD:
        {
            LOG_DEBUG("[EMI]Current Freq :%dMHz\n", custom_sweep_task_ctx.channel_current);
            
            if (!gap_vendor_tx_continuous_wave(1, 
                        custom_sweep_task_ctx.tx_power_level, 
                        custom_sweep_task_ctx.channel_current)) {
                return true;
            }
            break;
        }
        
    }
    return false;
}

/**
 * @brief Executes an iteration of the test case based on the current test mode.
 * 
 * This function performs an iteration of the test case based on the current test mode. It increments
 * the test parameters according to the specific requirements of the test mode. For SWEEP_TX_V4_FORWARD
 * and SWEEP_CARRIER_FORWARD, it increments the channel or frequency values respectively and resets
 * them if they exceed the maximum values.
 * 
 * @return true if the iteration was successfully completed, false otherwise.
 */
static bool iteration_dif_case()
{
    switch (g_req)
    {
        case SWEEP_TX_V4_FORWARD:
        {
            sweep_tx_v4_task_ctx.channel_current += 1;
            if (sweep_tx_v4_task_ctx.channel_current > sweep_tx_v4_task_ctx.channel_max) {
                sweep_tx_v4_task_ctx.channel_current = sweep_tx_v4_task_ctx.channel_min;
            }
            return true;
        }
        case SWEEP_CARRIER_FORWARD:
        {
            sweep_carrier_task_ctx.freq_current += 2;
            if (sweep_carrier_task_ctx.freq_current > sweep_carrier_task_ctx.freq_max) {
                sweep_carrier_task_ctx.freq_current = sweep_carrier_task_ctx.freq_min;
            }
            return true;
        }
        case CUS_SWEEP_BLE_TX_V4_FORWARD:
        {
            custom_sweep_task_ctx.channel_current += 2;
            if(custom_sweep_task_ctx.channel_current > 0x27) //  > 2480
            {
                if(custom_sweep_task_ctx.step == 0)
                {
                    custom_sweep_task_ctx.channel_current = 1;
                }
                else if (custom_sweep_task_ctx.step == 1)
                {
                    custom_sweep_task_ctx.channel_current = 0;
                }
                custom_sweep_task_ctx.step++;
                custom_sweep_task_ctx.step = custom_sweep_task_ctx.step % 2;
            }
            return true;
        }
        case CUS_SWEEP_24G_TX_V4_FORWARD:
        {
            custom_sweep_task_ctx.channel_current += 4;
            if(custom_sweep_task_ctx.channel_current > 80) // > 2480
            {
                if(custom_sweep_task_ctx.step == 0) custom_sweep_task_ctx.channel_current = 3; // 2403
                if(custom_sweep_task_ctx.step == 1) custom_sweep_task_ctx.channel_current = 4; // 2404
                if(custom_sweep_task_ctx.step == 2) custom_sweep_task_ctx.channel_current = 5; // 2405
                if(custom_sweep_task_ctx.step == 3) custom_sweep_task_ctx.channel_current = 2; // 2402
                custom_sweep_task_ctx.step++;
                custom_sweep_task_ctx.step = custom_sweep_task_ctx.step % 4;
            }
            return true;
        }
        case CUS_SWEEP_24G_CARRIER_FORWARD:
        {
            custom_sweep_task_ctx.channel_current++;
            if(custom_sweep_task_ctx.channel_current > 2480)
            {
                custom_sweep_task_ctx.channel_current = 2402;
            }
            return true;
        }
        default:
        {
            EMI_Unexpected_Occurence();
            break;
        }
    }
    return false;
}

bool emi_state_busy()
{
    return g_sweep_state == STATE_SWEEP || g_state != STATE_IDLE;
}

bool emi_has_running_test_or_task()
{
    return g_sweep_state == STATE_SWEEP || g_state == STATE_IN_PROGRESS;
}

/**
 * @brief Stops a currently running test.
 * 
 * This function is responsible for stopping a currently running test based on the current state
 * of the system. It is typically called within the main loop of a bare-metal environment to handle 
 * test execution flow. Depending on the current state, it performs different actions:
 * 
 * - If the state is STATE_IDLE, it logs that the test is idle.
 * - If the state is STATE_IN_PROGRESS, it attempts to stop the test using emi_test_stop_hci_test().
 *   If successful, it sets a timeout for the test start and transitions to STATE_SHUTDOWN, waiting for 
 *   the HCI command complete event.
 * - If the state is STATE_SHUTDOWN, it waits for the timeout or the HCI command complete event. If the 
 *   command is completed, it transitions to STATE_IDLE; otherwise, it logs a stop test timeout and 
 *   returns to STATE_IN_PROGRESS.
 * - If the state is STATE_STARTUP, it logs an unexpected occurrence.
 * 
 * @return true if the function completes an action indicating that the test is finished, 
 *         false otherwise, indicating that the function needs to be executed again.
 */
static bool emi_stop_a_test()
{
    switch((int)g_state) 
    {
        case STATE_IDLE:
        {
            LOG_DEBUG("[EMI]Test idle .\n");
            return true;
        }
        case STATE_IN_PROGRESS:
        {
            if (emi_test_stop_hci_test()) {
                emi_test_set_timeout(EMI_TEST_START_TIMEOUT_US);
                command_completed = false;
                setState(STATE_SHUTDOWN);
                LOG_DEBUG("[EMI]Wait HCI commnad_complete...\n");
            } else {
                setState(STATE_IN_PROGRESS);
                return true;
            }
            break;
        }
        case STATE_SHUTDOWN:
        {
            if(running_task == TASK_TYPE_CUS_SWEEP_24G_TX_V4)
            {
                command_completed = true;   //没有返回值，直接true
            }
    
            if (emi_test_wait_timeout(command_completed)) {
                if (command_completed) {
                    LOG_DEBUG("[EMI]Get HCI command complete OK.\n");
                    setState(STATE_IDLE);
                } else {
                    LOG_DEBUG("[EMI]Stop Test timeout .\n");
                    setState(STATE_IN_PROGRESS);
                }
                return true;
            }
            break;
        }
        case STATE_STARTUP:
        {
            EMI_Unexpected_Occurence();
            break;
        }
    }
    return false;
}


/**
 * @brief Starts a test based on the current state of the system.
 * 
 * This function is responsible for starting a test based on the current state of the system.
 * It is typically called within the main loop of a bare-metal environment to handle test 
 * execution flow. Depending on the current state, it performs different actions:
 * 
 * - If the state is STATE_IN_PROGRESS, it logs that the test is already in progress.
 * - If the state is STATE_IDLE, it attempts to start a test using start_test_dif_case().
 *   If successful, it sets a timeout for the test start and transitions to STATE_STARTUP,
 *   waiting for the HCI command complete event.
 * - If the state is STATE_STARTUP, it waits for the timeout or the HCI command complete event. 
 *   If the command is completed, it transitions to STATE_IN_PROGRESS; otherwise, it logs a 
 *   start test timeout and returns to STATE_IDLE.
 * - If the state is STATE_SHUTDOWN, it logs an unexpected occurrence.
 * 
 * @return true if the function completes an action indicating that the test is started or in progress, 
 *         false otherwise, indicating that the function needs to be executed again.
 */
static bool emi_star_a_test()
{
    switch((int)g_state)
    {
        case STATE_IN_PROGRESS:
        {
            LOG_DEBUG("[EMI]Test in progress .\n");
            return true;
        }
        case STATE_IDLE:
        {
            if (start_test_dif_case()) {
                emi_test_set_timeout(EMI_TEST_START_TIMEOUT_US);
                command_completed = false;
                setState(STATE_STARTUP);
                LOG_DEBUG("[EMI]Wait HCI commnad_complete...\n");
            } else {
                setState(STATE_IDLE);
                return true;
            }
            break;
        }
        case STATE_STARTUP:
        {
            if(running_task == TASK_TYPE_CUS_SWEEP_24G_TX_V4 || running_task == TASK_TYPE_TX_V4)
            {
                command_completed = true;   //没有返回值，直接true
            }

            if (emi_test_wait_timeout(command_completed)) {
                if (command_completed) {
                    LOG_DEBUG("[EMI]Get HCI command complete OK.\n");
                    setState(STATE_IN_PROGRESS);
                } else {
                    LOG_DEBUG("[EMI]Start Test timeout .\n");
                    setState(STATE_IDLE);
                }
                return true;
            }
            break;
        }
        case STATE_SHUTDOWN:
        {
            EMI_Unexpected_Occurence();
            break;
        }
    }
    return false;
}

/**
 * @brief Triggers the process of stopping a test.
 * 
 * This function sets the state to initiate the stopping process of a test.
 */
static void emi_trigger_stop_process()
{
    g_stop_process_state = STATE_STOP_BEGIN;
}

/**
 * @brief Triggers the process of starting a test.
 * 
 * This function sets the state to initiate the starting process of a test.
 */
static void emi_trigger_start_process()
{
    g_start_process_state = STATE_BEGIN;
}

/**
 * @brief Triggers the forwarding process for sweep tasks.
 * 
 * This function sets the state to initiate the forwarding process.
 */
static void emi_trigger_forward_process()
{
    g_forward_process_state = STATE_FORWARD_BEGIN;
}

/**
 * @brief Handles the process of stopping a test.
 * 
 * This function manages the process of stopping a test. It is typically called within a control loop
 * to handle the stopping procedure. Depending on the current state of the stopping process, it performs
 * different actions:
 * 
 * - If the state is STATE_STOP_BEGIN, it checks if the sweep is in normal state or running. If running,
 *   it logs the stop of the running test and transitions to STATE_STOP_STOPING_RUNNING_TEST; otherwise, 
 *   it stops the sweep timer and transitions to STATE_STOP_SWEEP.
 * - If the state is STATE_STOP_SWEEP, it resets the sweep state to normal and transitions to 
 *   STATE_STOP_STOPING_RUNNING_TEST.
 * - If the state is STATE_STOP_STOPING_RUNNING_TEST, it attempts to stop the running test using emi_stop_a_test().
 *   If successful and the system is idle, it logs the successful test stop and transitions to STATE_STOP_END;
 *   otherwise, it logs a failed test stop and returns to STATE_STOP_BEGIN.
 * - If the state is STATE_STOP_END, it logs the end of the test stopping process.
 * 
 * @return true if the function completes an action indicating that the test stopping process is finished, 
 *         false otherwise, indicating that the function needs to be executed again.
 */
static bool emi_stop_process()
{
    switch((int)g_stop_process_state)
    {
        case STATE_STOP_BEGIN:
        {
            if (g_sweep_state == STATE_NORMAL) {
                LOG_DEBUG("[EMI]Stop running Test.\n");
                g_stop_process_state = STATE_STOP_STOPING_RUNNING_TEST;
            } else {
                LOG_DEBUG("[EMI]Stop sweep timer.\n");
                g_stop_process_state = STATE_STOP_SWEEP;
            }
            break;
        }
        case STATE_STOP_SWEEP:
        {
            g_sweep_state = STATE_NORMAL;
            g_stop_process_state = STATE_STOP_STOPING_RUNNING_TEST;
        }
        case STATE_STOP_STOPING_RUNNING_TEST:
        {
            if (emi_stop_a_test()) {
                if (g_state == STATE_IDLE) {
                    LOG_DEBUG("[EMI]Stop Test OK .\n");
                    g_stop_process_state = STATE_STOP_END;
                    return true;
                } else {
                    LOG_DEBUG("[EMI]Stop Test Failed.\n");
                    g_stop_process_state = STATE_STOP_BEGIN;
                    return true;
                }
            }
            break;
        }
        case STATE_STOP_END:
        {
            LOG_DEBUG("[EMI]Stop Test End .\n");
            return true;
        }
    }
    return false;
}

/**
 * @brief Manages the process of starting a test.
 * 
 * This function handles the process of starting a test based on the current state of the system.
 * It is typically called within a control loop to manage the test starting procedure.
 * Depending on the current state of the starting process, it performs different actions:
 * 
 * - If the state is STATE_BEGIN, it checks if the system is busy with another task using emi_state_busy().
 *   If busy, it triggers the stop process and transitions to STATE_STOPING; otherwise, it directly starts
 *   the test or sweep timer based on the test request and transitions to the corresponding states.
 * - If the state is STATE_STOPING, it executes the stop process using emi_stop_process(). If successful and
 *   the stopping process is completed, it starts the test or sweep timer and transitions to the corresponding states;
 *   otherwise, it logs a start process failure and returns to STATE_BEGIN.
 * - If the state is STATE_STARTING, it executes the start process using emi_star_a_test(). If successful and
 *   a running test or task is detected, it logs a successful start and transitions to STATE_END; otherwise,
 *   it logs a start test failure and returns to STATE_BEGIN.
 * - If the state is STATE_START_SWEEP_TIMER, it directly starts the test or sweep timer based on the test request.
 *   If successful, it logs a successful start and transitions to STATE_END; otherwise, it logs an unexpected occurrence.
 * - If the state is STATE_END, it logs the end of the start test process.
 * 
 * @return true if the function completes an action indicating that the test starting process is finished, 
 *         false otherwise, indicating that the function needs to be executed again.
 */
static bool emi_start_process()
{
    switch((int)g_start_process_state)
    {
        case STATE_BEGIN:
        {
            if (emi_state_busy()) {
                LOG_DEBUG("[EMI]Trigger Stop Process.\n");
                emi_trigger_stop_process();
                g_start_process_state = STATE_STOPING;
            } else {
                if (g_req == START_TX_SWEEP_PAYLOAD || g_req == START_TX_SWEEP_CARRIER || 
                    g_req == START_TX_SWEEP_PAYLOAD_V2 || g_req == START_TX_SWEEP_24G_PAYLOAD || g_req == START_TX_SWEEP_24G_CARRIER) {
                    LOG_DEBUG("[EMI]Directly start sweep timer.\n");
                    g_start_process_state = STATE_START_SWEEP_TIMER;
                } else {
                    LOG_DEBUG("[EMI]Directly start Test.\n");
                    g_start_process_state = STATE_STARTING;
                }
            }
            break;
        }
        case STATE_STOPING:
        {
            if (emi_stop_process()) {
                if (g_stop_process_state == STATE_STOP_END) {
                    if (g_req == START_TX_SWEEP_PAYLOAD || g_req == START_TX_SWEEP_CARRIER || 
                        g_req == START_TX_SWEEP_PAYLOAD_V2 || g_req == START_TX_SWEEP_24G_PAYLOAD || g_req == START_TX_SWEEP_24G_CARRIER) {
                        LOG_DEBUG("[EMI]Start sweep timer.\n");
                        g_start_process_state = STATE_START_SWEEP_TIMER;
                    } else {
                        LOG_DEBUG("[EMI]Start Test.\n");
                        g_start_process_state = STATE_STARTING;
                    }
                } else {
                    LOG_DEBUG("[EMI]Start process Failed.\n");
                    g_start_process_state = STATE_BEGIN;
                    return true;
                }
            }
            break;
        }
        case STATE_STARTING:
        {
            if (emi_star_a_test()) {
                if (emi_has_running_test_or_task()) {
                    LOG_DEBUG("[EMI]Start Test OK .\n");
                    g_start_process_state = STATE_END;
                } else {
                    LOG_DEBUG("[EMI]Start Test Failed .\n");
                    g_start_process_state = STATE_BEGIN;
                }
                return true;
            }
            break;
        }
        case STATE_START_SWEEP_TIMER:
        {
            /* 
             * START_TX_SWEEP_PAYLOAD || START_TX_SWEEP_CARRIER 
             */
            if (start_test_dif_case()) {
                LOG_DEBUG("[EMI]Start Test OK .\n");
                g_start_process_state = STATE_END;
            } else {
                EMI_Unexpected_Occurence();
            }
            return true;
        }
        case STATE_END:
        {
            LOG_DEBUG("[EMI]Start Test End .\n");
            return true;
        }
    }
    return false;
}

/**
 * @brief Manages the process of forwarding.
 * 
 * This function handles the process of forwarding based on the current state of the system.
 * It is typically called within a control loop to manage the forwarding procedure.
 * Depending on the current state of the forwarding process, it performs different actions:
 * 
 * - If the state is STATE_FORWARD_BEGIN, it checks if the system is not idle. If not idle,
 *   it triggers the stop process and transitions to STATE_FORWARD_STOPING; otherwise, it directly
 *   starts the forward process and transitions to STATE_FORWARD_STARTING.
 * - If the state is STATE_FORWARD_STOPING, it executes the stop process using emi_stop_a_test().
 *   If successful and the system becomes idle, it logs a successful stop and transitions to STATE_FORWARD_STARTING;
 *   otherwise, it logs a failed stop and returns to STATE_FORWARD_BEGIN.
 * - If the state is STATE_FORWARD_STARTING, it executes the start process using emi_star_a_test().
 *   If successful and a running test or task is detected, it logs a successful start and transitions to STATE_FORWARD_ITERATION;
 *   otherwise, it logs a failed start and returns to STATE_FORWARD_BEGIN.
 * - If the state is STATE_FORWARD_ITERATION, it performs an iteration of the test case using iteration_dif_case(),
 *   logs the forward iteration, and transitions to STATE_FORWARD_END.
 * - If the state is STATE_FORWARD_END, it logs the end of the forward test process.
 * 
 * @return true if the function completes an action indicating that the forward process is finished, 
 *         false otherwise, indicating that the function needs to be executed again.
 */
static bool emi_forward_process()
{
    switch((int)g_forward_process_state)
    {
        case STATE_FORWARD_BEGIN:
        {
            if (g_state != STATE_IDLE) {
                LOG_DEBUG("[EMI]Trigger Stop Process.\n");
                emi_trigger_stop_process();
                g_forward_process_state = STATE_FORWARD_STOPING;
            } else {
                LOG_DEBUG("[EMI]Forward directly start.\n");
                g_forward_process_state = STATE_FORWARD_STARTING;
            }
            break;
        }
        case STATE_FORWARD_STOPING:
        {
            if (emi_stop_a_test()) {
                if (g_state == STATE_IDLE) {
                    LOG_DEBUG("[EMI]Forward stop Test OK .\n");
                    g_forward_process_state = STATE_FORWARD_STARTING;
                } else {
                    LOG_DEBUG("[EMI]Forward stop Test Failed.\n");
                    g_forward_process_state = STATE_FORWARD_BEGIN;
                    return true;
                }
            }
            break;
        }
        case STATE_FORWARD_STARTING:
        {
            if (emi_star_a_test()) {
                if (emi_has_running_test_or_task()) {
                    LOG_DEBUG("[EMI]Forward start Test OK .\n");
                    g_forward_process_state = STATE_FORWARD_ITERATION;
                } else {
                    LOG_DEBUG("[EMI]Forward start Test Failed .\n");
                    g_forward_process_state = STATE_FORWARD_BEGIN;
                    return true;
                }
            }
            break;
        }
        case STATE_FORWARD_ITERATION:
        {
            LOG_DEBUG("[EMI]Forward iteration.\n");
            iteration_dif_case();
            g_forward_process_state = STATE_FORWARD_END;
            return true;
        }
        case STATE_FORWARD_END:
        {
            LOG_DEBUG("[EMI]Forward Test End .\n");
            return true;
        }
    }
    return false;
}

void update_running_task_flag()
{
    switch(g_req) {
    case START_TX_SINGLE_CARRIER:
        running_task = TASK_TYPE_CW;
        break;
    case START_TX_SINGLE_PAYLOAD:
        running_task = TASK_TYPE_TX_V4;
        break;
    case START_TX_SWEEP_CARRIER:
        running_task = TASK_TYPE_SWEEP_CW;
        break;
    case START_TX_SWEEP_PAYLOAD:
        running_task = TASK_TYPE_SWEEP_TX_V4;
        break;
    case START_RX:
        running_task = TASK_TYPE_RX_V2;
        break;
    case STOP_TEST:
        running_task = TASK_TYPE_NONE;
        break;
    case START_TX_SWEEP_PAYLOAD_V2:
        running_task = TASK_TYPE_CUS_SWEEP_TX_V4;
        break;
    case START_TX_SWEEP_24G_PAYLOAD:
        running_task = TASK_TYPE_CUS_SWEEP_24G_TX_V4;
        break;
    case START_TX_SWEEP_24G_CARRIER:
        running_task = TASK_TYPE_CUS_SWEEP_24G_CW;
        break;
    default:
        EMI_Unexpected_Occurence();
        break;
    }
}

/**
 * @brief Executes the EMI loop for handling requests and processes.
 * 
 * This function manages the EMI loop, which involves processing requests 
 * queued in a FIFO queue. It checks the current state request and performs 
 * corresponding actions based on the request type. If the request is to 
 * stop a test, it triggers the stop process. If it's to forward a sweep 
 * or carrier, it triggers the forward process. Otherwise, it initiates 
 * the start process. After processing each request, it updates the state 
 * request accordingly.
 * 
 * @return 0 indicating successful execution.
 */
uint8_t emi_loop(void)
{
#define SLEEP_ENABLE 0
#define SLEEP_DISABLE 1
    
    if (!g_active_flag)
        return SLEEP_ENABLE;
    
    current_time = platform_get_us_time();
    
    switch((int)g_state_request)
    {
        case STATE_REQUEST_IDLE:
        {
            if (queue_is_empty())
                break;
            
            EMI_Request_t request = dequeue_request();
            g_req = request.req;
            g_cmd = request.content;
            g_state_request = STATE_REQUEST_IN_PROGRESS;
            LOG_DEBUG("[EMI]Pick a request.%d\n", g_req);
            if (g_req == STOP_TEST) {
                LOG_DEBUG("[EMI]Trigger Stop Process.\n");
                emi_trigger_stop_process();
            } 
            else if (g_req == SWEEP_TX_V4_FORWARD || g_req == SWEEP_CARRIER_FORWARD || g_req == CUS_SWEEP_BLE_TX_V4_FORWARD ||
                                                     g_req == CUS_SWEEP_24G_TX_V4_FORWARD || g_req == CUS_SWEEP_24G_CARRIER_FORWARD)
            {
                LOG_DEBUG("[EMI]Trigger Forward Process.\n");
                emi_trigger_forward_process();
            } else {
                LOG_DEBUG("[EMI]Trigger Start Process.\n");
                emi_trigger_start_process();
                update_running_task_flag();
            }
            break;
        }
        case STATE_REQUEST_IN_PROGRESS:
        {
            if (g_req == STOP_TEST) {
                if (emi_stop_process()) {
                    if (g_stop_process_state == STATE_STOP_END) {
                        update_running_task_flag();
                        EMI_Timer_Stop();
                        bsp_usb_send_success();
                    } else {
                        bsp_usb_send_failed();
                    }
                    g_state_request = STATE_REQUEST_IDLE;
                }
            } 
            else if (g_req == SWEEP_TX_V4_FORWARD || g_req == SWEEP_CARRIER_FORWARD || g_req == CUS_SWEEP_BLE_TX_V4_FORWARD ||
                                                     g_req == CUS_SWEEP_24G_TX_V4_FORWARD || g_req == CUS_SWEEP_24G_CARRIER_FORWARD) 
            {
                if (emi_forward_process()) {
                    if (g_forward_process_state == STATE_FORWARD_END) {
                        bsp_usb_send_success();
                    } else {
                        bsp_usb_send_failed();
                    }
                    g_state_request = STATE_REQUEST_IDLE;
                }
            } else {
                if (emi_start_process()) {
                    if (g_start_process_state == STATE_END) {
                        update_running_task_flag();
                        bsp_usb_send_success();
                    } else {
                        bsp_usb_send_failed();
                    }
                    g_state_request = STATE_REQUEST_IDLE;
                }
            }
            break;
        }
    }
    
    return g_state == STATE_IDLE ? SLEEP_ENABLE : SLEEP_DISABLE;
}

#define EMI_RX_ENABLE 0

EMI_Result_t EMI_Dispatch(uint8_t *data, uint16_t len){
    
    uint32_t i;
    LOG_INFO("data: ");for(i=0;i<len;i++){LOG_INFO(" 0x%x ",data[i]);}LOG_INFO("\n");
    
    uint8_t sum = data[31]; // check sum [0, 31)
    
    g_cmd = *(usb_rf_cmd_t *)data;
    
    if (g_cmd.is_start) {
        if (g_cmd.is_rx) {
#if(EMI_RX_ENABLE == 1)
            LOG_INFO("Start RX Test====\n");
            LOG_INFO("channel       : %02X\n", g_cmd.mode_data.single.channel);
            LOG_INFO("phy           : %02X\n", g_cmd.phy);
            inqueue_request(START_RX, g_cmd);
#else
            LOG_INFO("Start RX to Stop Test====\n");
            inqueue_request(STOP_TEST, g_cmd);
#endif
        } else {
            if (g_cmd.mode == RF_TEST_MODE_SINGLE) {
                if (g_cmd.payload_model == WAVE_CARRIER) {
                    LOG_INFO("Start TX Test (Single)(Carrier)====\n");
                    LOG_INFO("channel       : %02X\n", g_cmd.mode_data.single.channel);
                    LOG_INFO("tx_power      : %02X\n", g_cmd.tx_power);
                    LOG_INFO("phy           : %02X\n", g_cmd.phy);
    
                    inqueue_request(START_TX_SINGLE_CARRIER, g_cmd);
                } else {
                    LOG_INFO("Start TX Test (Single)(Payload)====\n");
                    LOG_INFO("channel       : %02X\n", g_cmd.mode_data.single.channel);
                    LOG_INFO("payload_model : %02X\n", g_cmd.payload_model);
                    LOG_INFO("payload_len   : %02X\n", g_cmd.payload_len);
                    LOG_INFO("tx_power      : %02X\n", g_cmd.tx_power);
                    LOG_INFO("phy           : %02X\n", g_cmd.phy);
    
                    inqueue_request(START_TX_SINGLE_PAYLOAD, g_cmd);
                }
            } else if (g_cmd.mode == RF_TEST_MODE_SWEEP) {
                if (g_cmd.payload_model == WAVE_CARRIER) {
                    LOG_INFO("Start TX Test (Sweep)(Carrier)====\n");
                    LOG_INFO("channel_min   : %02X\n", g_cmd.mode_data.sweep.channel_min);
                    LOG_INFO("channel_max   : %02X\n", g_cmd.mode_data.sweep.channel_max);
                    LOG_INFO("sweep_time    : %02X\n", g_cmd.mode_data.sweep.sweep_time);
                    LOG_INFO("tx_power      : %02X\n", g_cmd.tx_power);
                    LOG_INFO("phy           : %02X\n", g_cmd.phy);
    
                    inqueue_request(START_TX_SWEEP_CARRIER, g_cmd);
                } else {
                    LOG_INFO("Start TX Test (Sweep)(Payload)====\n");
                    LOG_INFO("channel_min   : %02X\n", g_cmd.mode_data.sweep.channel_min);
                    LOG_INFO("channel_max   : %02X\n", g_cmd.mode_data.sweep.channel_max);
                    LOG_INFO("sweep_time    : %02X\n", g_cmd.mode_data.sweep.sweep_time);
                    LOG_INFO("payload_model : %02X\n", g_cmd.payload_model);
                    LOG_INFO("payload_len   : %02X\n", g_cmd.payload_len);
                    LOG_INFO("tx_power      : %02X\n", g_cmd.tx_power);
                    LOG_INFO("phy           : %02X\n", g_cmd.phy);
    
                    inqueue_request(START_TX_SWEEP_PAYLOAD, g_cmd);
                }
            } else if(g_cmd.mode == RF_TEST_MODE_OTHER_SWEEP)
            {
                if(g_cmd.mode_data.custom_sweep.type == EMI_CUS_SWEEP_BLE)
                {
                    if(g_cmd.payload_model == WAVE_CARRIER) g_cmd.payload_model = WAVE_PRBS9;
                    LOG_INFO("Start TX Test (Custom sweep BLE)(Payload)====\n");
                    LOG_INFO("sweep_time    : %02X\n", g_cmd.mode_data.sweep.sweep_time);
                    LOG_INFO("payload_model : %02X\n", g_cmd.payload_model);
                    LOG_INFO("payload_len   : %02X\n", g_cmd.payload_len);
                    LOG_INFO("tx_power      : %02X\n", g_cmd.tx_power);
                    LOG_INFO("phy           : %02X\n", g_cmd.phy);
    
                    inqueue_request(START_TX_SWEEP_PAYLOAD_V2, g_cmd);
                }
                else
                {
                    if (g_cmd.payload_model == WAVE_CARRIER)
                    {
                        LOG_INFO("Start TX Test (Custom sweep 24g)(Carrier)====\n");
                        LOG_INFO("sweep_time    : %02X\n", g_cmd.mode_data.sweep.sweep_time);
                        LOG_INFO("payload_model : %02X\n", g_cmd.payload_model);
                        LOG_INFO("payload_len   : %02X\n", g_cmd.payload_len);
                        LOG_INFO("tx_power      : %02X\n", g_cmd.tx_power);
                        LOG_INFO("phy           : %02X\n", g_cmd.phy);
        
                        inqueue_request(START_TX_SWEEP_24G_CARRIER, g_cmd);
                    }
                    else
                    {
                        LOG_INFO("Start TX Test (Custom sweep 24g)(Payload)====\n");
                        LOG_INFO("sweep_time    : %02X\n", g_cmd.mode_data.sweep.sweep_time);
                        LOG_INFO("payload_model : %02X\n", g_cmd.payload_model);
                        LOG_INFO("payload_len   : %02X\n", g_cmd.payload_len);
                        LOG_INFO("tx_power      : %02X\n", g_cmd.tx_power);
                        LOG_INFO("phy           : %02X\n", g_cmd.phy);
        
                        inqueue_request(START_TX_SWEEP_24G_PAYLOAD, g_cmd);
                    }
                }
            }
        }
    } else {
        // stop
        LOG_INFO("Stop Test====\n");
        
        inqueue_request(STOP_TEST, g_cmd);
    }
    
    return EMI_RESULT_SUCCESS;
}

static void EMI_on_hci_cmd_complete_start_test(uint16_t opcode, const uint8_t *packet, uint16_t size)
{
    command_completed = true;
}


static void EMI_on_hci_cmd_complete_stop_test(uint16_t opcode, const uint8_t *packet, uint16_t size)
{
    if (running_task == TASK_TYPE_RX_V2) {
        const uint8_t *param = hci_event_command_complete_get_return_parameters(packet);
        LOG_DEBUG("%02X %02X %02X\n", param[0], param[1], param[2]);
        rx_received_packet_num = (param[2] << 8) | (param[1] << 0);
    }
    command_completed = true;
}


static hci_cmd_cmpl_user_handler_entry_t emi_cmd_cmpl_handler_map[] = {
    {HCI_LE_Receiver_Test_v2    , EMI_on_hci_cmd_complete_start_test},
    {HCI_LE_Transmitter_Test_v2 , EMI_on_hci_cmd_complete_start_test},
    {HCI_LE_Transmitter_Test_v4 , EMI_on_hci_cmd_complete_start_test},
    {HCI_LE_Vendor_CW_Test      , EMI_on_hci_cmd_complete_start_test},
    {HCI_LE_Test_End            , EMI_on_hci_cmd_complete_stop_test},
};
static uint16_t emi_cmd_cmpl_handler_map_size = sizeof(emi_cmd_cmpl_handler_map) / sizeof(hci_cmd_cmpl_user_handler_entry_t);

uint8_t emi_init(void)
{
    for (uint16_t i = 0; i < emi_cmd_cmpl_handler_map_size; ++i) {
        hci_command_complete_user_handler_register(emi_cmd_cmpl_handler_map[i].opcode, emi_cmd_cmpl_handler_map[i].handler);
    }
    
    memset(&g_cmd, 0, sizeof(usb_rf_cmd_t));
    memset(&sweep_carrier_task_ctx, 0, sizeof(sweep_carrier_task_context_t));
    memset(&sweep_tx_v4_task_ctx, 0, sizeof(sweep_tx_v4_task_context_t)); 
    
    g_active_flag = false;
    g_sweep_state = STATE_NORMAL;
    g_state = STATE_IDLE;
    g_state_request = STATE_REQUEST_IDLE;
    g_stop_process_state = STATE_STOP_BEGIN;
    g_start_process_state = STATE_BEGIN;
    g_forward_process_state = STATE_FORWARD_BEGIN;
    running_task = TASK_TYPE_NONE;
    
    EMI_Timer_Init();
    return 0;
}

void emi_enable(bool enable) {
    g_active_flag = enable;
}
