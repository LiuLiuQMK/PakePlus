/* $(license) */

/* 
 * 
 */
 
#include "ctrl.h"
#include "ctrl_emi.h"
#include "bsp_usb_hid_iap.h"

#define LOG_ERR platform_printf
#define LOG_INFO platform_printf
#define LOG_DEBUG platform_printf

#define UCM_COMMAND_MODIFY_USB_INFO 0x80

#define PACKAEG_BUFFER_SIZE 300

#define FIRST_PACK_MASK (1 << 15)
#define GET_NUMBER(x) ((x) & ~FIRST_PACK_MASK)

typedef enum {
    UCM_RESULT_SUCCESS = 0,
    UCM_RESULT_CHECKSUM_MISMATCH,
    UCM_RESULT_UNEXPECTED_PACKAGE,
    UCM_RESULT_BUFFER_OVERFLOW,
    UCM_RESULT_UNEXPECTED_COMMAND,
} UCM_Result_t;

typedef enum {
    MODIFY_USB_INFO_SUCCESS = 0,
    MODIFY_USB_INFO_UNEXPECT_ERROR,
} UCM_Command_Modify_USB_Info_Result_t;


typedef enum {
    PACKAGING_IDLE,
    PACKAGING_IN_PROGRESS,
} UCM_Packaging_State_e;

typedef struct {
    UCM_Packaging_State_e state;
    uint8_t buffer[PACKAEG_BUFFER_SIZE];
    uint8_t pos;
    uint16_t number; 
} UCM_Packaging_Ctrl_t;

#pragma pack (push, 1)
typedef struct {
    uint8_t header;
    uint8_t packetCode;
    uint16_t number;
    uint8_t size;
    uint8_t payload[26];
    uint8_t checksum;
} UCM_Packet_t;

typedef struct {
    uint8_t cmd;
    uint8_t checksum;
    uint16_t vid;
    uint16_t pid;
    uint8_t nameLength;
    uint8_t nameData[0];
} UCM_Command_Modify_USB_Info_t;

typedef struct {
    uint8_t header;
    uint8_t packetCode;
    uint8_t returnCode;
    uint8_t payload[28];
    uint8_t checksum;
} UCM_Response_t;
#pragma pack (pop)


static uint8_t buffer[32];
//static UCM_Packet_t packet;
static UCM_Packaging_Ctrl_t package;
static bool is_command_process = false;

static uint8_t calc_sum_8(uint8_t *data, int len) {
    uint8_t sum = 0;
    while (len-- > 0)
        sum += *data++;
    return sum;
}

static void bsp_usb_ucm_normal_send_result(UCM_Result_t r)
{
    //LOG_DEBUG("[UCM] Send result: %d.\n", r);
    UCM_Response_t *resp = (UCM_Response_t *)buffer;
    resp->header = UCM_HEADER;
    resp->packetCode = UCM_PACKET_CODE,
    resp->returnCode = r;
    memset(resp->payload, 0, 28);
    resp->checksum = calc_sum_8(buffer, 31);
    bsp_usb_hid_ucm_send(buffer, 32);
}

static void send_modify_usb_info_result(UCM_Command_Modify_USB_Info_Result_t r) {
    LOG_DEBUG("[UCM Command] Send modify USB info response: %d.\n", r);
    UCM_Response_t *resp = (UCM_Response_t *)buffer;
    resp->header = UCM_HEADER;
    resp->packetCode = UCM_PACKET_CODE,
    resp->returnCode = UCM_RESULT_SUCCESS;
    memset(resp->payload, 0, 28);
    resp->payload[0] = r;
    resp->checksum = calc_sum_8(buffer, 31);
    bsp_usb_hid_ucm_send(buffer, 32);
}

static void UCM_Packaging_Set_Idle()
{
    package.pos = 0;
    package.number = 0;
    package.state = PACKAGING_IDLE;
}

static UCM_Result_t UCM_Command_Dispatch_Modify_USB_Info(UCM_Command_Modify_USB_Info_t * command)
{
    LOG_INFO("[UCM Command] Modify USB Info:\n");
    LOG_INFO("VID: 0x%04X\n", command->vid);
    LOG_INFO("PID: 0x%04X\n", command->pid);
    LOG_INFO("NameLength: %d\n", command->nameLength);
    command->nameData[command->nameLength] = '\0';
    LOG_INFO("Name:%s\n", command->nameData);
    
    // Modify USB info
    
    send_modify_usb_info_result(MODIFY_USB_INFO_SUCCESS);
    return UCM_RESULT_SUCCESS;
}

static UCM_Result_t UCM_Command_Dispatch() {
    switch (package.buffer[0]) {
        case UCM_COMMAND_MODIFY_USB_INFO: { // Modify USB Info
            return UCM_Command_Dispatch_Modify_USB_Info((UCM_Command_Modify_USB_Info_t *)package.buffer);
        }
        default: {
            return UCM_RESULT_UNEXPECTED_COMMAND;
        }
    }
}

static UCM_Result_t UCM_Packaging_Dispatch(uint8_t* buff, uint16_t len) {
    
    if (buff[31] != calc_sum_8(buff, 31)) {
        LOG_ERR("[UCM] CRC Error: %02X, %02X.\n", buff[31], calc_sum_8(buff, 31));
        return UCM_RESULT_CHECKSUM_MISMATCH;
    }
    UCM_Packet_t *data = (UCM_Packet_t *)buff;
    
    switch ((int)package.state) {
        case PACKAGING_IDLE: {
            /* First Packet, startup packaging progress */
            if ((data->number & FIRST_PACK_MASK) == FIRST_PACK_MASK) {
                /* packaging... */
                package.pos = 0;
                memcpy(package.buffer + package.pos, buff + 5, data->size);
                package.number = GET_NUMBER(data->number);
                package.pos += data->size;
                package.state = PACKAGING_IN_PROGRESS;
            } else {
                LOG_ERR("[UCM] Not first packet.\n");
                return UCM_RESULT_UNEXPECTED_PACKAGE;
            }
            break;
        }
        case PACKAGING_IN_PROGRESS: {
            /* The PACKAGING_IN_PROGRESS status should not have received the first packet */
            if ((data->number & FIRST_PACK_MASK) == FIRST_PACK_MASK) {
                LOG_ERR("[UCM] Repeat first packet.\n");
                UCM_Packaging_Set_Idle();
                return UCM_RESULT_UNEXPECTED_PACKAGE;
            }
            
            /* Check packet sequence number */
            if (package.number != GET_NUMBER(data->number) + 1) {
                LOG_ERR("[UCM] Error packet sequnence %d -> %d.\n", package.number, GET_NUMBER(data->number));
                UCM_Packaging_Set_Idle();
                return UCM_RESULT_UNEXPECTED_PACKAGE;
            }
            
            /* Check buffer overflow */
            if (package.pos + data->size >= PACKAEG_BUFFER_SIZE) {
                LOG_ERR("[UCM] Buffer overflow.\n");
                UCM_Packaging_Set_Idle();
                return UCM_RESULT_BUFFER_OVERFLOW;
            }
            
            /* Save data */
            memcpy(package.buffer + package.pos, buff + 5, data->size);
            package.number = GET_NUMBER(data->number);
            package.pos += data->size;
            break;
        }
    }
    
    /* Last packet */
    if (package.number == 0) {
        is_command_process = true;
        UCM_Result_t r = UCM_Command_Dispatch();
        UCM_Packaging_Set_Idle();
    }
    return UCM_RESULT_SUCCESS;
}

static UCM_Result_t UCM_Dispatch(uint8_t *data, uint16_t len) {
    is_command_process = false;
    
    UCM_Result_t r = UCM_Packaging_Dispatch(data, len);
    
    if (!is_command_process) {
        bsp_usb_ucm_normal_send_result(r);
    }
    return r;
}

// recv callback.
static void bsp_usb_hid_ucm_recv_callback(uint8_t *data, uint16_t len){
    
    if (data[0] == UCM_HEADER) {
        if (data[1] == UCM_EMI_CODE) {  // EMI
            EMI_Dispatch(data, len);
        } else if (data[1] == UCM_PACKET_CODE) {    // UCM Command
            UCM_Dispatch(data, len);
        } else {
            LOG_ERR("Error packet: 0x%02X", data[3]);
        }
    } else {
        LOG_ERR("Error header: 0x%02X", data[0]);
    }
}

uint8_t ucm_init(void)
{
    emi_init();
    bsp_usb_hid_ucm_recv_callback_register(bsp_usb_hid_ucm_recv_callback);
    
    return 0;
}

uint8_t ucm_loop(void)
{
    uint8_t r = 0;
    r |= emi_loop();
    return r;
}
 
void ucm_emi_enable(bool enable)
{
    emi_enable(enable);
}