#include <stdio.h>
#include "platform_api.h"
#include "att_db.h"
#include "gap.h"
#include "btstack_event.h"
#include "btstack_defines.h"
#include "bluetooth_hci.h"
#include "profile.h"
#include "IAP_916.h"
#include "ctrl.h"

#define INVALIDE_HANDLE     0xffff

// ================================================================================

/* 处理 EMI 需要用到的 hci_event_command_complete 回调 */
hci_cmd_cmpl_user_handler_entry_t hci_cmd_complete_user_handler_map[HCI_CMD_CMPL_USER_HANDLER_MAP_MAX_SIZE] = {0};
uint16_t hci_cmd_cmpl_user_handler_map_len = 0;

uint8_t hci_command_complete_user_handler_register(uint16_t opcode, hci_cmd_cmpl_user_handler_cb_t handler) {
    if (hci_cmd_cmpl_user_handler_map_len >= HCI_CMD_CMPL_USER_HANDLER_MAP_MAX_SIZE) 
        return 1;
    hci_cmd_cmpl_user_handler_entry_t entry = {.opcode = opcode, .handler = handler};
    hci_cmd_complete_user_handler_map[hci_cmd_cmpl_user_handler_map_len++] = entry;
    return 0;
}

void hci_cmd_cmpl_user_handler(const uint8_t *packet, uint16_t size) {
    
    uint16_t opcode = hci_event_command_complete_get_command_opcode(packet);
    for (uint8_t i = 0; i < hci_cmd_cmpl_user_handler_map_len; ++i) {
        if (hci_cmd_complete_user_handler_map[i].opcode == opcode &&
            hci_cmd_complete_user_handler_map[i].handler != NULL) {
            hci_cmd_complete_user_handler_map[i].handler(opcode, packet, size);
        }
    }
}

// ================================================================================
        
static uint16_t att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset,
                                  uint8_t * buffer, uint16_t buffer_size)
{
    switch (att_handle)
    {

    default:
        return 0;
    }
}

static int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode,
                              uint16_t offset, const uint8_t *buffer, uint16_t buffer_size)
{
    switch (att_handle)
    {

    default:
        return 0;
    }
}

static void user_msg_handler(uint32_t msg_id, void *data, uint16_t size)
{
    switch (msg_id)
    {
    default:
        break;
    }
}

/*初始化上电也会调用此函数初始化2.4G*/
static void user_packet_handler(uint8_t packet_type, uint16_t channel, const uint8_t *packet, uint16_t size)
{
    
    uint8_t event = hci_event_packet_get_type(packet);
    const btstack_user_msg_t *p_user_msg;
    if (packet_type != HCI_EVENT_PACKET) return;

    switch (event)
    {
    case BTSTACK_EVENT_STATE:
        if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING)
            break;
        
        ucm_emi_enable(true);   // 默认使能EMI
        break;

    case HCI_EVENT_COMMAND_COMPLETE:
        switch (hci_event_command_complete_get_command_opcode(packet))
        {
            default:
                hci_cmd_cmpl_user_handler(packet, size);
                break;
        }
        break;

    case HCI_EVENT_LE_META:
        switch (hci_event_le_meta_get_subevent_code(packet))
        {
        case HCI_SUBEVENT_LE_ENHANCED_CONNECTION_COMPLETE:
            break;
        case HCI_SUBEVENT_LE_ADVERTISING_SET_TERMINATED:
            break;
        default:
            break;
        }

        break;

    case HCI_EVENT_DISCONNECTION_COMPLETE:
        break;

    case ATT_EVENT_CAN_SEND_NOW:
        // add your code
        break;

    case BTSTACK_EVENT_USER_MSG:
        p_user_msg = hci_event_packet_get_user_msg(packet);
        user_msg_handler(p_user_msg->msg_id, p_user_msg->data, p_user_msg->len);
        break;

    default:
        break;
    }
}


static btstack_packet_callback_registration_t hci_event_callback_registration;

uint32_t setup_profile(void *data, void *user_data)
{
    log_printf("setup profile\n");
    
    /*初始化升级使用到的一些变量、以及USB功能*/
    IAP_Init();
    
    ucm_init();
    
    // Note: security has not been enabled.
    att_server_init(att_read_callback, att_write_callback);
    hci_event_callback_registration.callback = &user_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);
    att_server_register_packet_handler(&user_packet_handler);
    return 0;
}
