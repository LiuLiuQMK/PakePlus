#include <stdio.h>
#include "platform_api.h"
#include "att_db.h"
#include "gap.h"
#include "att_dispatch.h"
#include "btstack_util.h"
#include "btstack_event.h"
#include "btstack_defines.h"
#include "gatt_client.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include "ad_parser.h"
#include "gatt_client_util.h"
#include "sm.h"
#include "string.h"
#include "profile.h"
#include "bsp_usb_hid_iap.h"


// GATT characteristic handles
#include "../data/gatt.const"

const static uint8_t adv_data[] = {
    #include "../data/advertising.adv"
};

#include "../data/advertising.const"

const static uint8_t scan_data[] = {
    #include "../data/scan_response.adv"
};

#include "../data/scan_response.const"

const static uint8_t profile_data[] = {
    #include "../data/gatt.profile"
};

static uint8_t Ble_State = KB_BLE_STATE_SCAN;

#define INVALID_HANDLE  0xffff
struct gatt_client_discoverer *discoverer = NULL;
uint16_t mas_conn_handle = INVALID_HANDLE;

#define USER_MSG_NOTIFY_ENABLE              1

//static SemaphoreHandle_t sem_led_ctl = NULL;

sm_persistent_t sm_persistent =
{
    .er = {1, 2, 3},
    .ir = {4, 5, 6},
    .identity_addr_type     = BD_ADDR_TYPE_LE_RANDOM,
    .identity_addr          = {0xC6, 0xFA, 0x5C, 0x20, 0x87, 0xA7}
};

static btstack_packet_callback_registration_t hci_event_callback_registration;

uint8_t ble_state_get(void)
{
    return Ble_State;
}
void btstack_callback(uint8_t packet_type, uint16_t channel, const uint8_t *packet, uint16_t size)
{
    switch (packet[0])
    {
    case GATT_EVENT_QUERY_COMPLETE:
        if (gatt_event_query_complete_parse(packet)->status != 0)
            return;
        log_printf("cmpl");
        break;
    }
}

static const scan_phy_config_t configs[2] =
{
    {
        .phy = PHY_1M,
        .type = SCAN_PASSIVE,
        .interval = 200,
        .window = 50
    },
    {
        .phy = PHY_CODED,
        .type = SCAN_PASSIVE,
        .interval = 200,
        .window = 50
    }
};


static uint8_t target_adv_data[31] = {
    // 0x01 - «Flags»
    2, 0x01,
    0x06, 

    // 0x03 - «Complete List of 16-bit Service Class UUIDs»
    3, 0x03,
    0x12, 0x18, 

    // 0x19 - «Appearance»
    3, 0x19,
    0xC1, 0x03, 

    // 0x09 - «Complete Local Name»
    7, 0x09,
    87, 97, 118, 101, 55, 53,

    // Total size = 18 bytes
};


typedef struct slave_info
{
    uint32_t    s2m_total;
    uint32_t    m2s_total;
    gatt_client_service_t                   service_tpt;
    gatt_client_characteristic_t            basic_char;
    gatt_client_characteristic_t            extend_char;
    gatt_client_characteristic_descriptor_t basic_desc;
    gatt_client_characteristic_descriptor_t extend_desc;
    gatt_client_notification_t              output_basic_notify;
    gatt_client_notification_t              output_extend_notify;
    uint16_t    conn_handle;
} slave_info_t;

slave_info_t slave = {.conn_handle = INVALID_HANDLE};
static uint16_t char_config_notification = GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;


static void user_msg_handler(uint32_t msg_id, void *data, uint16_t size)
{
    switch (msg_id)
    {
        // add your code
    case USER_MSG_NOTIFY_ENABLE:
        gatt_client_write_characteristic_descriptor_using_descriptor_handle(btstack_callback, slave.conn_handle,
            slave.basic_desc.handle, sizeof(char_config_notification),
            (uint8_t *)&char_config_notification);
        gatt_client_write_characteristic_descriptor_using_descriptor_handle(btstack_callback, slave.conn_handle,
            slave.extend_desc.handle, sizeof(char_config_notification),
            (uint8_t *)&char_config_notification);
        break;
    default:
        ;
    }
}

uint32_t get_sig_short_uuid(const uint8_t *uuid128)
{
    return uuid_has_bluetooth_prefix(uuid128) ? big_endian_read_32(uuid128, 0) : 0;
}

static void basic_output_notification_handler(uint8_t packet_type, uint16_t _, const uint8_t *packet, uint16_t size)
{
    const gatt_event_value_packet_t *value_packet;
    uint16_t value_size;
    switch (packet[0])
    {
    case GATT_EVENT_NOTIFICATION:
        value_packet = gatt_event_notification_parse(packet, size, &value_size);
        memcpy(KeybReport.Key_Byte_Table, value_packet->value, value_size);
        if(USB_HID_ERROR_NONE != bsp_usb_hid_keyboard_basic_report_start()) {
			    log_printf("[USB]:basic send fail");
        }
        break;
    }
}

static void extend_output_notification_handler(uint8_t packet_type, uint16_t _, const uint8_t *packet, uint16_t size)
{
    const gatt_event_value_packet_t *value_packet;
    uint16_t value_size;
    switch (packet[0])
    {
    case GATT_EVENT_NOTIFICATION:
        value_packet = gatt_event_notification_parse(packet, size, &value_size);
        if(USB_HID_ERROR_NONE != bsp_usb_hid_report_send(KB_REPORT_ID, (uint8_t *)value_packet->value, KEY_BIT_TABLE_LEN)){
			    log_printf("[USB]:extend send fail");
        }
        break;
    }
}

static void output_notify_char_init(void)
{
    slave.basic_char.value_handle = 29;
    slave.extend_char.value_handle = 45;
    slave.basic_desc.handle = 30;
    slave.extend_desc.handle = 46;
}
void service_discovery_callback(uint8_t packet_type, uint16_t _, const uint8_t *packet, uint16_t size)
{
    switch (packet[0])
    {
    case GATT_EVENT_SERVICE_QUERY_RESULT:
        {
            slave.service_tpt = gatt_event_service_query_result_parse(packet)->service;
            log_printf("service handle: %d %d",slave.service_tpt.start_group_handle, slave.service_tpt.end_group_handle);
        }
        break;
    case GATT_EVENT_QUERY_COMPLETE:
        if (gatt_event_query_complete_parse(packet)->status != 0)
            break;
        if (slave.service_tpt.start_group_handle != INVALID_HANDLE)
        {
            log_printf("service discover OK:%d\n", slave.conn_handle);
            output_notify_char_init();
            gatt_client_listen_for_characteristic_value_updates(&slave.output_basic_notify, basic_output_notification_handler,
                                                                slave.conn_handle, slave.basic_char.value_handle);
            gatt_client_listen_for_characteristic_value_updates(&slave.output_extend_notify, extend_output_notification_handler,
                                                                slave.conn_handle, slave.extend_char.value_handle);
            sm_request_pairing(slave.conn_handle);
            Ble_State = KB_BLE_STATE_COMMUNICATING;

        }
        else
        {
            log_printf("service not found, disc");
            gap_disconnect(slave.conn_handle);
        }
        break;
    }
}

uint8_t adv_finded(const uint8_t len, const uint8_t *data)
{
    if(len < ADV_CMP_LENGTH)
    {
        return 0;
    }

    if (memcmp(target_adv_data, data, ADV_CMP_LENGTH) == 0)
    {    
        return 1;
    }
    else
    {
        return 0;
    }
}

static initiating_phy_config_t phy_configs[] =
{
    {
        .phy = PHY_1M,
        .conn_param =
        {
            .scan_int = 200,
            .scan_win = 180,
            .interval_min = 50,
            .interval_max = 50,
            .latency = 0,
            .supervision_timeout = 200,
            .min_ce_len = 90,
            .max_ce_len = 90
        }
    }
};

bd_addr_t peer_addr;
static void user_packet_handler(uint8_t packet_type, uint16_t channel, const uint8_t *packet, uint16_t size)
{
    static const bd_addr_t rand_addr = { 0xC7, 0x45, 0xC1, 0xBE, 0x1D, 0x4F };
    uint8_t event = hci_event_packet_get_type(packet);
    const btstack_user_msg_t *p_user_msg;
    if (packet_type != HCI_EVENT_PACKET) return;

    switch (event)
    {
    case BTSTACK_EVENT_STATE:
        if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING)
            break;
        gap_set_random_device_address(rand_addr);
        gap_set_ext_scan_para(BD_ADDR_TYPE_LE_RANDOM, SCAN_ACCEPT_ALL_EXCEPT_NOT_DIRECTED,
                              sizeof(configs) / sizeof(configs[0]),
                              configs);
        gap_set_ext_scan_enable(1, 0, 0, 0);   // start continuous scanning
        break;

    case HCI_EVENT_LE_META:
        switch (hci_event_le_meta_get_subevent_code(packet))
        {
        case HCI_SUBEVENT_LE_EXTENDED_ADVERTISING_REPORT:
            {
                const le_ext_adv_report_t *report = decode_hci_le_meta_event(packet, le_meta_event_ext_adv_report_t)->reports;
                if(adv_finded(report->data_len, report->data))
                {
                    if(report->rssi < RSSI_THRESHOLD)
                    {
                        log_printf("RSSI low:%d\n", report->rssi);
                        return;
                    }
                    reverse_bd_addr(report->address, peer_addr);
                    gap_set_ext_scan_enable(0, 0, 0, 0);
                    Ble_State = KB_BLE_STATE_CONNECT;
                    gap_ext_create_connection(    INITIATING_ADVERTISER_FROM_PARAM, // Initiator_Filter_Policy,
                                                  BD_ADDR_TYPE_LE_RANDOM,           // Own_Address_Type,
                                                  report->addr_type,                // Peer_Address_Type,
                                                  peer_addr,                        // Peer_Address,
                                                  sizeof(phy_configs) / sizeof(phy_configs[0]),
                                                  phy_configs);
                }
            }
            break;
            case HCI_SUBEVENT_LE_ENHANCED_CONNECTION_COMPLETE:
            {
                const le_meta_event_enh_create_conn_complete_t *conn_complete
                     = decode_hci_le_meta_event(packet, le_meta_event_enh_create_conn_complete_t);
                if (conn_complete->status)
                    platform_reset();
                log_printf("connected\n");
                slave.conn_handle = conn_complete->handle;
                gap_read_remote_used_features(conn_complete->handle);
                gatt_client_discover_primary_services_by_uuid16(service_discovery_callback, conn_complete->handle, 0x1812);
            }
            break;
        default:
            break;
        }

        break;

    case HCI_EVENT_DISCONNECTION_COMPLETE:
    {
        slave.conn_handle = INVALID_HANDLE;
        gap_set_ext_scan_enable(1, 0, 0, 0);   // start continuous scanning
        log_printf("Disconn\n");
        Ble_State = KB_BLE_STATE_SCAN;
        TMR_SetReload(APB_TMR2, 0, 1000000);   //
        TMR_Enable(APB_TMR2, 0, 1);
        // add your code
        break;
    }
    case L2CAP_EVENT_CAN_SEND_NOW:
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

static void sm_packet_handler(uint8_t packet_type, uint16_t channel, const uint8_t *packet, uint16_t size)
{
    uint8_t event = hci_event_packet_get_type(packet);

    if (packet_type != HCI_EVENT_PACKET) return;
//    if (0 == bonding_flag) return;

//    log_printf("SM: %#x\n", event);
    switch (event)
    {
    case SM_EVENT_JUST_WORKS_REQUEST:
        log_printf("JUST_WORKS confirmed\n");
        sm_just_works_confirm(sm_event_just_works_request_get_handle(packet));
        break;
    case SM_EVENT_PASSKEY_DISPLAY_NUMBER:
        log_printf("===================\npasskey: %06d\n===================\n",
            sm_event_passkey_display_number_get_passkey(packet));
        break;
    case SM_EVENT_PASSKEY_DISPLAY_CANCEL:
        log_printf("TODO: DISPLAY_CANCEL\n");
        break;
    case SM_EVENT_PASSKEY_INPUT_NUMBER:
        // TODO: intput number
        log_printf("===================\ninput number:\n");
        break;
    case SM_EVENT_PASSKEY_INPUT_CANCEL:
        log_printf("TODO: INPUT_CANCEL\n");
        break;
    case SM_EVENT_IDENTITY_RESOLVING_FAILED:
        log_printf("not authourized\n");
        break;
    case SM_EVENT_IDENTITY_RESOLVING_SUCCEEDED:
        discoverer = gatt_client_util_discover_all(slave.conn_handle, gatt_client_util_dump_profile, NULL);
        break;
    case SM_EVENT_STATE_CHANGED:
        {
            const sm_event_state_changed_t *state_changed = decode_hci_event(packet, sm_event_state_changed_t);
            switch (state_changed->reason)
            {
                case SM_STARTED:
                    log_printf("[BLE]SM: STARTED\n");
                    break;
                case SM_FINAL_PAIRED:{
                        log_printf("[BLE]SM: PAIRED\n");
                        btstack_push_user_msg(USER_MSG_NOTIFY_ENABLE, NULL, 0);
                    }
                    break;
                case SM_FINAL_REESTABLISHED:
                    log_printf("[BLE]SM: REESTABLISHED\n");

                    break;
                case SM_FINAL_FAIL_TIMEOUT:
                    log_printf("[BLE]SM SM_FINAL_FAIL_TIMEOUT\n");
                    break;
                case SM_FINAL_FAIL_DISCONNECT:
                    log_printf("[BLE]SM FINAL ERROR: %d %d\n", state_changed->conn_handle, state_changed->reason);
                    break;

                case SM_FINAL_FAIL_PROTOCOL:
                    log_printf("[BLE]SM_FINAL_FAIL_PROTOCOL\n");
                    break;
                default:
                    log_printf("[BLE]SM: UNSUPPORT REASON: %d\n", state_changed->reason);
                    break;
            }
        }
        break;
    
    default:
        break;
    }
}


//static void init_tasks()
//{
//    sem_led_ctl = xSemaphoreCreateBinary();

//    xTaskCreate(heart_rate_task,
//               "h",
//               550,
//               NULL,
//               (configMAX_PRIORITIES - 1),
//               NULL);

//}

static btstack_packet_callback_registration_t sm_event_callback_registration  = {.callback = &sm_packet_handler};
uint32_t setup_profile(void *data, void *user_data)
{   
    log_printf("setup profile\n");
//    init_tasks();
//    att_server_init(att_read_callback, att_write_callback);
    hci_event_callback_registration.callback = user_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);
    att_server_register_packet_handler(user_packet_handler);
    gatt_client_register_handler(user_packet_handler);
    sm_add_event_handler(&sm_event_callback_registration);
    sm_config(1, IO_CAPABILITY_NO_INPUT_NO_OUTPUT,
              0,
              &sm_persistent);
//    sm_add_event_handler(&sm_event_callback_registration);
    sm_set_authentication_requirements(SM_AUTHREQ_BONDING);
    return 0;
}

