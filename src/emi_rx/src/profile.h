#ifndef _PROFILESTASK_H_
#define _PROFILESTASK_H_

#include <stdint.h>

uint32_t setup_profile(void *data, void *user_data);

#if (RELEASE_VER == 1)
    #define log_printf(...)
#else
    #define log_printf(...) platform_printf(__VA_ARGS__)
#endif

#endif


#define HCI_Reset                   0x0c03
#define HCI_LE_Receiver_Test_v1     0x201d
#define HCI_LE_Receiver_Test_v2     0x2033
#define HCI_LE_Transmitter_Test_v1  0x201E
#define HCI_LE_Transmitter_Test_v2  0x2034
#define HCI_LE_Transmitter_Test_v4  0x207b
#define HCI_LE_Test_End             0x201f

#define HCI_LE_Vendor_CW_Test       0xfc02

#define HCI_CMD_CMPL_USER_HANDLER_MAP_MAX_SIZE 10

/**
 * @brief Type definition for user-defined callback function to handle HCI command complete events.
 * 
 * This type definition defines the signature of a user-defined callback function that is used to handle
 * HCI command complete events.
 * 
 * @param opcode The opcode of the HCI command for which the command complete event is received.
 * @param param The parameter data associated with the command complete event.
 * @param param_len The length of the parameter data.
 */
typedef void (*hci_cmd_cmpl_user_handler_cb_t) (uint16_t, const uint8_t *, uint16_t);

typedef struct {
    uint16_t opcode;
    hci_cmd_cmpl_user_handler_cb_t handler;
} hci_cmd_cmpl_user_handler_entry_t;

/**
 * @brief Registers a user-defined handler function to handle HCI command complete events.
 * 
 * This function allows the user to register a callback function to handle HCI command complete events
 * for a specific HCI command opcode.
 * 
 * @param opcode The opcode of the HCI command for which the handler is being registered.
 * @param handler The user-defined callback function to be called when the corresponding HCI command complete event is received.
 */
uint8_t hci_command_complete_user_handler_register(uint16_t opcode, hci_cmd_cmpl_user_handler_cb_t handler);
