#ifndef _IAP_TRANSPORT_H_
#define _IAP_TRANSPORT_H_

#include <stdint.h>
#include "bsp_usb_hid_iap.h"


#define IAP_TRANSPORT_MAX_SEND_DATA_SIZE        (MAX_REPORT_SIZE)

// PARAM.
#define IAP_TRANSPORT_HEADER                    ((uint8_t)0xAA)
#define IAP_TRANSPORT_HEADER_DUMMY              ((uint8_t)0xAA)
#define IAP_TRANSPORT_MIN_LEN                   (7) //HEADER(1B) + CTL_1(1B) + PACK_CTL(2B) + LEN(2B) + BCC(1B)
#define IAP_TRANSPORT_MAX_SEND_SINGLE_PAYLOAD   (IAP_TRANSPORT_MAX_SEND_DATA_SIZE - IAP_TRANSPORT_MIN_LEN)

// CTL_1.
#define IAP_CTL_1_DIRECTION_MASK        (1<<7)
#define IAP_CTL_1_PACK_TYPE_MASK        (1<<6)
#define IS_DIRECTION_UP(x)              (((x)&IAP_CTL_1_DIRECTION_MASK)? 1 : 0)
#define IS_DIRECTION_DOWN(x)            (((x)&IAP_CTL_1_DIRECTION_MASK)? 0 : 1)
#define IS_PACK_TYPE_DATA(x)            (((x)&IAP_CTL_1_PACK_TYPE_MASK)? 0 : 1)
#define IS_PACK_TYPE_ACK(x)             (((x)&IAP_CTL_1_PACK_TYPE_MASK)? 1 : 0)

// PACK_CTL
#define IAP_PACK_CTL_FIRST_PACK_MASK    ((uint16_t)(1<<15))
#define IAP_PACK_CTL_PACK_NUM_MASK      ((uint16_t)(0x7FFF))
#define GET_PACK_NUM(x)                 ((uint16_t)(x)&IAP_PACK_CTL_PACK_NUM_MASK)
#define IS_FIRST_PACK(x)                (((uint16_t)(x)&IAP_PACK_CTL_FIRST_PACK_MASK)? 1 : 0)
#define IS_LAST_PACK(x)                 (GET_PACK_NUM(x) == 0x0000)


typedef enum{
    IAP_TRANSPORT_ERR_NONE,
    IAP_TRANSPORT_ERR_HEADER,
    IAP_TRANSPORT_ERR_DIRECTION,
    IAP_TRANSPORT_ERR_PACK_TYPE,
    IAP_TRANSPORT_ERR_PACK_NUM,
    IAP_TRANSPORT_ERR_LENGTH,
    IAP_TRANSPORT_ERR_BCC,
    IAP_TRANSPORT_ERR_BUFFER_OVERFLOW,
    IAP_TRANSPORT_ERR_NONE_REPLY,   // no error, no need to reply.
} IAP_TransportErr_t;

typedef enum{
    IAP_TRANSPORT_STA_IDLE,
    IAP_TRANSPORT_STA_RECVING,

} IAP_TransportState_t;



typedef struct __attribute__((packed)){
    uint8_t header;
    uint8_t ctl_1;
    uint16_t pack_ctl;
    uint16_t payload_length;
    uint8_t payload[1];
} IAP_Transport_Recv_t;

typedef struct __attribute__((packed)){
    uint8_t header;
    uint8_t ctl_1;
    uint16_t pack_ctl;
    uint16_t length;
    uint8_t errCode;
    uint8_t BCC;
} IAP_Transport_ACK_t;

typedef struct __attribute__((packed)){
    IAP_TransportState_t state;
    uint8_t total_pack_num;
    uint8_t next_pack_num;
} IAP_Transport_Ctl_t;

typedef struct __attribute__((packed)){
    uint16_t size;
    uint8_t  *buffer;
} IAP_Transport_Data_t;

typedef struct __attribute__((packed)){
    uint8_t header;
    uint8_t ctl_1;
    uint16_t pack_ctl;
    uint16_t length;
    uint8_t payload[1];
} IAP_Transport_Send_t;

typedef struct __attribute__((packed)){
    uint8_t  buf[IAP_TRANSPORT_MAX_SEND_DATA_SIZE+1];
    uint16_t size;
    uint16_t remainNum;
    IAP_Transport_Data_t data;
} IAP_Transport_sendCtl_t;

typedef void (* iap_transport_recv_cmd_cb_t)(uint8_t *data, uint16_t len);


void IAP_Transport_recv_cmd_callback_register(iap_transport_recv_cmd_cb_t cb);
uint8_t IAP_Transport_send_single_pack(uint8_t first_flag, uint16_t pack_num, uint8_t *payload_data, uint16_t payload_len);
void IAP_Transport_Init(void);

#endif


