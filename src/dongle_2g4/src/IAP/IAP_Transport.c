#include "string.h"
#include "IAP_Transport.h"
#include "platform_api.h"
#include "bsp_usb_hid_iap.h"
#include "btstack_util.h"
#include "IAP_Application.h"
#include "IAP_UserDef.h"


#if USER_IAP_TRANSPORT_ERROR_EN
#define TRANSPORT_ERROR(...)	platform_printf(__VA_ARGS__)
#else
#define TRANSPORT_ERROR(...)      
#endif

#if USER_IAP_TRANSPORT_DEBUG_EN
#define TRANSPORT_DEBUG(...)	platform_printf(__VA_ARGS__)
#else
#define TRANSPORT_DEBUG(...)      
#endif



static IAP_Transport_Ctl_t TransportRecvCtl;
static IAP_Transport_Data_t TransportRecvData;
static IAP_Transport_sendCtl_t sendCtl; 
static iap_transport_recv_cmd_cb_t     iap_transport_recv_cmd_callback = NULL;


static uint8_t IAP_TransportGetBCC(uint8_t *data, uint16_t len){
    uint8_t BCC = data[0];
    for(uint16_t i=1; i<len; i++){
        BCC ^= data[i];
    }
    return BCC;
}

static uint8_t IAP_TransportCheckBCC(uint8_t *data, uint16_t len){
    uint8_t BCC = IAP_TransportGetBCC(data, len-1);
    if(BCC != data[len-1]){
        TRANSPORT_ERROR("[Transport] error: BCC calc=[0x%02X], recv=[0x%02X]\n", BCC, data[len-1]);
        return 1;
    }
    return 0;
}

static void IAP_TransportClearRecvData(void){
    TransportRecvData.buffer = IAP_GetAppBuffer();
    TransportRecvData.size = 0;
}

static uint8_t IAP_TransportAttachData(uint8_t *data, uint16_t len){
    if ((TransportRecvData.size+len) > IAP_APP_MAX_BUFFER_SIZE)
        return 1;
    memcpy((uint8_t *)&TransportRecvData.buffer[TransportRecvData.size], data, len);
    TransportRecvData.size += len;
    return 0;
}

static void IAP_TransportRecvGotoIdle(void){
    memset(&TransportRecvCtl, 0, sizeof(TransportRecvCtl));
    TransportRecvCtl.state = IAP_TRANSPORT_STA_IDLE;
    IAP_TransportClearRecvData();
}

static IAP_TransportErr_t IAP_Transport_Dispatch(uint8_t *data, uint16_t len){

    IAP_Transport_Recv_t * transRecv = (IAP_Transport_Recv_t *)data;

#if 0
    TRANSPORT_DEBUG("header: 0x%02x\n", transRecv->header);
    TRANSPORT_DEBUG("ctl_1: 0x%02x\n", transRecv->ctl_1);
    TRANSPORT_DEBUG("pack_ctl: 0x%04x\n", transRecv->pack_ctl);
    TRANSPORT_DEBUG("length: 0x%04x\n", transRecv->payload_length);
    if(transRecv->payload_length){
        TRANSPORT_DEBUG("payload[%d]:\n", transRecv->payload_length);
        printf_hexdump(transRecv->payload, transRecv->payload_length);
    }
#endif

    // check len. 应用包小于7个字节
    if(len < IAP_TRANSPORT_MIN_LEN){
        TRANSPORT_ERROR("[Transport] error: len too small\n");
        IAP_TransportRecvGotoIdle();
        return IAP_TRANSPORT_ERR_LENGTH;
    }

    // check header. 不是0XAA
    if(transRecv->header != IAP_TRANSPORT_HEADER){
        TRANSPORT_ERROR("[Transport] error: header\n");
        IAP_TransportRecvGotoIdle();
        return IAP_TRANSPORT_ERR_HEADER;
    }

    // check direction. 最高位是否为1 下发数据
    if(!IS_DIRECTION_DOWN(transRecv->ctl_1)){
        TRANSPORT_ERROR("[Transport] error: direction\n");
        IAP_TransportRecvGotoIdle();
        return IAP_TRANSPORT_ERR_DIRECTION;
    }

    // check pack type. 应答包
    if(!IS_PACK_TYPE_DATA(transRecv->ctl_1)){
        TRANSPORT_ERROR("[Transport] error: pack type\n");
        IAP_TransportRecvGotoIdle();
        return IAP_TRANSPORT_ERR_PACK_TYPE;
    }

    // check data length 包长是否超过最大包长
    if(transRecv->payload_length > (len - IAP_TRANSPORT_MIN_LEN)){
        TRANSPORT_ERROR("[Transport] error: payload length: %d > %d\n", transRecv->payload_length, (len - IAP_TRANSPORT_MIN_LEN));
        IAP_TransportRecvGotoIdle();
        return IAP_TRANSPORT_ERR_PACK_TYPE;
    }

    // check BCC. [ctl_1 ~ BCC] [without filled bytes.] 从第一个字节到倒数第二个字节，每一个字节都异或一次，然后和最后一个字节对比
    if(IAP_TransportCheckBCC(&transRecv->ctl_1, transRecv->payload_length + IAP_TRANSPORT_MIN_LEN - 1)){
        TRANSPORT_ERROR("[Transport] error: BCC\n");
        IAP_TransportRecvGotoIdle();
        return IAP_TRANSPORT_ERR_BCC;
    }
    
    // recv data.
    switch(TransportRecvCtl.state){
        case IAP_TRANSPORT_STA_IDLE:{
            // first pack handle. 必须得是第一包
            if(IS_FIRST_PACK(transRecv->pack_ctl)){

                // store data. 将数据装载在TransportRecvData.buffer里面最多8100个 应用包的包头
                if(IAP_TransportAttachData(transRecv->payload, transRecv->payload_length)){
                    TRANSPORT_ERROR("[Transport] error: buffer overflow1\n");
                    IAP_TransportRecvGotoIdle();
                    return IAP_TRANSPORT_ERR_BUFFER_OVERFLOW;
                }

                // switch state.后面没有包了最后一包，直接写入
                if (IS_LAST_PACK(transRecv->pack_ctl)){
                    if(iap_transport_recv_cmd_callback)
                        iap_transport_recv_cmd_callback(TransportRecvData.buffer, TransportRecvData.size);
                    IAP_TransportRecvGotoIdle();
                    return IAP_TRANSPORT_ERR_NONE_REPLY;
                } else {
                    // need next pack. 当加载玩第一包之后把前面的八个字节装载玩，后面继续装载
                    TransportRecvCtl.total_pack_num = GET_PACK_NUM(transRecv->pack_ctl);
                    TransportRecvCtl.next_pack_num = TransportRecvCtl.total_pack_num-1;
                    TransportRecvCtl.state = IAP_TRANSPORT_STA_RECVING;
                }
            } else {
                TRANSPORT_ERROR("[Transport] error: not first pack\n");
                IAP_TransportRecvGotoIdle();
                return IAP_TRANSPORT_ERR_PACK_NUM;
            }
        }
        break;
        case IAP_TRANSPORT_STA_RECVING:{

            // first pack ? 数据传输过程中已经加载完第一包了
            if(IS_FIRST_PACK(transRecv->pack_ctl)){
                TRANSPORT_ERROR("[Transport] error: repeat first pack.\n");
                IAP_TransportRecvGotoIdle();
                return IAP_TRANSPORT_ERR_PACK_NUM;
            }

            // check pack number. 当前包包数不对
            if (TransportRecvCtl.next_pack_num != GET_PACK_NUM(transRecv->pack_ctl)){
                TRANSPORT_ERROR("[Transport] pack num error: recv(%d), need(%d)\n", GET_PACK_NUM(transRecv->pack_ctl), TransportRecvCtl.next_pack_num);
                IAP_TransportRecvGotoIdle();
                return IAP_TRANSPORT_ERR_PACK_NUM;
            }

            // store data. 将数据装载在TransportRecvData.buffer里面最多8100个 此处装载的已经是包数据了
            if(IAP_TransportAttachData(transRecv->payload, transRecv->payload_length)){
                TRANSPORT_ERROR("[Transport] error: buffer overflow2\n");
                IAP_TransportRecvGotoIdle();
                return IAP_TRANSPORT_ERR_BUFFER_OVERFLOW;
            }

            // swtich state. 后面没有包了最后一包，直接写入
            if(IS_LAST_PACK(transRecv->pack_ctl)){
                if(iap_transport_recv_cmd_callback)
                    iap_transport_recv_cmd_callback(TransportRecvData.buffer, TransportRecvData.size);
                IAP_TransportRecvGotoIdle();
				return IAP_TRANSPORT_ERR_NONE_REPLY;
            } else {
                if(TransportRecvCtl.next_pack_num)
                    TransportRecvCtl.next_pack_num--;
            }
        }
        break;
    }
    return IAP_TRANSPORT_ERR_NONE;
}

static void IAP_Transport_Send_Dummy() {
    memset(sendCtl.buf, 0, sizeof(sendCtl.buf));
    sendCtl.buf[0] = IAP_TRANSPORT_HEADER_DUMMY;
    bsp_usb_hid_iap_send(sendCtl.buf, IAP_TRANSPORT_MAX_SEND_DATA_SIZE);
}

// AA C0 00 80 01 00 ERR BCC
static USB_HID_IAP_STA_t IAP_Transport_Send_ACK(uint8_t err){
    IAP_Transport_ACK_t ACK;
    ACK.header   = IAP_TRANSPORT_HEADER;
    ACK.ctl_1    = 0xC0;   //up, ack
    ACK.pack_ctl = 0x8000; //first flag=1, num=0
    ACK.length   = 0x0001; //len=1
    ACK.errCode  = err;
    ACK.BCC      = IAP_TransportGetBCC(&ACK.ctl_1, sizeof(IAP_Transport_ACK_t)-2);
    return bsp_usb_hid_iap_send((uint8_t *)&ACK, sizeof(IAP_Transport_ACK_t));
}

// send single pack. 
uint8_t IAP_Transport_send_single_pack(uint8_t first_flag, uint16_t pack_num, uint8_t *payload_data, uint16_t payload_len){

    // check payload length.
    if(payload_len > IAP_TRANSPORT_MAX_SEND_SINGLE_PAYLOAD){
        return (uint8_t)USB_HID_STA_SIZE_TOO_LARGE;
    }

    // prepare data.
    IAP_Transport_Send_t * sendRSP = (IAP_Transport_Send_t * )sendCtl.buf;
    // Fill header
    sendRSP->header    = IAP_TRANSPORT_HEADER;
    // Fill ctl_1
    sendRSP->ctl_1     = 0x80;   //up, data
    // Fill pack_ctl
    if(first_flag)  sendRSP->pack_ctl = 0x8000;
    else            sendRSP->pack_ctl = 0x0000;
    sendRSP->pack_ctl |= (pack_num&0x7FFF);
    // Fill payload_len
    sendRSP->length    = payload_len;
    // Fill payload data.
    memcpy(sendRSP->payload, payload_data, payload_len);
    // Fill bcc.
    sendRSP->payload[payload_len] = IAP_TransportGetBCC(&sendRSP->ctl_1, 1+2+2+payload_len); //BCC: ctl_1 ~ payload

    // prepare length.
    sendCtl.size = payload_len + IAP_TRANSPORT_MIN_LEN;

#if USER_IAP_TRANSPORT_DEBUG_EN
    log_printf("SingleSend[%d]: ", sendCtl.size);
    printf_hexdump(sendCtl.buf, sendCtl.size);
#endif

    // send data.
    bsp_usb_hid_iap_send(sendCtl.buf, sendCtl.size);
    return 0;
}

// recv callback.
static void bsp_usb_hid_iap_recv_callback(uint8_t *data, uint16_t len){
    
#if USER_IAP_TRANSPORT_DEBUG_EN
    log_printf("RECV[%d]: ", len);
    printf_hexdump(data, len);
#endif

	/*收到一笔64字节的USB数据会通过此函数装载数据*/
    IAP_TransportErr_t errCode = IAP_Transport_Dispatch(data, len);
    if(errCode != IAP_TRANSPORT_ERR_NONE_REPLY){
		if(errCode != IAP_TRANSPORT_ERR_NONE){
			/*如果出错返回错误的ACK*/
            IAP_Transport_Send_ACK(errCode);
		}else{
			/*如果成功则返回一笔USB数据*/
            IAP_Transport_Send_Dummy();
        }
    }
}

// init.
void IAP_Transport_Init(void){
    IAP_TransportRecvGotoIdle();
    IAP_TransportClearRecvData();
    bsp_usb_hid_iap_recv_callback_register(bsp_usb_hid_iap_recv_callback);
}

void IAP_Transport_recv_cmd_callback_register(iap_transport_recv_cmd_cb_t cb){
    iap_transport_recv_cmd_callback = cb;
}

