#include <stdio.h>
#include <string.h>
#include "ingsoc.h"
#include "platform_api.h"
#include "bsp_usb_hid_iap.h"
#include "btstack_util.h"
#include "profile.h"

#if 0
#define USB_DEBUG(...)	platform_printf(__VA_ARGS__)
#else
#define USB_DEBUG(...)      
#endif

/*设备描述符*/
const USB_DEVICE_DESCRIPTOR_REAL_T DeviceDescriptor __attribute__ ((aligned (4))) = USB_DEVICE_DESCRIPTOR;

/*配置描述符*/
const BSP_USB_DESC_STRUCTURE_T ConfigDescriptor __attribute__ ((aligned (4))) =
{
	USB_CONFIG_DESCRIPTOR,
    USB_INTERFACE_DESCRIPTOR_KB,  USB_HID_DESCRIPTOR_KB,  {USB_EP_IN_DESCRIPTOR_KB},
	USB_INTERFACE_DESCRIPTOR_VIA, USB_HID_DESCRIPTOR_VIA, {USB_EP_IN_DESCRIPTOR_VIA, USB_EP_OUT_DESCRIPTOR_VIA},
	USB_INTERFACE_DESCRIPTOR_CTL, USB_HID_DESCRIPTOR_CTL, {USB_EP_IN_DESCRIPTOR_CTL, USB_EP_OUT_DESCRIPTOR_CTL}
};

/*字符串描述符*/
const uint8_t StringDescriptor_0[] __attribute__ ((aligned (4))) = USB_STRING_LANGUAGE;
const uint8_t StringDescriptor_1[] __attribute__ ((aligned (4))) = USB_STRING_MANUFACTURER;
const uint8_t StringDescriptor_2[] __attribute__ ((aligned (4))) = USB_STRING_PRODUCT;

/*报表描述符*/
const uint8_t ReportKeyDescriptor[] __attribute__ ((aligned (4))) = USB_HID_KB_REPORT_DESCRIPTOR;
const uint8_t ReportViaDescriptor[] __attribute__ ((aligned (4))) = USB_HID_VIA_REPORT_DESCRIPTOR;
const uint8_t ReportCtlDescriptor[] __attribute__ ((aligned (4))) = USB_HID_CTL_REPORT_DESCRIPTOR;

BSP_KEYB_DATA_s KeybReport __attribute__ ((aligned (4))) = {
	.sendBusy = U_FALSE,
	.led_state = 0X00,
	.kb_led_state_recving = 0X00,
	.SendBit_KB = 0X00,
	.Send_System = 0X00,
	.Send_Consumer = 0X00,
	.Send_Mouse = 0X00
};

BSP_CTL_DATA_s CtlReport __attribute__ ((aligned (4))) = {
    .preReady = U_FALSE,
    .ready = U_FALSE,
    .sendBusy = U_FALSE
};

BSP_VIA_DATA_s UcmReport __attribute__ ((aligned (4))) = {
    .preReady = U_FALSE,
    .ready = U_FALSE,
    .sendBusy = U_FALSE
};

void Printf_Usb_Status_Fucn(void) {
	log_printf("KeybReport.sendBusy = %d\n", 			KeybReport.sendBusy);
	log_printf("KeybReport.led_state = %d\n", 			KeybReport.led_state);
	log_printf("KeybReport.kb_led_state_recving = %d\n",KeybReport.kb_led_state_recving);
	log_printf("KeybReport.SendBit_KB = %d\n", 			KeybReport.SendBit_KB);
	log_printf("KeybReport.Send_System = %d\n", 		KeybReport.Send_System);
	log_printf("KeybReport.Send_Consumer = %d\n", 		KeybReport.Send_Consumer);
	log_printf("KeybReport.Send_Mouse = %d\n", 			KeybReport.Send_Mouse);
	
	log_printf("CtlReport.preReady = %d\n", 			CtlReport.preReady);
	log_printf("CtlReport.ready = %d\n", 				CtlReport.ready);
	log_printf("CtlReport.sendBusy = %d\n", 			CtlReport.sendBusy);
	
	log_printf("UcmReport.preReady = %d\n", 			UcmReport.preReady);
	log_printf("UcmReport.ready = %d\n", 				UcmReport.ready);
	log_printf("UcmReport.sendBusy = %d\n", 			UcmReport.sendBusy);
}


BSP_USB_VAR_s UsbVar;
uint8_t DataRecvBuf[EP_X_MPS_BYTES] __attribute__ ((aligned (4)));
uint8_t DataSendBuf[EP_X_MPS_BYTES] __attribute__ ((aligned (4)));

uint8_t DynamicDescriptor[64] __attribute__ ((aligned (4)));

static uint8_t interfaceAlternateSetting[bNUM_INTERFACES];    // 记录可替换设置  FOR USB-IF

#define MOUSE_LEFT_KEY		0X01
#define MOUSE_RIGHT_KEY		0X02
#define MOUSE_MIDDLE_KEY	0X04
#define MOUSE_BACKWARD_KEY 	0X10
#define MOUSE_FORWARD_KEY 	0X20

#define MOUSE_X_RIGHT		{MOUSE_X_LOW = 0X01, MOUSE_X_HIGH = 0X00}
#define MOUSE_X_LEFT		{MOUSE_X_LOW = 0XFE, MOUSE_X_HIGH = 0XFF}
#define MOUSE_Y_UP			{MOUSE_Y_LOW = 0XFE, MOUSE_Y_HIGH = 0XFF}
#define MOUSE_Y_DOWN		{MOUSE_Y_LOW = 0X01, MOUSE_Y_HIGH = 0X00}

#define MOUSE_WHEEL_UP		{MOUSE_WHEEL_L = 0X01}
#define MOUSE_WHEEL_DOWN	{MOUSE_WHEEL_L = 0XFF}

#define MOUSE_WHEEL_LEFT	{MOUSE_WHEEL_H = 0XFF}
#define MOUSE_WHEEL_RIGHT	{MOUSE_WHEEL_H = 0x01}

#define MOUSE_KEY_TYPE		KeybReport.Key_Mouse_Table[0]
#define MOUSE_X_LOW			KeybReport.Key_Mouse_Table[1]
#define MOUSE_X_HIGH		KeybReport.Key_Mouse_Table[2]
#define MOUSE_Y_LOW			KeybReport.Key_Mouse_Table[3]
#define MOUSE_Y_HIGH		KeybReport.Key_Mouse_Table[4]
#define MOUSE_WHEEL_L		KeybReport.Key_Mouse_Table[5]
#define MOUSE_WHEEL_H		KeybReport.Key_Mouse_Table[6]

static bsp_usb_hid_iap_recv_cb_t			usb_hid_iap_recv_callback = NULL;
static bsp_usb_hid_iap_send_complete_cb_t	usb_hid_iap_send_complete_callback = NULL;
static bsp_usb_hid_ucm_recv_cb_t            usb_hid_ucm_recv_callback = NULL;

static void bsp_usb_hid_iap_push_rx_data_to_user(uint8_t *data, uint16_t len){
    if (usb_hid_iap_recv_callback){
        usb_hid_iap_recv_callback(data, len);
    }
}

static void bsp_usb_hid_iap_push_send_complete_to_user(void){
    if (usb_hid_iap_send_complete_callback){
        usb_hid_iap_send_complete_callback();
    }
}

static void bsp_usb_hid_ucm_push_rx_data_to_user(uint8_t *data, uint16_t len) {
    if (usb_hid_ucm_recv_callback){
        usb_hid_ucm_recv_callback(data, len);
    }
}

/*usb iap 升级接收函数*/
static USB_ERROR_TYPE_E bsp_usp_hid_ctl_rx_data_trigger(uint8_t printFLAG){
    USB_DEBUG("===> RECVING(%d) ...\n", printFLAG);
    memset(DataRecvBuf, 0x00, sizeof(DataRecvBuf));
	return USB_RecvData(ConfigDescriptor.ep_ctl[EP_CTL_IDX_GET(EP_CTL_OUT)].ep, DataRecvBuf, ConfigDescriptor.ep_ctl[EP_CTL_IDX_GET(EP_CTL_OUT)].mps, (1<<USB_TRANSFERT_FLAG_FLEXIBLE_RECV_LEN));
}

/*usb iap 升级发送函数*/
static USB_ERROR_TYPE_E bsp_usp_hid_iap_tx_data_trigger(uint8_t printFLAG, uint8_t *data, uint16_t len){
    USB_DEBUG("===> Sending(%d) ...\n", printFLAG);
    DataSendBuf[0] = CTL_REPORT_ID;
    memcpy(&DataSendBuf[1], data, len);
    return USB_SendData(ConfigDescriptor.ep_ctl[EP_CTL_IDX_GET(EP_CTL_IN)].ep, DataSendBuf, ConfigDescriptor.ep_ctl[EP_CTL_IDX_GET(EP_CTL_IN)].mps, 0);
}


/*usb via EMI接收函数*/
static USB_ERROR_TYPE_E bsp_usp_hid_ucm_rx_data_trigger(uint8_t printFLAG) {
    USB_DEBUG("===> RECVING(%d) ...\n", printFLAG);
    memset(DataRecvBuf, 0x00, sizeof(DataRecvBuf));
    return USB_RecvData(ConfigDescriptor.ep_via[EP_VIA_IDX_GET(EP_VIA_OUT)].ep, DataRecvBuf, ConfigDescriptor.ep_via[EP_VIA_IDX_GET(EP_VIA_OUT)].mps, 0);
}

/*usb via EMI发送函数*/
static USB_ERROR_TYPE_E bsp_usp_hid_ucm_tx_data_trigger(uint8_t printFLAG, uint8_t *data, uint16_t len) {
    USB_DEBUG("===> Sending(%d) ...\n", printFLAG);
    memcpy(&DataSendBuf[0], data, len);
    return USB_SendData(ConfigDescriptor.ep_via[EP_VIA_IDX_GET(EP_VIA_IN)].ep, DataSendBuf, ConfigDescriptor.ep_via[EP_VIA_IDX_GET(EP_VIA_IN)].mps, 0);
}


// trigger sending of basic key value. 绑定 EP_KB_IN 端点 发送 byte 键盘 8字节 函数
USB_HID_OperateSta_t bsp_usb_hid_keyboard_basic_report_start(void) {
	/* 如果查询到USB繁忙则不发送 返回错误繁忙*/
	if (U_TRUE == KeybReport.sendBusy) {
        return USB_HID_ERROR_BUSY;
    }

    /*发送前把 USB 繁忙标志 置位*/
    KeybReport.sendBusy = U_TRUE;
    
    if(USB_ERROR_NONE != USB_SendData(ConfigDescriptor.ep_kb[EP_KB_IDX_GET(EP_KB_IN)].ep, KeybReport.Key_Byte_Table, KEY_TABLE_LEN, 0)) {
        KeybReport.sendBusy = U_FALSE;
    }
	
    return USB_HID_ERROR_NONE;
}


/*发送 bit 键盘 16字节 数据 会默认绑定 EP_CTL_IN 这个 USB 端点 发送数据*/
static USB_ERROR_TYPE_E bsp_usp_hid_ctl_tx_data_trigger(uint8_t reportID, uint8_t *data, uint16_t len) {
    USB_DEBUG("===> Sending[RID=0x%02X] ...\n", reportID);
	
    DataSendBuf[0] = reportID;
    memcpy(&DataSendBuf[1], data, len);
	
    uint8_t size = len + 1;
    size = (size <= ConfigDescriptor.ep_ctl[EP_CTL_IDX_GET(EP_CTL_IN)].mps) ? (size) : (ConfigDescriptor.ep_ctl[EP_CTL_IDX_GET(EP_CTL_IN)].mps);
    return USB_SendData(ConfigDescriptor.ep_ctl[EP_CTL_IDX_GET(EP_CTL_IN)].ep, DataSendBuf, size, 0);
}

/*此函数 是发送 bit 键盘的函数 返回 USB_HID_ERROR_NONE 发送成功*/
USB_HID_OperateSta_t bsp_usb_hid_report_send(uint8_t reportID, uint8_t *data, uint16_t len) {
    if(!CtlReport.ready) {	//USB 错误
        return USB_HID_ERROR_NOT_READY;
    }

    if(CtlReport.sendBusy) {//USB 繁忙
        return USB_HID_ERROR_BUSY;
    }

    if( data == NULL || len == 0 || len > MAX_REPORT_SIZE ) {//数据错误
        return USB_HID_ERROR_INVALID_PARAM;
    }

    USB_HID_OperateSta_t error = USB_HID_ERROR_NONE;
    
    /* 首先设置标记为busy，防止线程安全导致busy状态异常. */
    CtlReport.sendBusy = U_TRUE;

	/*使用 EP_CTL_IN 输入端点 发送 bit 键盘 16字节 数据 返回 USB_ERROR_NONE 发送成功*/
    USB_ERROR_TYPE_E status = bsp_usp_hid_ctl_tx_data_trigger(reportID, data, len);
    switch(status) {
        case USB_ERROR_NONE: {
            error = USB_HID_ERROR_NONE;
            break;
		}
        case USB_ERROR_INVALID_INPUT: {
            error = USB_HID_ERROR_INTERNAL_ERR;
            break;
		}
        case USB_ERROR_INACTIVE_EP: {
            error = USB_HID_ERROR_INACTIVE_EP;
            break;
		}
        default: {
            error = USB_HID_ERROR_UNKNOW_ERR;
            break;
		}
    }

    if (error != USB_HID_ERROR_NONE) {
        CtlReport.sendBusy = U_FALSE;
    }
	
    return error;
}

static uint32_t bsp_usb_event_handler(USB_EVNET_HANDLER_T *event)
{
    uint32_t size;
    uint32_t status = USB_ERROR_NONE;

//    USB_DEBUG("\n\n------------- evt_id:%d ------------\n", event->id);
	
    switch(event->id)
    {
        case USB_EVENT_DEVICE_RESET:	//USB总线复位
        {
            UsbVar.config_index = ConfigDescriptor.config.configIndex; //set default configuration;   FOR USB-IF
            interfaceAlternateSetting[KB_INTERFACE_IDX]  = 0x00; // set interface 0 default alternate settings.  FOR USB-IF
			interfaceAlternateSetting[VIA_INTERFACE_IDX]  = 0x00; // set interface 1 default alternate settings.  FOR USB-IF
            interfaceAlternateSetting[CTL_INTERFACE_IDX]  = 0x00; // set interface 2 default alternate settings.     FOR USB-IF
            UsbVar.remote_wakeup = REMOTE_WAKEUP;  // 初始化远程唤醒默认值  FOR USB-IF

            #ifdef FEATURE_DISCONN_DETECT
				platform_set_timer(bsp_usb_device_disconn_timeout,160);
            #endif
            USB_DEBUG("#USB RESET\n");
			log_printf("USB RESET\n");
        }break;
        case USB_EVENT_DEVICE_SOF:		//USB SOF信号
        {
            USB_DEBUG("#USB SOF\n");
            // handle sof, need enable interrupt in config.intmask
        }break;
        case USB_EVENT_DEVICE_SUSPEND:	//suspend
        {
            USB_DEBUG("#USB SUSPEND\n");
            CtlReport.ready = U_FALSE;
            UcmReport.ready = U_FALSE;
			log_printf("suspend\n");
            // handle suspend, need enable interrupt in config.intmask
        }break;
        case USB_EVENT_DEVICE_RESUME:	//USB复位挂起
        {
            USB_DEBUG("#USB RESUME\n");
            CtlReport.ready = U_FALSE;
            UcmReport.ready = U_FALSE;
			log_printf("DEVICE_RESUME\n");
            // handle resume, need enable interrupt in config.intmask
        }break;
        case USB_EVENT_EP0_SETUP:		//SETUP事务	
        {

            USB_SETUP_T* setup = USB_GetEp0SetupData();

            USB_DEBUG("#USB EP0 SETUP: Recipient(%d), Type(%d), Direction(%d), bRequest(%d) \n",setup->bmRequestType.Recipient,
                                                                                                setup->bmRequestType.Type,
                                                                                                setup->bmRequestType.Direction,
                                                                                                setup->bRequest
                                                                                                );
            switch(setup->bmRequestType.Recipient)
            {
                case USB_REQUEST_DESTINATION_DEVICE:	//请求设备
                {
                    USB_DEBUG("##USB dst device req.\n");
                    switch(setup->bRequest)
                    {
                        case USB_REQUEST_DEVICE_SET_ADDRESS:	//设置地址
                        {
                            // handled internally
                            #ifdef FEATURE_DISCONN_DETECT
								platform_set_timer(bsp_usb_device_disconn_timeout,0);
                            #endif
                            status = USB_ERROR_NONE;
                            USB_DEBUG("###USB Set Address.\n");
                        }
                        break;
                        case USB_REQUEST_DEVICE_CLEAR_FEATURE:	//清除特性
                        {
                            UsbVar.remote_wakeup = (setup->wValue & 0xF) ? 0 : 1;
                            status = USB_ERROR_NONE;
                            USB_DEBUG("###USB Clear feature.\n");
                        }
                        break;
                        case USB_REQUEST_DEVICE_SET_FEATURE:	//设置特性请求
                        {
                            UsbVar.remote_wakeup = (setup->wValue&0xF) ? 1 : 0;
                            status = USB_ERROR_NONE;
                            USB_DEBUG("###USB Set Feature.\n");
                        }
                        break;
                        case USB_REQUEST_DEVICE_SET_CONFIGURATION:	//设置配置
                        {
                            uint8_t cfg_idx = setup->wValue & 0xFF;
                            // check if the cfg_idx is correct
                            USB_DEBUG("###USB Set Configuration: cfg_idx(%d), ConfigDescriptor.config.configIndex(%d)\n", cfg_idx, ConfigDescriptor.config.configIndex);

                            UsbVar.config_index = cfg_idx;   //  FOR USB-IF

                            if (ConfigDescriptor.config.configIndex == cfg_idx){
								status |= USB_ConfigureEp(&(ConfigDescriptor.ep_kb[0]));
                                status |= USB_ConfigureEp(&(ConfigDescriptor.ep_via[0]));
                                status |= USB_ConfigureEp(&(ConfigDescriptor.ep_via[1]));
                                status |= USB_ConfigureEp(&(ConfigDescriptor.ep_ctl[0]));
                                status |= USB_ConfigureEp(&(ConfigDescriptor.ep_ctl[1]));
                            } else {
                                USB_DEBUG("### cfg_idx error !!!\n");
                            }
                        }
                        break;
                        case USB_REQUEST_DEVICE_GET_DESCRIPTOR:	//获取描述符
                        {
                            USB_DEBUG("###USB Get descriptor:%d\n", (setup->wValue >> 8));
                            switch(setup->wValue >> 8)
                            {
                                case USB_REQUEST_DEVICE_DESCRIPTOR_DEVICE:	//获取设备描述符
                                {
                                    size = sizeof(USB_DEVICE_DESCRIPTOR_REAL_T);
                                    size = (setup->wLength < size) ? (setup->wLength) : size;

                                    status |= USB_SendData(0, (void*)&DeviceDescriptor, size, 0);

                                    USB_DEBUG("####USB Get descriptor device.\n");
                                }
                                break;
                                case USB_REQUEST_DEVICE_DESCRIPTOR_CONFIGURATION:	//获取配置描述符
                                {
                                    size = sizeof(BSP_USB_DESC_STRUCTURE_T);
                                    size = (setup->wLength < size) ? (setup->wLength) : size;

                                    status |= USB_SendData(0, (void*)&ConfigDescriptor, size, 0);

                                    USB_DEBUG("####USB Get descriptor configuration.\n");
                                }
                                break;
                                case USB_REQUEST_DEVICE_DESCRIPTOR_STRING:	//获取字符串描述符
                                {
                                    const uint8_t *addr;
                                    switch(setup->wValue & 0xFF)
                                    {
                                        case USB_STRING_LANGUAGE_IDX:		//获取USB版本
                                        {
                                            size = sizeof(StringDescriptor_0);
                                            addr = StringDescriptor_0;
                                        }break;
                                        case USB_STRING_MANUFACTURER_IDX:	//获取设备名称
                                        {
                                            size = sizeof(StringDescriptor_1);
                                            addr = StringDescriptor_1;
                                        }break;
                                        case USB_STRING_PRODUCT_IDX:		//获取生产厂商
                                        {
                                            size = sizeof(StringDescriptor_2);
                                            addr = StringDescriptor_2;
                                        }break;
                                    }

                                    size = (setup->wLength < size) ? (setup->wLength) : size;
                                    status |= USB_SendData(0, (void*)addr, size, 0);

                                    USB_DEBUG("####USB Get descriptor string: wValue(%d)\n", (setup->wValue&0xFF));
                                }
                                break;
                                default:
                                {
                                    status = USB_ERROR_REQUEST_NOT_SUPPORT;
                                    USB_DEBUG("####USB Get descriptor not support: %d!!!\n", (setup->wValue >> 8));
                                }break;
                            }
                        }
                        break;
                        case USB_REQUEST_DEVICE_GET_STATUS:		//获取USB 设备、接口、端点的状态
                        {
                            DynamicDescriptor[0] = SELF_POWERED | (UsbVar.remote_wakeup << 1);
                            DynamicDescriptor[1] = 0;
                            status |= USB_SendData(0, DynamicDescriptor, 2, 0);
                            USB_DEBUG("###USB Get status.\n");
                        }
                        break;
                        case USB_REQUEST_DEVICE_GET_CONFIGURATION:	//获取配置描述符
                        {
                            DynamicDescriptor[0] = UsbVar.config_index;  // FOR USB-IF
                            status |= USB_SendData(0, DynamicDescriptor, 1, 0);
                            USB_DEBUG("###USB Get configuration.\n");
                        }
                        break;
                        default:
                        {
                            status = USB_ERROR_REQUEST_NOT_SUPPORT;
                            USB_DEBUG("###USB dst device req not support !!!\n");
                        }break;
                    }
                }
                break;

                case USB_REQUEST_DESTINATION_INTERFACE:	//请求接口
                {
                    USB_DEBUG("=========> ##USB dst interface req: %d, %d\n", setup->bmRequestType.Type, setup->bRequest);
                    switch(setup->bmRequestType.Type)
                    {
						//--------------------------------------------------标准请求
                        case USB_REQUEST_TYPE_STANDARD:	
                        {
                            switch(setup->bRequest)
                            {
                                case USB_REQUEST_DEVICE_GET_DESCRIPTOR:	//获取配置描述符
                                {
                                    USB_DEBUG("###USB get interface descriptor: HID_CLASS(0x%x), wIndex(%d)\n",(((setup->wValue)>>8)&0xFF), setup->wIndex);
                                    switch(((setup->wValue)>>8)&0xFF)
                                    {
                                        case USB_REQUEST_HID_CLASS_DESCRIPTOR_REPORT:	//获取报表描述符
                                        {
                                            USB_DEBUG("####USB get report descriptor.\n");
                                            switch(setup->wIndex)
                                            {
												case KB_INTERFACE_IDX:				//第一个interface
												{
													size = sizeof(ReportKeyDescriptor);
													size = (setup->wLength < size) ? (setup->wLength) : size;

													status |= USB_SendData(0, (void*)&ReportKeyDescriptor, size, 0);
													KeybReport.sendBusy = U_FALSE;

													USB_DEBUG("#####USB Report Keyb Descriptor: get_size:%d, send_size:%d\n", setup->wLength, size);
												}break;
												
												case VIA_INTERFACE_IDX:				//第二个interface
												{
													size = sizeof(ReportViaDescriptor);
													size = (setup->wLength < size) ? (setup->wLength) : size;

													status |= USB_SendData(0, (void*)&ReportViaDescriptor, size, 0);
													UcmReport.preReady = U_TRUE;
													USB_DEBUG("#####USB Report Via Descriptor: get_size:%d, send_size:%d\n", setup->wLength, size);
													log_printf("setup->wIndex == %d,size = %d\n",setup->wIndex,size);
												}break;

												case CTL_INTERFACE_IDX:				//第三个interface
												{
													size = sizeof(ReportCtlDescriptor);
													size = (setup->wLength < size) ? (setup->wLength) : size;

													status |= USB_SendData(0, (void*)&ReportCtlDescriptor, size, 0);
													CtlReport.preReady = U_TRUE;
													USB_DEBUG("#####USB Report Ctl Descriptor: get_size:%d, send_size:%d\n", setup->wLength, size);
													log_printf("setup->wIndex == %d,size = %d\n",setup->wIndex,size);
												}break;
													
                                                default:
                                                {
                                                    USB_DEBUG("###USB unsupport report descriptor:%d, TODO\n", setup->wIndex);
                                                    status = USB_ERROR_REQUEST_NOT_SUPPORT;
                                                }break;
                                            }
                                        }break;
                                        default:
                                        {
                                            USB_DEBUG("###USB unsupport interface descriptor:0x%02X!!!, TODO\n", ((setup->wValue)>>8)&0xFF);
                                            status = USB_ERROR_REQUEST_NOT_SUPPORT;
                                        }break;
                                    }
                                }
                                break;
                                case USB_REQUEST_DEVICE_GET_INTERFACE:	//获取接口 FOR USB-IF
                                {
                                    USB_DEBUG("###USB Get interface: %d\n", setup->wIndex);
                                    switch(setup->wIndex)
                                    {
                                        case KB_INTERFACE_IDX:				//第一个interface
                                        {
                                            DynamicDescriptor[0] = interfaceAlternateSetting[KB_INTERFACE_IDX];
                                            status |= USB_SendData(0, DynamicDescriptor, 1, 0);
                                        }break;
                                            
                                        case VIA_INTERFACE_IDX:				//第二个interface
                                        {
                                            DynamicDescriptor[0] = interfaceAlternateSetting[VIA_INTERFACE_IDX];
                                            status |= USB_SendData(0, DynamicDescriptor, 1, 0);
                                        }break;
                                           
										case CTL_INTERFACE_IDX:				//第三个interface
										{
                                            DynamicDescriptor[0] = interfaceAlternateSetting[CTL_INTERFACE_IDX];
                                            status |= USB_SendData(0, DynamicDescriptor, 1, 0);
										}break;
                                        default:
                                        {
                                            USB_DEBUG("###USB unsupport get interface:%d, TODO\n", setup->wIndex);
                                            status = USB_ERROR_REQUEST_NOT_SUPPORT;
                                        }break;
                                    }
                                }
                                break;
                                case USB_REQUEST_DEVICE_SET_INTERFACE:	//设置接口 FOR USB-IF
                                {
                                    USB_DEBUG("###USB Set interface.\n");
                                    switch(setup->wIndex)
                                    {
                                        case KB_INTERFACE_IDX:				//第一个interface
                                        {
                                            interfaceAlternateSetting[KB_INTERFACE_IDX] = setup->wValue;
                                        }break;
                                            
                                        case VIA_INTERFACE_IDX:				//第二个interface
                                        {
                                            interfaceAlternateSetting[VIA_INTERFACE_IDX] = setup->wValue;
                                        }break;
										
										case CTL_INTERFACE_IDX:				//第三个interface
										{
                                            interfaceAlternateSetting[CTL_INTERFACE_IDX] = setup->wValue;
										}break;
										
                                        default:
                                        {
                                            USB_DEBUG("###USB unsupport set interface:%d, TODO\n", setup->wIndex);
                                            status = USB_ERROR_REQUEST_NOT_SUPPORT;
                                        }break;
                                    }
                                }
                                break;
                                default:
                                {
                                    USB_DEBUG("###USB dst interface req type standard unsupport:%d !!!, TODO\n", setup->bRequest);
                                    status = USB_ERROR_REQUEST_NOT_SUPPORT;
                                }break;
                            }
                        }
                        break;
						
						//--------------------------------------------------类请求
                        case USB_REQUEST_TYPE_CLASS:
                        {
                            switch(setup->bRequest)
                            {
                                //This request is mandatory and must be supported by all devices.
                                case USB_REQUEST_HID_CLASS_REQUEST_GET_REPORT:	//类请求
                                {
                                    USB_DEBUG("###USB get report.\n");
                                    switch(((setup->wValue)>>8)&0xFF)
                                    {
                                        case USB_REQUEST_HID_CLASS_REQUEST_REPORT_INPUT:	//Get_report
                                        {
                                            USB_DEBUG("####USB REQUEST_REPORT_INPUT.\n");
                                            switch(setup->wIndex)
                                            {
                                                case KB_INTERFACE_IDX:		//请求第一个接口
                                                {
                                                    USB_SendData(0, (void*)&KeybReport, sizeof(KeybReport.Key_Byte_Table), 0);
                                                }break;
												/*case VIA_INTERFACE_IDX:	//请求第二个接口
                                                {
                                                    USB_SendData(0, (void*)&KeybReport.ext_key_table, sizeof(KeybReport.Key_Bit_Table), 0);
                                                }break;*/
                                                /*case CTL_INTERFACE_IDX:	//请求第三个接口
                                                {
                                                    USB_SendData(0, (void*)&KeybReport.ext_key_table, sizeof(KeybReport.Key_Bit_Table), 0);
                                                }break;*/
                                                default:
                                                    USB_DEBUG("#####USB wIndex:%d, TODO\n", setup->wIndex);
                                                    status = USB_ERROR_REQUEST_NOT_SUPPORT;
                                                    break;
                                            }
                                        }break;
                                        default:
                                        {
                                            USB_DEBUG("####USB REQUEST_REPORT_INPUT unsupport:%d, TODO\n", (((setup->wValue)>>8)&0xFF));
                                            status = USB_ERROR_REQUEST_NOT_SUPPORT;
                                        }break;
                                    }
                                }break;
                                case USB_REQUEST_HID_CLASS_REQUEST_SET_REPORT:	//Set_report
                                {
                                    USB_DEBUG("###USB report set: req_type(0x%02X), wIndex(%d) \n", (((setup->wValue)>>8)&0xFF), setup->wIndex);
                                    switch(((setup->wValue)>>8) & 0xFF)
                                    {
                                        case USB_REQUEST_HID_CLASS_REQUEST_REPORT_OUTPUT:
                                        {
                                            switch(setup->wIndex)
                                            {
												case KB_INTERFACE_IDX:	//请求第一个接口 电脑下发点灯数据
												{
													// check the length, setup->wLength, for keyb, 8bit led state output is defined
													// KeybReport.led_state = setup->data[0];
													// refer to BSP_KEYB_KEYB_LED_e
													if (setup->wLength == 1){
														//需要去获取系统指示灯
														KeybReport.kb_led_state_recving = 1;
													}
												}break;
												/*case VIA_INTERFACE_IDX:	//请求第二个接口 电脑下发Set_Report,直接返回
												{
			
												}break;*/
												/*case CTL_INTERFACE_IDX:	//请求第三个接口 电脑下发Set_Report,直接返回
												{
			
												}break;*/
                                                default:
                                                    USB_DEBUG("#####USB CLASS unsupported index:%d, TODO\n", setup->wIndex);
                                                    status = USB_ERROR_REQUEST_NOT_SUPPORT;
                                                    break;
                                            }
                                        }break;
                                        case USB_REQUEST_HID_CLASS_REQUEST_REPORT_FEATURE:
                                        {
                                            log_printf("Sst feature\n");
                                        }break;
                                        default:
                                        {
                                            USB_DEBUG("###USB CLASS: req_type(0x%02X), TODO\n", ((setup->wValue)>>8)&0xFF);
                                            status = USB_ERROR_REQUEST_NOT_SUPPORT;
                                        }break;
                                    }
                                }break;
                                case USB_REQUEST_HID_CLASS_REQUEST_SET_IDLE:
                                {
                                    USB_DEBUG("###USB set idle: interface index(%d)\n", setup->wIndex);
                                    switch(setup->wIndex)
                                    {
                                        case KB_INTERFACE_IDX:
                                        {
                                            // KeybReport.sendBusy = U_TRUE; // set idle.
                                        }break;
										case VIA_INTERFACE_IDX:
                                        {
                                            // CtlReport.ready = U_FALSE; // set idle.
                                        }break;
                                        case CTL_INTERFACE_IDX:
                                        {
                                            // CtlReport.ready = U_FALSE; // set idle.
                                        }break;
                                        default:
                                            USB_DEBUG("#####USB unsupported index:%d, TODO\n", setup->wIndex);
                                            status = USB_ERROR_REQUEST_NOT_SUPPORT;
                                            break;
                                    }
                                }break;
                                default:
                                {
                                    USB_DEBUG("###USB dst interface req type class unsupport:%d !!!, TODO\n", setup->bRequest);
                                    status = USB_ERROR_REQUEST_NOT_SUPPORT;
                                }break;
                            }
                        }
                        break;
						default:
						{
							USB_DEBUG("###USB dst interface req type class unsupport:%d !!!, TODO\n", setup->bRequest);
							status = USB_ERROR_REQUEST_NOT_SUPPORT;
						}break;
                    }
                }
                break;
                
                case USB_REQUEST_DESTINATION_EP:
                {
					switch(setup->bRequest)
					{
						case USB_REQUEST_DEVICE_GET_STATUS:		//获取端点状态 USB-IF
						{
							uint8_t epWithDirection = setup->wIndex&0xFF; //ep num with direction
							DynamicDescriptor[0] = USB_IsEpStall(epWithDirection);
							DynamicDescriptor[1] = 0;
							status |= USB_SendData(0, DynamicDescriptor, 2, 0);
							USB_DEBUG("get status(EP=%#x,stall:%d)\n", epWithDirection, DynamicDescriptor[0]);
						}
						break;
					
						case USB_REQUEST_DEVICE_CLEAR_FEATURE: // USB-IF
						{
							// 当发生数据通讯错误时，USB主机也可以通过SET_FEATURE(ENDPINT_HALT)请求主动将USB设备的端点设置为停止状态。
							// 在错误被处理后，USB主机可以通过CLEAR_FEATURE（ENDPOINT_HALT)请求来清除该端点的停止状态，即使端点恢复正常数据通读的工作状态。
							// USB停止的端点在被清除ENDPOINT_HALT前，任保与该端点的通讯都应该返回STALL握手包。

							uint8_t epWithDirection = setup->wIndex&0xFF; //ep num with direction
							USB_DEBUG("get feature(EP=%#x)\n", epWithDirection);
							USB_SetStallEp(epWithDirection, U_FALSE); // RESUME EP
						}
						break;
						
						case USB_REQUEST_DEVICE_SET_FEATURE: // USB-IF
						{
							// 当发生数据通讯错误时，USB主机也可以通过SET_FEATURE(ENDPINT_HALT)请求主动将USB设备的端点设置为停止状态。
							// 在错误被处理后，USB主机可以通过CLEAR_FEATURE（ENDPOINT_HALT)请求来清除该端点的停止状态，即使端点恢复正常数据通读的工作状态。
							// USB停止的端点在被清除ENDPOINT_HALT前，任保与该端点的通讯都应该返回STALL握手包。

							uint8_t epWithDirection = setup->wIndex&0xFF; //ep num with direction
							USB_DEBUG("set feature(EP=%#x)\n", epWithDirection);
							USB_SetStallEp(epWithDirection, U_TRUE); // STALL EP
						}
						break;
						
						default:
						{
							status = USB_ERROR_REQUEST_NOT_SUPPORT;
						}
						break;
					}
                }
                break;

				default:
				{
					USB_DEBUG("###USB dst req type unsupport: %d !!!, TODO\n", setup->bmRequestType.Recipient);
					status = USB_ERROR_REQUEST_NOT_SUPPORT;
				}break;
            }

            // if status equals to USB_ERROR_REQUEST_NOT_SUPPORT: it is not supported request.
            // if status equals to USB_ERROR_NONE: it is successfully executed.
            if((USB_ERROR_NONE != status) && (USB_ERROR_REQUEST_NOT_SUPPORT != status))
            {
                USB_DEBUG("USB event exec error %x (0x%x 0x%x)\n", status, *(uint32_t*)setup,*((uint32_t*)setup+1));
            }
        }break;

        case USB_EVENT_EP_DATA_TRANSFER:
        {
            USB_DEBUG("#USB ep DATA TRANSFER\n");
            switch(event->data.type)
            {
                case USB_CALLBACK_TYPE_RECEIVE_END:
                {
                    USB_DEBUG("##USB RECV END: ep(%d)\n", event->data.ep);
					
                    // Endpoint 0 received data.
                    if (event->data.ep == 0){	//端点0 获取系统指示灯
                        uint8_t *data = (uint8_t *)USB_GetEp0SetupData();
                        if (KeybReport.kb_led_state_recving){
                            KeybReport.kb_led_state_recving = 0;
                            KeybReport.led_state = data[0];
							CtlReport.ready = U_TRUE;
                        }
                    }
                    if(event->data.ep == EP_CTL_OUT){	//OUT端点5
                        CtlReport.ready = U_TRUE;
	
						/* Push rx data to user callback. 当report id为0X3F说明主机下发了一笔64字节的数据*/
						switch(DataRecvBuf[0]){
							case CTL_REPORT_ID:{	//IAP升级数据
								/*获取数据*/
								bsp_usb_hid_iap_push_rx_data_to_user(&DataRecvBuf[1], event->data.size - 1);
								break;
							}
							case KB_REPORT_ID:{		//bit键盘OUT事务一般没有
								printf_hexdump(&DataRecvBuf[0], event->data.size);
								break;
							}
							case SYS_REPORT_ID:{	//系统类OUT事务一般没有
								printf_hexdump(&DataRecvBuf[0], event->data.size);
								break;
							}
							case CON_REPORT_ID:{	//多媒体OUT事务一般没有
								printf_hexdump(&DataRecvBuf[0], event->data.size);
								break;
							}
							case MOUSE_REPORT_ID:{	//鼠标事件OUT事务一般没有
								printf_hexdump(&DataRecvBuf[0], event->data.size);
								break;
							}
							case 0XAA:{	//2.4G强制广播
								USB_SendData(ConfigDescriptor.ep_ctl[EP_CTL_IDX_GET(EP_CTL_IN)].ep, DataRecvBuf, ConfigDescriptor.ep_ctl[EP_CTL_IDX_GET(EP_CTL_IN)].mps, 0);
								break;
							}
							default:{
								log_printf("KB RECV[%d]: ", event->data.size - 1);
								break;
							}
						}
						/* Start next rx proc. 将数据从USB里面回获取到 DataRecvBuf*/
						bsp_usp_hid_ctl_rx_data_trigger(2);
						
                    } else if(event->data.ep == EP_VIA_OUT){	//OUT端点3
                        UcmReport.ready = U_TRUE;
                        
						/*VIA 驱动下发32 字节 数据，没有repro ID*/
						bsp_usb_hid_ucm_push_rx_data_to_user(&DataRecvBuf[0], event->data.size);	/*获取数据*/
						/* Start next rx proc. 将数据从USB里面回获取到 DataRecvBuf*/
						bsp_usp_hid_ucm_rx_data_trigger(2);
					}
                }break;
                case USB_CALLBACK_TYPE_TRANSMIT_END:
                {
                    USB_DEBUG("##USB send OK: ep(%d)\n", event->data.ep);
                    /* Enter receiving status after setup complete. */
                    if(event->data.ep == 0 && CtlReport.preReady == U_TRUE){
                        CtlReport.preReady = U_FALSE;

                        /* Start first rx proc. */
                        bsp_usp_hid_ctl_rx_data_trigger(1);

                        CtlReport.ready = U_TRUE; // 枚举结束
                        log_printf("===> USB OK _iap <===\n");
                    }
                    /* Enter receiving status after setup complete. */
                    if (event->data.ep == 0 && UcmReport.preReady == U_TRUE){
                        UcmReport.preReady = U_FALSE;
                        
                        /* Start first rx proc. */
                        bsp_usp_hid_ucm_rx_data_trigger(1);
                        UcmReport.ready = U_TRUE; // 枚举结束
                        log_printf("===> USB OK _ucm <===\n");

                    }
                    /* If send ok, Clear busy status, and notify user. */
                    switch(event->data.ep)
                    {
                        case EP_KB_IN:
                        {
                            KeybReport.sendBusy = U_FALSE;
                        }break;
                        case EP_VIA_IN:
                        {
                            UcmReport.sendBusy = U_FALSE;
                        }break;  
                        case EP_CTL_IN:
                        {
                            CtlReport.sendBusy = U_FALSE;
//                            bsp_usb_hid_iap_push_send_complete_to_user();
                        }break;
                    }
                }break;
                default:
                    USB_DEBUG("##USB unsupport type:%d\n", event->data.type);
                    break;
            }
        }break;
        default:
            USB_DEBUG("#USB unsupport id:%d\n", event->id);
            break;
    }

    return status;
}

/**
 * @brief bsp_usb_init
 * 
 */

/*此函数会轮询是否有USB的数据需要发送出去*/
#if 0
uint8_t User_Usb_Send_Report(void) {
	uint8_t Return_Ack = USB_ACK_PASS;
	
	if(KeybReport.SendByte_KB)	//Byte 键盘
	{
		KeybReport.SendByte_KB = 0;
		if(USB_HID_ERROR_NONE != bsp_usb_hid_keyboard_basic_report_start()) {
			Return_Ack = USB_ACK_FAIL;
		}
	}

	if(KeybReport.SendBit_KB)	//Bit 键盘
	{
		KeybReport.SendBit_KB = 0;
		if(USB_HID_ERROR_NONE != bsp_usb_hid_report_send(KB_REPORT_ID, KeybReport.Key_Bit_Table, KEY_BIT_TABLE_LEN)){
			Return_Ack = USB_ACK_FAIL;
		}
	}
	
	if(KeybReport.Send_System)	//系统按键
	{
		KeybReport.Send_System = 0;
		if(USB_HID_ERROR_NONE != bsp_usb_hid_report_send(SYS_REPORT_ID, KeybReport.Key_Sys_Table, KEY_SYS_TABLE_LEN)){
			Return_Ack = USB_ACK_FAIL;
		}
	}
	
	if(KeybReport.Send_Consumer) //多媒体按键
	{
		KeybReport.Send_Consumer = 0;
		if(USB_HID_ERROR_NONE != bsp_usb_hid_report_send(CON_REPORT_ID, KeybReport.Key_Con_Table, KEY_CON_TABLE_LEN)){
			Return_Ack = USB_ACK_FAIL;
		}
	}
	
	if(KeybReport.Send_Mouse)	//鼠标按键
	{
		KeybReport.Send_Mouse = 0;
		if(USB_HID_ERROR_NONE != bsp_usb_hid_report_send(MOUSE_REPORT_ID, KeybReport.Key_Mouse_Table, KEY_MOU_TABLE_LEN)){
			Return_Ack = USB_ACK_FAIL;
		}
	}

	return Return_Ack;
}
#endif

#if 0
static void Read_Spi_To_2P4G_Data_Callback_Func(uint8_t type, uint8_t *data, uint16_t len){
	if((RESULT_FALSE == Get_Usb_Status()) || (RESULT_TRUE == Get_Usb_Suspend_Status())) {	//如果USB没有枚举成功，或者电脑休眠了
		return;
	}
	
	switch(type){
		case USER_KEY_BYTE_DATA:{
			memcpy(KeybReport.Key_Byte_Table, data, len);
			KeybReport.SendByte_KB = 1;
			break;
		}
		case USER_KEY_BIT_DATA:{
			memcpy(KeybReport.Key_Bit_Table, data, len);
			KeybReport.SendBit_KB = 1;
			break;
		}
		case USER_MOUSE_DATA:{
			memcpy(KeybReport.Key_Mouse_Table, data, len);
			KeybReport.Send_Mouse = 1;
			break;
		}
		case USER_CONSUMER_DATA:{
			memcpy(KeybReport.Key_Con_Table, data, len);
			KeybReport.Send_Consumer = 1;
			break;
		}
		case USER_SYSTEM_DATA:{
			memcpy(KeybReport.Key_Sys_Table, data, len);
			KeybReport.Send_System = 1;
			break;
		}
		default:{
			break;
		}
	}

	if(User_Usb_Send_Report() == USB_ACK_PASS) {
		app_2g4_buffer_send_add();
	}
}
#endif

void bsp_usb_init(void)
{
    USB_DEBUG("bsp_usb_init\n");
    USB_INIT_CONFIG_T config;

    SYSCTRL_ClearClkGateMulti(1 << SYSCTRL_ITEM_APB_USB);
    //use SYSCTRL_GetClk(SYSCTRL_ITEM_APB_USB) to confirm, USB module clock has to be 48M.
    SYSCTRL_SelectUSBClk((SYSCTRL_ClkMode)(SYSCTRL_GetPLLClk()/48000000));

    platform_set_irq_callback(PLATFORM_CB_IRQ_USB, USB_IrqHandler, NULL);

    PINCTRL_SelUSB(USB_PIN_DP,USB_PIN_DM);

    SYSCTRL_USBPhyConfig(BSP_USB_PHY_ENABLE,BSP_USB_PHY_DP_PULL_UP);

    memset(&config, 0x00, sizeof(USB_INIT_CONFIG_T));
    config.intmask = USBINTMASK_SUSP | USBINTMASK_RESUME;
    config.handler = bsp_usb_event_handler;
    USB_InitConfig(&config);
	
}

void bsp_usb_disable(void)
{
    SYSCTRL_ClearClkGateMulti(1 << SYSCTRL_ITEM_APB_USB);
    
    USB_Close();
    SYSCTRL_SetClkGateMulti(1 << SYSCTRL_ITEM_APB_USB);

    SYSCTRL_USBPhyConfig(BSP_USB_PHY_DISABLE,0);
}

void bsp_usb_device_remote_wakeup(void) {
    if(!UsbVar.remote_wakeup) {
        return;
	}

	USB_DeviceSetRemoteWakeupBit(U_TRUE);
	USB_DeviceSetRemoteWakeupBit(U_FALSE);
}

#ifdef FEATURE_DISCONN_DETECT
void bsp_usb_device_disconn_timeout(void)
{
    bsp_usb_disable();
    USB_DEBUG("USB cable disconnected.");
}
#endif

USB_HID_IAP_STA_t bsp_usb_hid_iap_send(uint8_t *data, uint16_t len) {

    if(!CtlReport.ready) {
        return USB_HID_STA_NOT_READY;
    }

    if(CtlReport.sendBusy) {
        return USB_HID_STA_BUSY;
    }

    if( data == NULL || len == 0 || len > MAX_REPORT_SIZE ) {
        return USB_HID_STA_INVALID_PARAM;
    }

    USB_HID_IAP_STA_t error = USB_HID_STA_SUCCESS;
    
    CtlReport.sendBusy = U_TRUE;

    USB_ERROR_TYPE_E status = bsp_usp_hid_iap_tx_data_trigger(1, data, len);
    switch(status) {
        case USB_ERROR_NONE:
            error = USB_HID_STA_SUCCESS;
            break;
        case USB_ERROR_INVALID_INPUT:
            error = USB_HID_STA_INTERNAL_ERR;
            break;
        case USB_ERROR_INACTIVE_EP:
            error = USB_HID_STA_INACTIVE_EP;
            break;
        default:
            error = USB_HID_STA_UNKNOW_ERR;
            break;
    }
    
    if(error != USB_HID_STA_SUCCESS) {
        CtlReport.sendBusy = U_FALSE;
    }

    return error;
}


USB_HID_IAP_STA_t bsp_usb_hid_ucm_send(uint8_t *data, uint16_t len) {

    if(!UcmReport.ready) {
        return USB_HID_STA_NOT_READY;
    }

    if(UcmReport.sendBusy) {
        return USB_HID_STA_BUSY;
    }

    if( data == NULL || len == 0 || len > EP_VIA_MPS_BYTES ) {
        return USB_HID_STA_INVALID_PARAM;
    }

    USB_HID_IAP_STA_t error = USB_HID_STA_SUCCESS;
    
    UcmReport.sendBusy = U_TRUE;

    USB_ERROR_TYPE_E status = bsp_usp_hid_ucm_tx_data_trigger(1, data, len);
    switch(status) {
        case USB_ERROR_NONE: {
            error = USB_HID_STA_SUCCESS;
            break;
		}
        case USB_ERROR_INVALID_INPUT: {
            error = USB_HID_STA_INTERNAL_ERR;
            break;
		}
        case USB_ERROR_INACTIVE_EP: {
            error = USB_HID_STA_INACTIVE_EP;
            break;
		}
        default: {
            error = USB_HID_STA_UNKNOW_ERR;
            break;
		}
    }

    if(error != USB_HID_STA_SUCCESS) {
        UcmReport.sendBusy = U_FALSE;
    }
    
    return error;
}

void bsp_usb_hid_iap_recv_callback_register(bsp_usb_hid_iap_recv_cb_t cb){
    usb_hid_iap_recv_callback = cb;
}

void bsp_usb_hid_ucm_recv_callback_register(bsp_usb_hid_ucm_recv_cb_t cb){
    usb_hid_ucm_recv_callback = cb;
}
