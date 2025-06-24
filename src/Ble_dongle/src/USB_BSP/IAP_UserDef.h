#ifndef _IAP_USER_DEF_H_
#define _IAP_USER_DEF_H_

// Product config.
#define USER_FLASH_LOCK_EN              (0)     /* <= Enable flash protection function. The test phase can disable, 
                                                        and the MP phase is strongly recommended to enable. */

// Test config.
#define USER_ERASE_BOOT_PARAM_EN        (0)     /* <= For test. Enable erase boot params area for startup. */
#define USER_APP_CODE_CHECK_CRC_EN      (0)     /* <= For test. Enable checking CRC of APP before jumping to APP. */

// Log enable/disable
#define USER_IAP_MAIN_DEBUG_LOG_EN      (1)
#define USER_IAP_MAIN_ERROR_LOG_EN      (1)
#define USER_IAP_PARAM_DEBUG_LOG_EN     (0)
#define USER_IAP_PARAM_ERROR_LOG_EN     (1)
#define USER_IAP_APP_DEBUG_LOG_EN       (0)
#define USER_IAP_APP_ERROR_LOG_EN       (1)
#define USER_IAP_TRANSPORT_DEBUG_EN     (0)
#define USER_IAP_TRANSPORT_ERROR_EN     (1)

// version information.
#define USER_DEF_CHIP_ID                "ING91680C_TB"          // max = 15bytes.
#define USER_DEF_ITEM_STR               "HS_DG_WH_V01"          // max = 23bytes.

// boot usb information.
#define USER_DEF_USB_VID                (0x36B0)
#define USER_DEF_USB_PID                (0x3002)
#define USER_DEF_IAP_REPORT_ID          (0x3F)


#define HID_USAGE_KEYBOARD_PRINT_SCREEN	0x46
#define HID_USAGE_KEYBOARD_APPLICATION	0x65

/* Usage Pages */
#define HID_USAGE_PAGE_UNDEFINED		0x00
#define HID_USAGE_PAGE_GENERIC			0x01
#define HID_USAGE_PAGE_SIMULATION		0x02
#define HID_USAGE_PAGE_VR				0x03
#define HID_USAGE_PAGE_SPORT			0x04
#define HID_USAGE_PAGE_GAME				0x05
#define HID_USAGE_PAGE_DEV_CONTROLS		0x06
#define HID_USAGE_PAGE_KEYBOARD			0x07
#define HID_USAGE_PAGE_LED				0x08
#define HID_USAGE_PAGE_BUTTON			0x09
#define HID_USAGE_PAGE_ORDINAL			0x0A
#define HID_USAGE_PAGE_TELEPHONY		0x0B
#define HID_USAGE_PAGE_CONSUMER			0x0C
#define HID_USAGE_PAGE_DIGITIZER		0x0D
#define HID_USAGE_PAGE_UNICODE			0x10
#define HID_USAGE_PAGE_ALPHANUMERIC		0x14


/* Generic Desktop Page (0x01) */
#define HID_USAGE_GENERIC_POINTER		0x01
#define HID_USAGE_GENERIC_MOUSE			0x02
#define HID_USAGE_GENERIC_JOYSTICK		0x04
#define HID_USAGE_GENERIC_GAMEPAD		0x05
#define HID_USAGE_GENERIC_KEYBOARD		0x06
#define HID_USAGE_GENERIC_KEYPAD		0x07
#define HID_USAGE_GENERIC_SYSTEM_CTL	0x80
#define HID_USAGE_GENERIC_SYSCTL_POWER	0x81
#define HID_USAGE_GENERIC_SYSCTL_SLEEP	0x82
#define HID_USAGE_GENERIC_SYSCTL_WAKE	0x83
#define HID_USAGE_GENERIC_WHEEL			0x38
#define HID_USAGE_GENERIC_X				0x30
#define HID_USAGE_GENERIC_Y				0x31

/* LED Page (0x08) */
#define HID_USAGE_LED_NUM_LOCK       	0x01
#define HID_USAGE_LED_CAPS_LOCK      	0x02
#define HID_USAGE_LED_SCROLL_LOCK    	0x03
#define HID_USAGE_LED_COMPOSE        	0x04
#define HID_USAGE_LED_KANA            	0x05
/* Consumer Page (0x0C) */
#define HID_USAGE_CONSUMER_CONTROL		0x01

/* Modifier Keys */
#define HID_USAGE_KEYBOARD_LCTRL      	0xE0
#define HID_USAGE_KEYBOARD_LSHFT      	0xE1
#define HID_USAGE_KEYBOARD_LALT       	0xE2
#define HID_USAGE_KEYBOARD_LGUI       	0xE3
#define HID_USAGE_KEYBOARD_RCTRL      	0xE4
#define HID_USAGE_KEYBOARD_RSHFT      	0xE5
#define HID_USAGE_KEYBOARD_RALT       	0xE6
#define HID_USAGE_KEYBOARD_RGUI       	0xE7
#define HID_USAGE_KEYBOARD_SCROLL_LOCK	0x47
#define HID_USAGE_KEYBOARD_NUM_LOCK   	0x53
#define HID_USAGE_KEYBOARD_CAPS_LOCK  	0x39

/* Main Items */
#define HID_Input(x)           			0x81,x
#define HID_Output(x)          			0x91,x
#define HID_Feature(x)         			0xB1,x
#define HID_Collection(x)      			0xA1,x
#define HID_EndCollection      			0xC0

/* Data (Input, Output, Feature) */
#define HID_Data               			(0 << 0)
#define HID_Constant           			(1 << 0)
#define HID_Array              			(0 << 1)
#define HID_Variable           			(1 << 1)
#define HID_Absolute           			(0 << 2)
#define HID_Relative           			(1 << 2)
#define HID_NoWrap             			(0 << 3)
#define HID_Wrap               			(1 << 3)
#define HID_Linear             			(0 << 4)
#define HID_NonLinear          			(1 << 4)
#define HID_PreferredState     			(0 << 5)
#define HID_NoPreferred        			(1 << 5)
#define HID_NoNullPosition     			(0 << 6)
#define HID_NullState          			(1 << 6)
#define HID_NonVolatile        			(0 << 7)
#define HID_Volatile           			(1 << 7)

/* Collection Data */
#define HID_Physical           			0x00
#define HID_Application        			0x01
#define HID_Logical            			0x02
#define HID_Report             			0x03
#define HID_NamedArray         			0x04
#define HID_UsageSwitch        			0x05
#define HID_UsageModifier      			0x06

/* Global Items */
#define HID_UsagePage(x)       			0x05,x
#define HID_UsagePageVendor(x) 			0x06,x,0xFF
#define HID_LogicalMin(x)      			0x15,x
#define HID_LogicalMinS(x)     			0x16,(x&0xFF),((x>>8)&0xFF)
#define HID_LogicalMinL(x)     			0x17,(x&0xFF),((x>>8)&0xFF),((x>>16)&0xFF),((x>>24)&0xFF)
#define HID_LogicalMax(x)      			0x25,x
#define HID_LogicalMaxS(x)     			0x26,(x&0xFF),((x>>8)&0xFF)
#define HID_LogicalMaxL(x)     			0x27,(x&0xFF),((x>>8)&0xFF),((x>>16)&0xFF),((x>>24)&0xFF)
#define HID_PhysicalMin(x)     			0x35,x
#define HID_PhysicalMinS(x)    			0x36,(x&0xFF),((x>>8)&0xFF)
#define HID_PhysicalMinL(x)    			0x37,(x&0xFF),((x>>8)&0xFF),((x>>16)&0xFF),((x>>24)&0xFF)
#define HID_PhysicalMax(x)     			0x45,x
#define HID_PhysicalMaxS(x)    			0x46,(x&0xFF),((x>>8)&0xFF)
#define HID_PhysicalMaxL(x)    			0x47,(x&0xFF),((x>>8)&0xFF),((x>>16)&0xFF),((x>>24)&0xFF)
#define HID_UnitExponent(x)    			0x55,x
#define HID_Unit(x)            			0x65,x
#define HID_UnitS(x)           			0x66,(x&0xFF),((x>>8)&0xFF)
#define HID_UnitL(x)           			0x67,(x&0xFF),((x>>8)&0xFF),((x>>16)&0xFF),((x>>24)&0xFF)
#define HID_ReportSize(x)      			0x75,x
#define HID_ReportID(x)        			0x85,x
#define HID_ReportCount(x)     			0x95,x
#define HID_Push               			0xA0
#define HID_Pop                			0xB0

/* Local Items */
#define HID_Usage(x)           			0x09,x
#define HID_UsageS(x)          			0x0A,(x&0xFF),((x>>8)&0xFF)
#define HID_UsageMin(x)        			0x19,x
#define HID_UsageMinS(x)				0x1A,(x&0xFF),((x>>8)&0xFF)
#define HID_UsageMax(x)        			0x29,x
#define HID_UsageMaxS(x)       			0x2A,(x&0xFF),((x>>8)&0xFF)

#endif

