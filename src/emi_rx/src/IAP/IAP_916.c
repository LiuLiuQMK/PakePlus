#include "IAP_916.h"
#include "platform_api.h"
#include "eflash.h"
#include "bsp_usb_hid_iap.h"
#include "btstack_util.h"
#include "IAP_Transport.h"
#include "IAP_Application.h"
#include "stdbool.h"
#include "rom_tools.h"
#include "IAP_Flash_WP.h"
#include "IAP_UserDef.h"
#include "crc16.h"
#include "profile.h"

#if 1
#define IAP_ERROR(...)  platform_printf(__VA_ARGS__)
#else
#define IAP_ERROR(...)      
#endif

#if 1
#define IAP_DEBUG(...)  platform_printf(__VA_ARGS__)
#else
#define IAP_DEBUG(...)      
#endif

// =================================================================================================

/**
 * @brief IAP_Init
 * 
 */
void IAP_Init(void){
    platform_printf("\n===>IAP Init\n");

    IAP_Flash_WP_Init();
    IAP_Application_Init();
    IAP_Transport_Init();
    bsp_usb_init();
}

