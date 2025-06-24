#ifndef _IAP_FLASH_WP_H_
#define _IAP_FLASH_WP_H_

#include <stdint.h>
#include "ingsoc.h"
#include "eflash.h"
#include "rom_tools.h"

typedef enum
{
    FLASH_WP_SIZE_NONE,
    FLASH_WP_SIZE_4KB,
    FLASH_WP_SIZE_8KB,
    FLASH_WP_SIZE_16KB,
    FLASH_WP_SIZE_32KB,
    FLASH_WP_SIZE_64KB,
    FLASH_WP_SIZE_128KB,
    FLASH_WP_SIZE_256KB,
    FLASH_WP_SIZE_384KB,
    FLASH_WP_SIZE_448KB,
    FLASH_WP_SIZE_480KB,
    FLASH_WP_SIZE_496KB,
    FLASH_WP_SIZE_504KB,
    FLASH_WP_SIZE_508KB,
    FLASH_WP_SIZE_ALL_512KB,

} flash_wp_size_t;

void flash_write_protection_set(flash_wp_size_t wp_size);
void IAP_Flash_lock(void);
void IAP_Flash_Unlock(void);
void IAP_Flash_WP_Init(void);

#endif
