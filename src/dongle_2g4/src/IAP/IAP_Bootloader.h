#ifndef _IAP_BOOTLOADER_H_
#define _IAP_BOOTLOADER_H_

#include <stdint.h>
#include "eflash.h"

#define DEF_UPDATE_FLAG 0x5A5A5A5A

typedef struct block_info
{
    uint32_t size;
    uint32_t dst;
    uint32_t src;
    uint32_t magic;
} block_info_t;

/**
 * @brief Program FOTA metadata.
 *
 * @param[in] block_num         number of blocks
 * @param[in] blocks            an array of `fota_update_block_t`
 * @return                      0 if successful else non-0
 */
int program_fota_metadata(const int block_num, const fota_update_block_t *blocks);

#endif