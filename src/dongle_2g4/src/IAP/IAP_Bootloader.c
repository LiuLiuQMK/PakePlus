#include "IAP_Bootloader.h"

int program_fota_metadata(const int block_num, const fota_update_block_t *blocks)
{
#define START               (FLASH_BASE + EFLASH_ERASABLE_SIZE)

    int i;
    block_info_t info =
    {
        .magic = DEF_UPDATE_FLAG,
    };

    erase_flash_sector(START);

    uint32_t addr = START + EFLASH_ERASABLE_SIZE - sizeof(info);

    for (i = 0; i < block_num; i++, addr -= sizeof(info))
    {
        info.size = blocks[i].size;
        info.dst  = blocks[i].dest;
        info.src  = blocks[i].src;
        int r = write_flash(addr, (uint8_t *)&info, sizeof(info));
        if (r) return r;
    }

    return 0;
}