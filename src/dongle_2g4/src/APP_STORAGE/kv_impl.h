#ifndef __KV_IMPL_H_
#define __KV_IMPL_H_

#include <stdint.h>
#include "IAP_FLASH_MAP.H"
#include "kv_storage.h"
#include "btstack_util.h"

#define USE_KEY_REMAP_FOR_PAIR  (1)

void kv_impl_init(void);

#if USE_KEY_REMAP_FOR_PAIR
void key_remap_key_set(uint8_t key);
#endif

#endif

