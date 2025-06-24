#ifndef __APP_STORAGE_H_
#define __APP_STORAGE_H_

#include <stdint.h>
#include "kv_storage.h"
#include "btstack_util.h"
#include "kv_impl.h"

//key map
#define APP_STORAGE_KEY_START           (KV_USER_KEY_START)
#define APP_STORAGE_KEY_END             (KV_USER_KEY_END-1)
#define KV_KEY_INVALID                  (KV_USER_KEY_END)

enum
{
    KV_KEY_CHANNEL_1 = APP_STORAGE_KEY_START, //0xC9, 201 // 存 app_store_ble_channel_info_t
    KV_KEY_CHANNEL_2,
    KV_KEY_CHANNEL_3,

    KV_KEY_REMAP_CHANNEL_1 = (APP_STORAGE_KEY_START+10), //0xD3, 211  // 重映射存储channel_1的配对信息，需打开 USE_KEY_REMAP_FOR_PAIR 开关才能生效.
    KV_KEY_REMAP_CHANNEL_2,
    KV_KEY_REMAP_CHANNEL_3,

    KV_KEY_LOCAL_ADDR_POOL = (APP_STORAGE_KEY_START+20), //0xDD, 221 //本地地址池
    KV_KEY_KB_MODE,  // 222 存储键盘模式和通道
    KV_KEY_LED_MODE, // 223 存储led灯常规灯效模式
    KV_KEY_2G4_BOND_STATE, //224 存储2.4G绑定信息
};

#define KV_KEY_CHANNEL(x)               (KV_KEY_CHANNEL_1 + x - 1) // x=1,2,3

#define KV_KEY_REMAP_KEY(x)             (KV_KEY_REMAP_CHANNEL_1 + x - 1) // x=1,2,3
#define KV_KEY_REMAP_KEY_UPDATE(x)      key_remap_key_set(KV_KEY_REMAP_KEY(x)) // x=1,2,3  更新底层配对映射key值 

// struct.

typedef struct __attribute__((packed)) {
    uint8_t state; 
    uint32_t addr;
} app_store_2g4_bond_state_t;

void print_addr(const uint8_t *addr);
void print_addr2(const uint8_t *addr);

uint8_t app_storage_2g4_state_read(void);
uint32_t app_storage_2g4_addr_read(void);
uint8_t app_storage_2g4_addr_write(uint32_t addr);
void app_storage_2g4_state_reset(void);
void app_storage_2g4_state_init(void);
void app_storage_init(void);

#endif
