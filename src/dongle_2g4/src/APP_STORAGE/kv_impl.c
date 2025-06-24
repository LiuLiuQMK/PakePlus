/**
 * @brief An backend for kv storage
 *
 * Features:
 *
 * 1. Flash written occurs on the level of a k-v pair
 * 1. Dual Flash block are used for secure operation
 *    Note: block size = `EFLASH_ERASABLE_SIZE`
 * 1. value length must be > 0.
*/

#include <string.h>
#include <errno.h>
#include "eflash.h"
#include "kv_storage.h"
#include "platform_api.h"
#include "port_gen_os_driver.h"
#include "IAP_FLASH_MAP.H"
#include "kv_impl.h"
#include "profile.h"


#if (INGCHIPS_FAMILY == INGCHIPS_FAMILY_918)
#define DB_FLASH_ADDRESS  0x42000
#elif (INGCHIPS_FAMILY == INGCHIPS_FAMILY_916)
#define DB_FLASH_ADDRESS  RSVD_START_ADDR
#else
#error unknown or unsupported chip family
#endif

#define BLOCK_SIZE              (EFLASH_ERASABLE_SIZE)

#define DB_FLASH_ADDR_END		(DB_FLASH_ADDRESS + EFLASH_ERASABLE_SIZE)

#define DB_FLASH_ADDRESS_BACKUP	(DB_FLASH_ADDRESS + EFLASH_ERASABLE_SIZE)

// maximum number k-v can be stored
#ifndef KV_CACHE_SIZE
#define KV_CACHE_SIZE		50
#endif

#define GEN_OS          ((const gen_os_driver_t *)platform_get_gen_os_driver())

#pragma pack (push, 1)
struct kv_item
{
    uint8_t len;		// length of data (<= KV_VALUE_MAX_LEN)
    kvkey_t key;
    uint8_t data[0];
};
#pragma pack (pop)

struct kv_cache_item
{
    kvkey_t         key;
    uint8_t         len;
    void           *data;
};

struct kv_cache_item  kv_cache[KV_CACHE_SIZE] = {0};

static uint32_t kv_storage_tail = 0;

#if USE_KEY_REMAP_FOR_PAIR

typedef enum {
    KEY_REMAP_TYPE_PUT = 0,
    KEY_REMAP_TYPE_REMOVE,
    KEY_REMAP_TYPE_GET,
    KEY_REMAP_TYPE_VISIT,
    KEY_REMAP_TYPE_MODIFY,
} kv_remap_type_t;

static kvkey_t key_channel = 0xFF;

void key_remap_key_set(uint8_t key){
    key_channel = key;
}


static kvkey_t key_remap_by_type(kv_remap_type_t type, kvkey_t key){
    kvkey_t new_key = key;
    switch(type)
    {
        case KEY_REMAP_TYPE_PUT:
            if(key >= 0x01 && key <= 0x05){
                new_key = key_channel;
            }
            break;
        case KEY_REMAP_TYPE_REMOVE:
            if(key >= 0x01 && key <= 0x05){
                new_key = key_channel;
            }
            break;
        case KEY_REMAP_TYPE_GET:
            if(key == 0x01){
                new_key = key_channel;
            }
            break;
        case KEY_REMAP_TYPE_VISIT:
            if(key == 0x01){
                new_key = key_channel;
            }
            break;
        case KEY_REMAP_TYPE_MODIFY:
            if(key == 0x01){
                new_key = key_channel;
            }
            break;
    }    
    return new_key;
}

#endif

static uint32_t next_item(const struct kv_item *item)
{
    uint32_t s = (sizeof(struct kv_item) + item->len + 3) & ~0x3;

    return (uint32_t)item + s;
}

static int kv_flash_repair(uint32_t start, uint32_t end)
{
    struct kv_item *item = (struct kv_item *)start;
    while ((start < end) && (item->len < 255))
    {
        uint32_t next = next_item(item);
        if (next >= end)
            break;
        start = next;
        item = (struct kv_item *)start;
    }

    kv_storage_tail = start;

    while (start < end)
    {
        uint32_t *p = (uint32_t *)start;
        if (*p != 0xffffffff)
            return 1;
        start += 4;
    }
    return 0;
}

static void kv_do_backup(uint32_t from, uint32_t to)
{
#if (INGCHIPS_FAMILY == INGCHIPS_FAMILY_918)
        program_flash(to, (uint8_t *)from, BLOCK_SIZE);
#elif (INGCHIPS_FAMILY == INGCHIPS_FAMILY_916)
        int i;
        uint32_t buf[16];
        erase_flash_sector(to);

        for (i = 0; i < BLOCK_SIZE / sizeof(buf); i++)
        {
            memcpy(buf, (void *)from, sizeof(buf));
            write_flash(to, (uint8_t *)buf, sizeof(buf));
            from += sizeof(buf);
            to += sizeof(buf);
        }
#endif
}

static void kv_reset(void)
{
    log_printf("[KV]%s\n", __func__);
#if (INGCHIPS_FAMILY == INGCHIPS_FAMILY_918)
    erase_flash_page(DB_FLASH_ADDRESS);
#elif (INGCHIPS_FAMILY == INGCHIPS_FAMILY_916)
    erase_flash_sector(DB_FLASH_ADDRESS);
#endif
    kv_storage_tail = DB_FLASH_ADDRESS;
}

static void impl_kv_remove_all(void)
{
    log_printf("[KV]%s\n", __func__);
    int i;
    for (i = 0; i < KV_CACHE_SIZE; i++)
    {
        if (kv_cache[i].data)
        {
            kv_cache[i].key = (kvkey_t)-1;
            GEN_OS->free(kv_cache[i].data);
            kv_cache[i].data = NULL;
        }
    }
    kv_reset();
}

static struct kv_item *kv_search_flash(const kvkey_t key, uint32_t start, uint32_t end)
{
    struct kv_item *found = NULL;
    struct kv_item *item = (struct kv_item *)start;
    while (item->len < 255)
    {
        uint32_t next = next_item(item);
        if (next > end)
            break;

        if (item->key == key)
        {
            found = item;
        }

        start = next;
        item = (struct kv_item *)start;
    }

    if (found)
        return found->len > 0 ? found : NULL;
    else
        return NULL;
}

static struct kv_cache_item *kv_search_cache(const kvkey_t key)
{
    int i;
    for (i = 0; i < KV_CACHE_SIZE; i++)
    {
        if ((kv_cache[i].key == key) && (kv_cache[i].data))
        {
            return kv_cache + i;
        }
    }
    return NULL;
}

static void impl_do_kv_visit(f_kv_visitor visitor, void *user_data, uint32_t start, uint32_t end)
{
#if USE_KEY_REMAP_FOR_PAIR
    int k;
    for (k = 0; k <= (KV_USER_KEY_END-1); k++)
    {
        kvkey_t new_key = key_remap_by_type(KEY_REMAP_TYPE_VISIT, (kvkey_t)k); 
        // host 永远visit不到0x01的key，即认为0x01一直不存在，所以host存储时永远使用0x01进行存储；
        // host 回连的时候采用的是遍历1~5这几个key的内容是否存在，以及是否可解析/地址是否匹配等来进行回连的，并不采用visit，所以不会引起回连问题；
        // 所以，目前这种方案可以很好的解决多通道问题，但前提是host的运行机制保持以上两个机制不更改，否则需要重新适配；
        // 吉哥说以上机制应该不会变更，所以，采用此方案，应该可以比较巧妙的解决了存储只存储0x01却不会引起回连问题。

        struct kv_cache_item * r = kv_search_cache(new_key);
        if (r)
        {
            // log_printf("[KV]cache, ori_key:%d, new_key:%d\n", k, new_key);
            if (visitor(r->key, r->data, r->len, user_data) != KV_OK)
                break;
        }
        else
        {
            struct kv_item *item = kv_search_flash(new_key, start, end);
            if (item){
                // log_printf("[KV]flash, ori_key:%d, new_key:%d\n", k, new_key);
                if (visitor(item->key, item->data, item->len, user_data) != KV_OK)
                    break;
            }
                
        }
    }

#else
    int k;
    for (k = 0; k <= KV_USER_KEY_END; k++)
    {
        struct kv_cache_item * r = kv_search_cache(k);
        if (r)
        {
            if (visitor(r->key, r->data, r->len, user_data) != KV_OK)
                break;
        }
        else
        {
            struct kv_item *item = kv_search_flash(k, start, end);
            if (item)
                if (visitor(item->key, item->data, item->len, user_data) != KV_OK)
                    break;
        }
    }
#endif
}

static void impl_kv_visit(f_kv_visitor visitor, void *user_data)
{
    // log_printf("[KV]%s\n", __func__);
    impl_do_kv_visit(visitor, user_data, DB_FLASH_ADDRESS, DB_FLASH_ADDR_END);
}

static struct kv_cache_item *kv_search(const kvkey_t key)
{
    int i;
    struct kv_item *item;
    struct kv_cache_item * r = kv_search_cache(key);
    if (r) return r;

    item = kv_search_flash(key, DB_FLASH_ADDRESS, DB_FLASH_ADDR_END);

    if (item && item->len > 0)
    {
        for (i = 0; i < KV_CACHE_SIZE; i++)
        {
            if (kv_cache[i].data == NULL)
                break;
        }
        if (i >= KV_CACHE_SIZE)
        {
            // cache can't store so many items
            return NULL;
        }

        kv_cache[i].data = GEN_OS->malloc(item->len);
        kv_cache[i].key = key;
        kv_cache[i].len = item->len;
        memcpy(kv_cache[i].data, item->data, item->len);

        return kv_cache + i;
    }

    return NULL;
}

static int kv_do_append_key(kvkey_t key, const void *data, int len);

static int kv_visitor_dump(const kvkey_t key, const uint8_t *data, const int16_t len, void *user_data)
{
    kvkey_t k = (kvkey_t)(uintptr_t)user_data;
    if ((k != key) && (len > 0))
       kv_do_append_key(key, data, len);
    return KV_OK;
}

static void kv_do_gc(kvkey_t except_key)
{
    kv_do_backup(DB_FLASH_ADDRESS, DB_FLASH_ADDRESS_BACKUP);
    kv_reset();
    impl_do_kv_visit(kv_visitor_dump, (void *)(uintptr_t)except_key, DB_FLASH_ADDRESS_BACKUP, DB_FLASH_ADDRESS_BACKUP + BLOCK_SIZE);
}

static int kv_do_append_key(kvkey_t key, const void *data, int len)
{
    int aligned = (2 + len + 3) & ~0x3ul;
    const uint8_t *d = (const uint8_t *)data;
    uint8_t t[4] = {len, key, d[0], d[1]};

    if (kv_storage_tail + aligned > DB_FLASH_ADDR_END)
        kv_do_gc(key);
    if (kv_storage_tail + aligned > DB_FLASH_ADDR_END)
        return KV_ERR_OUT_OF_MEM;

    write_flash(kv_storage_tail, t, sizeof(t));
    kv_storage_tail += sizeof(t);

    aligned -= 4;
    if (aligned > 0)
    {
        write_flash(kv_storage_tail, d + 2, aligned);
        kv_storage_tail += aligned;
    }
    return KV_OK;
}

static void kv_do_remove_key(const kvkey_t key)
{
    kv_do_append_key(key, NULL, 0);
}

static void impl_kv_remove(const kvkey_t key)
{
#if USE_KEY_REMAP_FOR_PAIR
    kvkey_t new_key = key_remap_by_type(KEY_REMAP_TYPE_REMOVE, (kvkey_t)key);
    // log_printf("[KV] REMOVE: key=%d,new_key=%d\n", key, new_key);

    struct kv_cache_item * r = kv_search_cache(new_key);
    if (r)
    {
        GEN_OS->free(r->data);
        r->data = 0;
        r->key = (kvkey_t)-1;
        kv_do_remove_key(new_key);
        return;
    }

    struct kv_item *item = kv_search_flash(new_key, DB_FLASH_ADDRESS, DB_FLASH_ADDR_END);
    if (item)
        kv_do_remove_key(new_key);
#else
    log_printf("[KV]%s,key=%d\n", __func__, key);
    struct kv_cache_item * r = kv_search_cache(key);
    if (r)
    {
        GEN_OS->free(r->data);
        r->data = 0;
        r->key = (kvkey_t)-1;
        kv_do_remove_key(key);
        return;
    }

    struct kv_item *item = kv_search_flash(key, DB_FLASH_ADDRESS, DB_FLASH_ADDR_END);
    if (item)
        kv_do_remove_key(key);
#endif
    
}

static int impl_kv_put(const kvkey_t key, const uint8_t *data, int16_t len)
{
#if USE_KEY_REMAP_FOR_PAIR
    kvkey_t new_key = key_remap_by_type(KEY_REMAP_TYPE_PUT, (kvkey_t)key);
    log_printf("[KV] PUT: key=%d,new_key=%d\n", key, new_key);
    struct kv_cache_item *item = kv_search(new_key);
    if (item)
    {
        GEN_OS->free(item->data);
        item->data = NULL;
        item->key = (kvkey_t)-1;
    }

    return kv_do_append_key(new_key, data, len);
#else
    log_printf("[KV]%s,key=%d\n", __func__, key);
    struct kv_cache_item *item = kv_search(key);
    if (item)
    {
        GEN_OS->free(item->data);
        item->data = NULL;
        item->key = (kvkey_t)-1;
    }

    return kv_do_append_key(key, data, len);
#endif
}

static uint8_t *impl_kv_get(const kvkey_t key, int16_t *len)
{
#if USE_KEY_REMAP_FOR_PAIR
    kvkey_t new_key = key_remap_by_type(KEY_REMAP_TYPE_GET, (kvkey_t)key);
    // if(key != 0)
    //     log_printf("[KV] GET: key=%d,new_key=%d\n", key, new_key);

    struct kv_cache_item *item = kv_search(new_key);
    if (item)
    {
        if (len) *len = item->len;
        return item->data;
    }
    return NULL;
#else
    if(key != 0)
        log_printf("[KV]%s,key=%d\n", __func__, key);
    struct kv_cache_item *item = kv_search(key);
    if (item)
    {
        if (len) *len = item->len;
        return item->data;
    }
    return NULL;
#endif
}

static void impl_kv_value_modified_of_key(kvkey_t key)
{
#if USE_KEY_REMAP_FOR_PAIR
    kvkey_t new_key = key_remap_by_type(KEY_REMAP_TYPE_MODIFY, (kvkey_t)key);
    // log_printf("[KV] MODIFY: key=%d,new_key=%d\n", key, new_key);

    struct kv_cache_item * r = kv_search_cache(new_key);
    if (r)
        kv_do_append_key(new_key, r->data, r->len);
#else
    log_printf("[KV]%s,key=%d\n", __func__, key);
    struct kv_cache_item * r = kv_search_cache(key);
    if (r)
        kv_do_append_key(key, r->data, r->len);
#endif
}

static const kv_backend_t kv_backend =
{
    .kv_remove_all  = impl_kv_remove_all,
    .kv_remove      = impl_kv_remove,
    .kv_get         = impl_kv_get,
    .kv_put         = impl_kv_put,
    .kv_visit       = impl_kv_visit,
    .kv_value_modified_of_key  = impl_kv_value_modified_of_key,
};

void kv_impl_init(void)
{
    log_printf("[KV]%s\n", __func__);
    int r = kv_flash_repair(DB_FLASH_ADDRESS, DB_FLASH_ADDR_END);
    if (r)
    {
        r = kv_flash_repair(DB_FLASH_ADDRESS_BACKUP, DB_FLASH_ADDRESS_BACKUP + BLOCK_SIZE);
        if (r == 0)
        {
            kv_do_backup(DB_FLASH_ADDRESS_BACKUP, DB_FLASH_ADDRESS);
            kv_flash_repair(DB_FLASH_ADDRESS, DB_FLASH_ADDR_END);
        }
        else
            kv_reset();
    }

    kv_init_backend(&kv_backend);
}
