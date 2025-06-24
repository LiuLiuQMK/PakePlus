#include <string.h>
#include "kv_storage.h"
#include "platform_api.h"
#include "btstack_util.h"
#include "le_device_db.h"
#include "app_storage.h"
#include "app_2g4.h"
#include "profile.h"


static const app_store_2g4_bond_state_t app_2g4_state_default = {
    .state = APP_2G4_BOND_STATE_NOT_BONDED,
    .addr = 0x1234567A,
};

//=======================================================================================
void print_addr(const uint8_t *addr) {
    log_printf("addr: %02X_%02X_%02X_%02X_%02X_%02X\r\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
}

void print_addr2(const uint8_t *addr) {
    log_printf(" %02X_%02X_%02X_%02X_%02X_%02X ", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
}


static void app_storage_remove_all(void){
    log_printf("remove all kv.\n");
    kv_remove_all();
}



void app_storage_2g4_state_init(void){
    int16_t len;
    
	log_printf("%s \n", __func__);
	app_store_2g4_bond_state_t * app_2g4_state = (app_store_2g4_bond_state_t *)kv_get(KV_KEY_2G4_BOND_STATE, &len);
    if(app_2g4_state == NULL || (len != sizeof(app_store_2g4_bond_state_t))){
        kv_put(KV_KEY_2G4_BOND_STATE, (const uint8_t *)&app_2g4_state_default, sizeof(app_store_2g4_bond_state_t)); //clear
    }
}


uint8_t app_storage_2g4_state_read(void){
    int16_t len;
    app_store_2g4_bond_state_t * app_2g4_state = (app_store_2g4_bond_state_t *)kv_get(KV_KEY_2G4_BOND_STATE, &len);
    if(app_2g4_state != NULL && len == sizeof(app_store_2g4_bond_state_t)){
        return app_2g4_state->state; // KB_MODE_2_4G=1, KB_MODE_BLE=2
    }
    return 0; // invalid state
}

uint32_t app_storage_2g4_addr_read(void){
    int16_t len;
    app_store_2g4_bond_state_t * app_2g4_state = (app_store_2g4_bond_state_t *)kv_get(KV_KEY_2G4_BOND_STATE, &len);
    if(app_2g4_state != NULL && len == sizeof(app_store_2g4_bond_state_t)){
        return app_2g4_state->addr; // KB_MODE_2_4G=1, KB_MODE_BLE=2
    }
    return 0; // WIRELESS_MODE_INVALID
}

//if addr is going to be written, it should be bonded
uint8_t app_storage_2g4_addr_write(uint32_t addr){
    int16_t len;
    app_store_2g4_bond_state_t * app_2g4_state = (app_store_2g4_bond_state_t *)kv_get(KV_KEY_2G4_BOND_STATE, &len);
    if(app_2g4_state != NULL && len == sizeof(app_store_2g4_bond_state_t)){
        app_2g4_state->state = APP_2G4_BOND_STATE_BONDED; //bonded
        app_2g4_state->addr = addr;
        kv_value_modified_of_key(KV_KEY_2G4_BOND_STATE);
//        kv_put(KV_KEY_2G4_BOND_STATE, (const uint8_t *)&app_2g4_state, sizeof(app_store_2g4_bond_state_t));
        return 0;
    }
    return 1;
}

void app_storage_2g4_state_reset(void)
{
    int16_t len;
    log_printf("%s\n", __func__);

    app_store_2g4_bond_state_t * app_2g4_state = (app_store_2g4_bond_state_t *)kv_get(KV_KEY_2G4_BOND_STATE, &len);
    if((app_2g4_state == NULL) || (app_2g4_state->state == APP_2G4_BOND_STATE_INVALID) || (app_2g4_state->state > APP_2G4_BOND_STATE_MAX))
    {
        log_printf("%s, info not exist.\n", __func__); 
        return;
    }

    kv_remove(KV_KEY_2G4_BOND_STATE); //清除对应通道的密钥信息；

    log_printf("2.4G bond info clear\n");

    // clear channel info.
    kv_put(KV_KEY_2G4_BOND_STATE, (const uint8_t *)&app_2g4_state_default, sizeof(app_store_2g4_bond_state_t));
    return;
}

//=======================================================================================
// *****************************************************
// kv storage test function
// *****************************************************
static int kv_visit_dump(const kvkey_t key, const uint8_t *data, const int16_t len, void *user_data){
    log_printf("k = %02X:\nv = ", key);
    printf_hexdump(data, len);
    log_printf("\n");
    return KV_OK;
}

void all_kv_dump(void){
    kv_visit(kv_visit_dump, NULL);
}

void print_all_device_db(void) {
    log_printf("[WARNING] YOU CAN GET ONLY ONE DB!\n");

    le_device_memory_db_iter_t iter;
    le_device_db_iter_init(&iter);
    while (le_device_db_iter_next(&iter)) {
        le_device_memory_db_t *cur = le_device_db_iter_cur(&iter);
        log_printf("[db]: key:%d peer_addr_type:%d, peer_addr: ", iter.key, cur->addr_type);
        print_addr2(cur->addr);
        log_printf(",irk: ");
        for(uint8_t i=0; i<sizeof(sm_key_t); i++){
            log_printf("%02X ", cur->irk[i]);
        }
        log_printf("\n");
    }
}


//=======================================================================================
void app_storage_init(void){
    /*初始化 KV_IMPL 架构地址*/
	kv_impl_init();
    
	/*获取2.4G里面的绑定信息以及2.4G的绑定地址*/
	app_storage_2g4_state_init();
}


/**
 * @brief 恢复出厂设置
 * @note 
 * 1. 蓝牙配对信息会被清除；
 * 2. dongle配对信息不会被清除；
 * 3. 常规灯效记忆状态会恢复；
 * 
 */
void app_storage_reset_to_default(void){
    app_storage_remove_all();
    all_kv_dump();
    print_all_device_db();
}

