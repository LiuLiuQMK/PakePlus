#include "string.h"
#include "IAP_Application.h"
#include "platform_api.h"
#include "bsp_usb_hid_iap.h"
#include "btstack_util.h"
#include "IAP_Transport.h"
#include "eflash.h"
#include "rom_tools.h"
#include "IAP_UserDef.h"
#include "IAP_Flash_WP.h"
#include "IAP_916.h"
#include "crc16.h"
#include "IAP_Bootloader.h"

#if USER_IAP_APP_ERROR_LOG_EN
#define IAP_APP_ERROR(...)	platform_printf(__VA_ARGS__)
#else
#define IAP_APP_ERROR(...)      
#endif

#if USER_IAP_APP_DEBUG_LOG_EN
#define IAP_APP_DEBUG(...)	platform_printf(__VA_ARGS__)
#else
#define IAP_APP_DEBUG(...)      
#endif

// =================================================================================================

#define IAP_UPGRADE_START_ADDR  (IAP_STORAGE_ADDR)
#define IAP_MAX_UPGRADE_SIZE    (IAP_STORAGE_SIZE)
#define FLASH_MIN_ERASE_UNIT    (EFLASH_UNIT_SIZE)

// INFO BEGIN.
static const char bUpgradeFlag[] = "INGCHIPS";
static const char bChipID[]      = USER_DEF_CHIP_ID;
static const char bItemStr[]     = USER_DEF_ITEM_STR;
static const char bHardware[]    = "V0.0.0";
static const char bSoftware[]    = "V0.0.0";
#define GET_STR_LEN(x)           (strlen(x))
// INFO END.

// =================================================================================================

// 传输层接收APP命令使用此buffer存数据
// 传输层发送APP命令从此buffer取数据
static uint8_t appBuffer[IAP_APP_MAX_BUFFER_SIZE];

static IAP_APP_ctl_t cmdCtl = {
    // APP CMD control
    .rspCmd = 0x00,
    .buffer = (uint8_t *)&appBuffer[0],
    .size   = 0,
    .payload_size = 0,
    .payload = NULL,

    // upgrade control
    .upgrade_state = IAP_UPGRADE_STATE_IDLE,
    .nextBlockNum = 0,
    .nextOffsetAddr = 0,

    // Record header information.
    .chk.type = IAP_CHECK_TYPE_CRC,
    .chk.val.CRC = 0x0000,
    .sblockSize = 0x0000,
    .tBlockNum = 0x0000,
    .upgrdType = IAP_UPGRADE_TYPE_APP_ONLY,
    .encrypt.en = 0,
    .encrypt.type = IAP_ENCRYPT_TYPE_AES128,
    .encrypt.key = {0},
    .encrypt.iv = {0},
};

// =================================================================================================
static uint8_t IAP_Flash_Erase(uint32_t size){

    if(size > IAP_MAX_UPGRADE_SIZE){
        IAP_APP_ERROR("[FLASH] error: erase size too large [%d]>[%d]\n", size, IAP_MAX_UPGRADE_SIZE);
        return IAP_FAIL;
    }

    uint16_t SEC_NUM = (size % FLASH_MIN_ERASE_UNIT)? (size / FLASH_MIN_ERASE_UNIT + 1) : (size / FLASH_MIN_ERASE_UNIT);
    for(uint16_t index = 0; index < SEC_NUM; index++){
        if(erase_flash_sector(IAP_UPGRADE_START_ADDR + (index * FLASH_MIN_ERASE_UNIT))){
            IAP_APP_ERROR("[FLASH] error: erase fail. addr=[0x%08X]\n", IAP_UPGRADE_START_ADDR + (index * FLASH_MIN_ERASE_UNIT));
            return IAP_FAIL;
        }
    }
    return IAP_OK;
}

static uint8_t IAP_Flash_Write(uint32_t offsetAddr, uint8_t *buffer, uint16_t size){
    int err = write_flash(IAP_UPGRADE_START_ADDR + offsetAddr, buffer, size);
    if(err){
        IAP_APP_ERROR("[FLASH] error: write [%d]\n", err);
        return IAP_FAIL;
    }
    return IAP_OK;
}

static uint8_t * IAP_Flash_StartAddr_Get(uint32_t offsetAddr){
    return (uint8_t *)(IAP_UPGRADE_START_ADDR + offsetAddr);
}

static uint8_t IAP_Flash_offsetAddr_valid_check(uint32_t offsetAddr, uint16_t read_size){
    if ( offsetAddr < IAP_UPGRADE_START_ADDR ){
        return IAP_INVALID;
    }
    if ( (offsetAddr+read_size) > (IAP_UPGRADE_START_ADDR+IAP_MAX_UPGRADE_SIZE) ){
        return IAP_INVALID;
    }
    return IAP_VALID;
}


static void* IAP_Reboot_Delay_Timeout_Callback(platform_us_timer_handle_t timer_handle, uint64_t time_us, void *param){
    platform_reset();
    return NULL;
}

static void* IAP_DoUpdate_Delay_Timeout_Callback(platform_us_timer_handle_t timer_handle, uint64_t time_us, void *param){
    IAP_APP_ERROR("[CMD] do update [0x%08X]\n", cmdCtl.loadAddress);
    
    /* 如果需要重新枚举的话，需要先关掉usb，再进行软复位 */
    bsp_usb_disable();
    
    for (uint32_t i = 0; i < 24000; i++)
        __NOP();
    
    /* program meta data */
    fota_update_block_t ota_meta;
    ota_meta.src = IAP_STORAGE_ADDR;
    ota_meta.dest = cmdCtl.loadAddress;
    ota_meta.size = cmdCtl.binSize;
    program_fota_metadata(1, &ota_meta);
    
    /* reset */
    platform_reset();
    return NULL;
}


// =================================================================================================
uint8_t * IAP_GetAppBuffer(void){
    return (uint8_t *)&appBuffer[0];
}

// 只是初始化指针，并不会改变appBuffer的值，所以不影响应用数据解析。
static void IAP_APP_PreparePayloadDataStart(void){
    cmdCtl.payload  = IAP_GetAppBuffer() + IAP_APP_SEND_CMD_PAYLOAD_OFFSET;
    cmdCtl.payload_size = 0;
}

// 会改变应用buffer的值，所以要在处理完应用数据后再进行
static uint8_t IAP_APP_AddPayloadData(uint8_t *data, uint16_t len){
    if( len==0 && data==NULL ){
        IAP_APP_ERROR("[CMD] error: params.\n");
        return IAP_FAIL;
    }
    
    if( (len + cmdCtl.payload_size + IAP_APP_SEND_CMD_MIN_SIZE) > IAP_APP_MAX_BUFFER_SIZE ){
        IAP_APP_ERROR("[CMD] error: too much data to send.\n");
        return IAP_FAIL;
    }

    memcpy(cmdCtl.payload, data, len);
    cmdCtl.payload += len;
    cmdCtl.payload_size += len;

#if USER_IAP_APP_DEBUG_LOG_EN
    IAP_APP_DEBUG("ADD[%d]: ", len);
    printf_hexdump(data, len);
#endif

    return IAP_OK;
}

// send with payload. 
static void IAP_APP_SendACK(uint8_t error){

    IAP_APP_ACK_t * ACK = (IAP_APP_ACK_t *)cmdCtl.buffer;
    ACK->CMD      = IAP_CMD_ACK;
    ACK->errCode  = error;
    ACK->rspCmd   = cmdCtl.rspCmd;
    ACK->length   = cmdCtl.payload_size;
    cmdCtl.size   = cmdCtl.payload_size + IAP_APP_SEND_CMD_MIN_SIZE;
    uint16_t *CRC = (uint16_t *)&cmdCtl.buffer[cmdCtl.size-2];
    *CRC  = getCRC16((uint8_t *)cmdCtl.buffer, cmdCtl.size-2);

#if USER_IAP_APP_DEBUG_LOG_EN
    IAP_APP_DEBUG("APP SEND ACK[%d]: ", cmdCtl.size);
    printf_hexdump(cmdCtl.buffer, cmdCtl.size);
#endif

    /* 
     * The current version's reply package is always one transport package, 
     * and the application package's ACK payload length is always 0.
     */
    if (ACK->length != 0) {
        return;
    }
    IAP_Transport_send_single_pack(1, 1, cmdCtl.buffer, cmdCtl.size);
}

// calc crc.
static uint8_t IsAppCrcValid(uint8_t *data, uint16_t len){
    uint16_t *CRC = (uint16_t *)&data[len-2];
    if(*CRC != getCRC16(data, len-2)){
        IAP_APP_ERROR("[CMD] error: CRC: Calc[0x%04X], Recv[0x%04X]\n", getCRC16(data, len-2), *CRC);
        return IAP_INVALID;
    } else {
        IAP_APP_DEBUG("[CMD] CRC:[0x%04X]\n", *CRC);
    }
    return IAP_VALID;
}

// =================================================================================================
static void PrintStr(char *comment, char *str, uint8_t len){
    uint8_t i;
    IAP_APP_DEBUG("%s", comment);
    for(i=0; i<len; i++){
        IAP_APP_DEBUG("%c", str[i]);
    }
    IAP_APP_DEBUG("\n");
}

static void PrintHeaderInfo(IAP_HeaderTypedef * iapHeader){
#if USER_IAP_APP_DEBUG_LOG_EN
    platform_printf("\n\n ------------------ HEADER INFO -------------------\n");
    PrintStr((char *)"upgradeFlag: ", (char *)&iapHeader->upgradeFlag, 8);

    IAP_APP_DEBUG("chip_id_len = %d\n", iapHeader->verInfo.chip_id.len);
    PrintStr((char *)"chip_id: ", (char *)&iapHeader->verInfo.chip_id.str, iapHeader->verInfo.chip_id.len);

    IAP_APP_DEBUG("item_info_len = %d\n", iapHeader->verInfo.item_info.len);
    PrintStr((char *)"item_info: ", (char *)&iapHeader->verInfo.item_info.str, iapHeader->verInfo.item_info.len);

    PrintStr((char *)"HW: ", (char *)&iapHeader->verInfo.HW, 6);
    PrintStr((char *)"SW: ", (char *)&iapHeader->verInfo.SW, 6);

    if (iapHeader->check.type == IAP_CHECK_TYPE_CRC){
        IAP_APP_DEBUG("check type = CRC\n");
        IAP_APP_DEBUG("check len = %d\n", iapHeader->check.len);
        IAP_APP_DEBUG("check data: 0x%04X\n", iapHeader->check.CRC); 
    } else {
        IAP_APP_DEBUG("check type = SUM\n");
        IAP_APP_DEBUG("check len = %d\n", iapHeader->check.len);
        IAP_APP_DEBUG("check data: 0x%04X\n", iapHeader->check.SUM);
    }

    IAP_APP_DEBUG("block size = 0x%04X\n", iapHeader->block.size);
    IAP_APP_DEBUG("block num  = 0x%04X\n", iapHeader->block.num);

    if(iapHeader->upgradeType == IAP_UPGRADE_TYPE_APP_ONLY)
        IAP_APP_DEBUG("upgradeType : APP Only.\n");
    else 
        IAP_APP_DEBUG("upgradeType : %d\n", iapHeader->upgradeType);

    IAP_APP_DEBUG("encrypt enable = %d\n", iapHeader->encrypt.enable);
    if (iapHeader->encrypt.enable){
        IAP_APP_DEBUG("encrypt type = %d\n", iapHeader->encrypt.type);
        IAP_APP_DEBUG("encrypt len = %d\n", iapHeader->encrypt.len);
        if(iapHeader->encrypt.type == IAP_ENCRYPT_TYPE_AES128){ //aes
            IAP_APP_DEBUG("encrypt type = AES128\n");
            IAP_APP_DEBUG("encrypt key: ");
            printf_hexdump(iapHeader->encrypt.key, iapHeader->encrypt.len/2);
            IAP_APP_DEBUG("encrypt iv: ");
            printf_hexdump(iapHeader->encrypt.iv, iapHeader->encrypt.len/2);
        } else {
            IAP_APP_DEBUG("encrypt type = XOR\n");
            IAP_APP_DEBUG("encrypt key: ");
            printf_hexdump(iapHeader->encrypt.key, iapHeader->encrypt.len);
        }
    }
#endif
}

static uint8_t isCheckHeaderValid(IAP_HeaderTypedef * header) {
    // check CRC.
    uint32_t headerDataSize = sizeof(IAP_HeaderTypedef) - sizeof(uint16_t);
    uint16_t headerCRC = getCRC16((uint8_t *)header, headerDataSize);
    // iap_switch_big_endian_u16(&allBinCRC);
    if(headerCRC != header->headerCRC){
        IAP_APP_ERROR("allBinSize: 0x%X\n", headerCRC);
        IAP_APP_ERROR("[WR] error: =====>CRC CHECK: calc[0x%04x], recv[0x%04x]\n", headerCRC, header->headerCRC);
        return IAP_INVALID;
    } else {
        IAP_APP_DEBUG("[WR] ----------->CRC OK:[0x%04x]\n", headerCRC);
    }
    return IAP_VALID;
}

static uint8_t IAP_APP_VersionFormatValidCheck(char *str){
    if(str[0] != 'V')    return IAP_INVALID;
    if(str[1] < '0' || str[1] > '9')    return IAP_INVALID;
    if(str[2] != '.')    return IAP_INVALID;
    if(str[3] < '0' || str[3] > '9')    return IAP_INVALID;
    if(str[4] != '.')    return IAP_INVALID;
    if(str[5] < '0' || str[5] > '9')    return IAP_INVALID;
    return IAP_VALID;
}

static uint8_t IsCheckInfoValid(IAP_CheckInfoTypedef * check){

#if USER_CFG_IAP_CHECK_TYPE_SUPPORT_CRC_EN
    if(check->type == IAP_CHECK_TYPE_CRC && check->len == 2){
        return IAP_VALID;
    }
#endif

#if USER_CFG_IAP_CHECK_TYPE_SUPPORT_SUM_EN
    if(check->type == IAP_CHECK_TYPE_SUM && check->len == 2){
        return IAP_VALID;
    }
#endif

    return IAP_INVALID; //unsupported check type.
}

static uint8_t IsUpgradeTypeValid(uint8_t upgradeType){

#if USER_CFG_IAP_UPGRADE_TYPE_SUPPORT_APP_ONLY_EN
    if( upgradeType == IAP_UPGRADE_TYPE_APP_ONLY ){
        return IAP_VALID;
    }
#endif

#if USER_CFG_IAP_UPGRADE_TYPE_SUPPORT_PLATFORM_APP_EN
    if( upgradeType == IAP_UPGRADE_TYPE_PLATFORM_APP ){
        return IAP_VALID;
    }
#endif

#if USER_CFG_IAP_UPGRADE_TYPE_SUPPORT_PLATFORM_BOOT_EN
    if( upgradeType == IAP_UPGRADE_TYPE_PLATFORM_BOOT ){
        return IAP_VALID;
    }
#endif

#if USER_CFG_IAP_UPGRADE_TYPE_SUPPORT_PLATFORM_BOOT_APP_EN
    if( upgradeType == IAP_UPGRADE_TYPE_PLAT_BOOT_APP ){
        return IAP_VALID;
    }
#endif

    return IAP_INVALID;
}

static uint8_t IsBlockInfoValid(IAP_BlockInfoTypedef * block, uint8_t upgradeType){

    // block size check.
    if(block->size < IAP_MIN_BLOCK_SIZE || block->size > IAP_MAX_BLOCK_SIZE){
        IAP_APP_ERROR("[HEADER] error: invalid blockSize! \n");
        return IAP_INVALID;
    }

    // check upgrade code size.
    if( (uint32_t)(block->size * block->num) > IAP_STORAGE_SIZE ){
        IAP_APP_ERROR("[HEADER] error: upgrade data too large! \n");
        return IAP_INVALID;
    }

    return IAP_VALID;
}

static uint8_t IsEncryptInfoValid(IAP_EncryptInfoTypedef * encrypt){

    // disable encrypt, return success.
    if (encrypt->enable == 0x00){
        return IAP_VALID;
    }

    if( encrypt->type == IAP_ENCRYPT_TYPE_XOR ){
#if USER_CFG_IAP_ENCRYPT_TYPE_SUPPORT_XOR_EN
        if (encrypt->len != IAP_ENCRYPT_LEN_XOR){
            return IAP_INVALID;
        }
#else
        return IAP_INVALID;
#endif
    }
    else if( encrypt->type == IAP_ENCRYPT_TYPE_AES128 ){
#if USER_CFG_IAP_ENCRYPT_TYPE_SUPPORT_AES_EN
        if (encrypt->len != IAP_ENCRYPT_LEN_AES128){
            return IAP_INVALID;
        }
#else
        return IAP_INVALID;
#endif
    }
    else{
        return IAP_INVALID; //unsupported encrypt type.
    }

    return IAP_VALID; // success.
}

static IAP_APP_ErrCode_t IAP_Header_Check(IAP_HeaderTypedef * header){
    
    // Check upgradeFlag
    if (memcmp(header->upgradeFlag, bUpgradeFlag, GET_STR_LEN(bUpgradeFlag)) != 0){
        IAP_APP_ERROR("[HEADER] error: upgradeFlag\n");
        return IAP_APP_ERR_HEADER_UPGRADE_FLAG;
    }
    
    // Check CRC
    if (!isCheckHeaderValid(header)) {
        IAP_APP_ERROR("[HEADER] error: headerCRC\n");
        return IAP_APP_ERR_HEADER_CRC;
    }

    // Check CHIP_ID
    if ( (GET_STR_LEN(bChipID) != header->verInfo.chip_id.len) || (memcmp(header->verInfo.chip_id.str, bChipID, GET_STR_LEN(bChipID)) != 0) ){
        IAP_APP_ERROR("[HEADER] error: chipID\n");
        return IAP_APP_ERR_HEADER_CHIP_ID;
    }

    // Check item information
    if ( (GET_STR_LEN(bItemStr) != header->verInfo.item_info.len) || (memcmp(header->verInfo.item_info.str, bItemStr, GET_STR_LEN(bItemStr)) != 0) ){
        IAP_APP_ERROR("[HEADER] error: itemInfo\n");
        return IAP_APP_ERR_HEADER_ITEM_IINFO;
    }

    // Check hardware version
    if (!IAP_APP_VersionFormatValidCheck((char *)header->verInfo.HW)){
        IAP_APP_ERROR("[HEADER] error: HW version.\n");
        return IAP_APP_ERR_HEADER_HARDWARE_VER;
    }

    // Check software version
    if (!IAP_APP_VersionFormatValidCheck((char *)header->verInfo.SW)){
        IAP_APP_ERROR("[HEADER] error: SW version.\n");
        return IAP_APP_ERR_HEADER_SOFTWARE_VER;
    }

    // Check CHECK_TYPE
    if (!IsCheckInfoValid(&header->check)){
        IAP_APP_ERROR("[HEADER] error: CHECK INFO.\n");
        return IAP_APP_ERR_HEADER_CHECK_INFO;
    }
    
    // Check UpgradeType
    if (!IsUpgradeTypeValid(header->upgradeType)){
        IAP_APP_ERROR("[HEADER] error: upgrade type.\n");
        return IAP_APP_ERR_HEADER_UPGRADE_TYPE;
    }

    // Check block info
    if (!IsBlockInfoValid(&header->block, header->upgradeType)){
        IAP_APP_ERROR("[HEADER] error: BLOCK INFO.\n");
        return IAP_APP_ERR_HEADER_BLOCK_INFO;
    }

    // Check encrypt information
    if (!IsEncryptInfoValid(&header->encrypt)){
        IAP_APP_ERROR("[HEADER] error: encrypt.\n");
        return IAP_APP_ERR_HEADER_ENCRYPT;
    }
    
    // Check LoadAddress
    if (header->loadAddress < PLATFORM_START_ADDR || FLASH_TOP_ADDR < header->loadAddress) {
        IAP_APP_ERROR("[HEADER] error: 0x%08X -> 0x%08X -> 0x%08X \n", PLATFORM_START_ADDR, header->loadAddress, FLASH_TOP_ADDR);
        IAP_APP_ERROR("[HEADER] error: loadAddress.\n");
        return IAP_APP_ERR_HEADER_LOAD_ADDRESS;
    }
    
    // Check BinSize
    if (IAP_STORAGE_SIZE < header->binSize) {
        IAP_APP_ERROR("[HEADER] error: upgrade data too large! \n");
        IAP_APP_ERROR("[HEADER] error: binSize.\n");
        return IAP_APP_ERR_HEADER_BIN_SIZE;
    }

    return IAP_APP_ERR_NONE;
}

static void IAP_Fill_header_info(IAP_HeaderTypedef * header){
    cmdCtl.chk.type = header->check.type;
    cmdCtl.chk.val.CRC = header->check.CRC;
    cmdCtl.sblockSize = header->block.size;
    cmdCtl.tBlockNum = header->block.num;
    cmdCtl.upgrdType = header->upgradeType;
    cmdCtl.encrypt.en = header->encrypt.enable;
    cmdCtl.encrypt.type = header->encrypt.type;
    cmdCtl.loadAddress = header->loadAddress;
    cmdCtl.binSize = header->binSize;
    memcpy(cmdCtl.encrypt.key, header->encrypt.key, 16);
    memcpy(cmdCtl.encrypt.iv, header->encrypt.iv, 16);
}

static void IAP_CtlInit(void){
    memset(&cmdCtl, 0, sizeof(IAP_APP_ctl_t));
    cmdCtl.buffer = (uint8_t *)&appBuffer[0];
}

static IAP_APP_ErrCode_t IAP_CMD_Start_handler(uint8_t * payload, uint16_t length){

    // check upgrade state.
    if(cmdCtl.upgrade_state == IAP_UPGRADE_STATE_FINAL){
        IAP_APP_ERROR("[WR] error: upgrade condition not satisfied.\n");
        return IAP_APP_ERR_STATE_NOT_SATISFIED;
    }
    
    IAP_APP_ErrCode_t errCode = IAP_APP_ERR_NONE;

    if (length != sizeof(IAP_HeaderTypedef)){
        IAP_APP_ERROR("[CMD] error: len: header[%d], Recv[%d]\n", sizeof(IAP_HeaderTypedef), length);
        return IAP_APP_ERR_LENGTH;
    }

    // get header struct.
    IAP_HeaderTypedef * header = (IAP_HeaderTypedef *)payload;

    // print header.
    PrintHeaderInfo(header);

    // check header.
    errCode = IAP_Header_Check(header);
    if(IAP_APP_ERR_NONE != errCode){
        IAP_APP_ERROR("[CMD] error: header check: 0x%02X\n", errCode);
        return errCode;
    }

    // store header information to flash.
    IAP_APP_DEBUG("TODO : Store header info to flash.\n");

    // Init stu
    IAP_CtlInit();

    // Erase flash.
    IAP_Flash_Erase((uint32_t)(header->block.size * header->block.num));

    // record upgrade info.
    IAP_Fill_header_info(header);

    // start upgrade.
    cmdCtl.upgrade_state = IAP_UPGRADE_STATE_BUSY;

    return errCode;
}

static void iap_switch_big_endian_u16(uint16_t *data){
    *data = (*data<<8)|(*data>>8);
}
static IAP_APP_ErrCode_t IAP_CMD_FlashWrite_handler(uint8_t * payload, uint16_t length){

    IAP_APP_ErrCode_t errCode = IAP_APP_ERR_NONE;

    // check upgrade state.
    if(cmdCtl.upgrade_state != IAP_UPGRADE_STATE_BUSY){
        IAP_APP_ERROR("[WR] error: upgrade condition not satisfied.\n");
        return IAP_APP_ERR_STATE_NOT_SATISFIED;
    }

    IAP_BlockWrite_t * fWrite = (IAP_BlockWrite_t *)payload;
    uint16_t currBlockSize = (length - 6);

    // last block num must be 0xFFFF
    if (cmdCtl.nextBlockNum + 1 == cmdCtl.tBlockNum){
        IAP_APP_DEBUG("[WR] This block is last block.\n");
        if (fWrite->blockNum != IAP_APP_LAST_BLOCK){
            IAP_APP_ERROR("[WR] error: block num, last block must be 0xFFFF, but recv[0x%04x]\n", fWrite->blockNum);
            return IAP_APP_ERR_BLOCK_NUM;
        }
    }
    // other block num should not be 0xFFFF
    else if(cmdCtl.nextBlockNum + 1 < cmdCtl.tBlockNum){
        if (fWrite->blockNum == IAP_APP_LAST_BLOCK){
            IAP_APP_ERROR("[WR] error: block num, middle block should not be 0xFFFF, must[0x%04x]\n", cmdCtl.nextBlockNum);
            return IAP_APP_ERR_BLOCK_NUM;
        }
    }
    // exceed max block num.
    else {
        IAP_APP_ERROR("[WR] error: block num, exceed max=[0x%04x]\n", cmdCtl.tBlockNum);
        return IAP_APP_ERR_BLOCK_NUM;
    }

    // last block.
    if (fWrite->blockNum == IAP_APP_LAST_BLOCK){
    // if (fWrite->blockNum == (cmdCtl.tBlockNum-1)){

        // check offset address
        if ( fWrite->offsetAddr != cmdCtl.nextOffsetAddr ){
            IAP_APP_ERROR("[WR] error: offsetAddr_last: must[0x%08x], Recv[0x%08x]\n", cmdCtl.nextOffsetAddr, fWrite->offsetAddr);
            return IAP_APP_ERR_WR_OFFSET_ADDR;
        }
        // check block data size
        if ( currBlockSize > cmdCtl.sblockSize ){
            IAP_APP_ERROR("[WR] error: blockSize_last: [%d] > [%d]\n", currBlockSize, cmdCtl.sblockSize);
            return IAP_APP_ERR_BLOCK_SIZE;
        }
        // store to flash.
        if (IAP_Flash_Write(fWrite->offsetAddr, fWrite->blockData, currBlockSize) != IAP_OK){
            IAP_APP_ERROR("[WR] error: flash write last: offsetAddr[0x%08x], size[%d]\n", fWrite->offsetAddr, currBlockSize);
            return IAP_APP_ERR_FLASH_OPERATE_FAIL;
        }

        // check CRC.
        uint32_t allBinSize = (cmdCtl.nextOffsetAddr + currBlockSize);
        uint8_t * pBinData   = (uint8_t *)IAP_Flash_StartAddr_Get(0);
        uint16_t allBinCRC = getCRC16(pBinData, allBinSize);
        // iap_switch_big_endian_u16(&allBinCRC);
        if(allBinCRC != cmdCtl.chk.val.CRC){
            IAP_APP_ERROR("allBinSize: 0x%X\n", allBinSize);
            IAP_APP_ERROR("[WR] error: =====>CRC CHECK: calc[0x%04x], recv[0x%04x]\n", allBinCRC, cmdCtl.chk.val.CRC);
            return IAP_APP_ERR_CRC;
        } else {
            IAP_APP_DEBUG("[WR] ----------->CRC OK:[0x%04x]\n", allBinCRC);
        }

        // // update allbinSize to BOOT PARAM flash.
        // IAP_Params_update_AllBinSize_ToFlash(allBinSize);

        // update control variable.
        cmdCtl.upgrade_state = IAP_UPGRADE_STATE_OVER;
    }
    // not last.
    else {

        // check block number
        if (fWrite->blockNum != cmdCtl.nextBlockNum) {
            IAP_APP_ERROR("[WR] error: blockNum: must[%d], Recv[%d]\n", cmdCtl.nextBlockNum, fWrite->blockNum);
            return IAP_APP_ERR_BLOCK_NUM;
        }

        // check offset address
        if ( fWrite->offsetAddr != cmdCtl.nextOffsetAddr ){
            IAP_APP_ERROR("[WR] error: offsetAddr: must[0x%08x], Recv[0x%08x]\n", cmdCtl.nextOffsetAddr, fWrite->offsetAddr);
            return IAP_APP_ERR_WR_OFFSET_ADDR;
        }

        // check block data size
        if ( currBlockSize != cmdCtl.sblockSize ){
            IAP_APP_ERROR("[WR] error: blockSize: must[%d], Recv[%d]\n", cmdCtl.sblockSize, currBlockSize);
            return IAP_APP_ERR_BLOCK_SIZE;
        }

        // store to flash.
        if (IAP_Flash_Write(fWrite->offsetAddr, fWrite->blockData, currBlockSize) != IAP_OK){
            IAP_APP_ERROR("[WR] error: flash write: offsetAddr[0x%08x], size[%d]\n", fWrite->offsetAddr, currBlockSize);
            return IAP_APP_ERR_FLASH_OPERATE_FAIL;
        }

        // update control variable.
        cmdCtl.nextBlockNum++;
        cmdCtl.nextOffsetAddr += currBlockSize;
    }

    return errCode;
}

static IAP_APP_ErrCode_t IAP_CMD_FlashRead_handler(uint8_t * payload, uint16_t length, uint8_t * outData, uint16_t * outLen){

    IAP_APP_ErrCode_t errCode = IAP_APP_ERR_NONE;

    // check upgrade state.
    if(cmdCtl.upgrade_state == IAP_UPGRADE_STATE_IDLE){
        IAP_APP_ERROR("[RD] error: upgrade condition not satisfied.\n");
        return IAP_APP_ERR_STATE_NOT_SATISFIED;
    }

    IAP_BlockRead_t * fRead = (IAP_BlockRead_t *)payload;

    // check offset address & read size.
    if(IAP_INVALID == IAP_Flash_offsetAddr_valid_check(fRead->offsetAddr, fRead->readBytes)){
        IAP_APP_ERROR("[RD] error: param.\n");
        return IAP_APP_ERR_PARAM;
    }

    // read data to payload area of sending buffer.
    outData = (uint8_t *)IAP_Flash_StartAddr_Get(fRead->offsetAddr);
    *outLen = fRead->readBytes;

    return errCode;
}

static IAP_APP_ErrCode_t IAP_CMD_Reboot_handler(uint8_t * payload, uint16_t length){

    IAP_APP_ErrCode_t errCode = IAP_APP_ERR_NONE;

    // if(cmdCtl.upgrade_state == IAP_UPGRADE_STATE_IDLE){
    //     IAP_APP_ERROR("[RB] error: upgrade condition not satisfied.\n");
    //     return IAP_APP_ERR_STATE_NOT_SATISFIED;
    // }

    if(length != 2){
        IAP_APP_ERROR("[RB] error: LEN=%d\n", length);
        return IAP_APP_ERR_LENGTH;
    }

    uint16_t * delay_ms = (uint16_t *)payload;

    if ((*delay_ms) > IAP_CMD_REBOOT_MAX_DELAY_MS){
        IAP_APP_ERROR("[RB] error: delay_ms=%d > %d\n", (*delay_ms), IAP_CMD_REBOOT_MAX_DELAY_MS);
        return IAP_APP_ERR_PARAM;
    }

	platform_create_us_timer(platform_get_us_time() + *delay_ms * 1000, IAP_Reboot_Delay_Timeout_Callback, NULL);

    return errCode;
}

static IAP_APP_ErrCode_t IAP_CMD_SwitchApp_handler(uint8_t * payload, uint16_t length){

    IAP_APP_ErrCode_t errCode = IAP_APP_ERR_NONE;

    // if(cmdCtl.upgrade_state == IAP_UPGRADE_STATE_IDLE){
    //     IAP_APP_ERROR("[SA] error: upgrade condition not satisfied.\n");
    //     return IAP_APP_ERR_STATE_NOT_SATISFIED;
    // }

    if(length != 2){
        IAP_APP_ERROR("[SA] error: LEN=%d\n", length);
        return IAP_APP_ERR_LENGTH;
    }

    uint16_t * delay_ms = (uint16_t *)payload;

    if ((*delay_ms) > IAP_CMD_SWITCH_APP_MAX_DELAY_MS){
        IAP_APP_ERROR("[SA] error: delay_ms=%d > %d\n", (*delay_ms), IAP_CMD_SWITCH_APP_MAX_DELAY_MS);
        return IAP_APP_ERR_PARAM;
    }

    cmdCtl.upgrade_state = IAP_UPGRADE_STATE_FINAL;
    
    platform_create_us_timer(platform_get_us_time() + *delay_ms * 1000, IAP_DoUpdate_Delay_Timeout_Callback, NULL);

    return errCode;
}



static IAP_APP_ErrCode_t IAP_APP_cmd_dispatch(uint8_t *data, uint16_t len){

    IAP_APP_ErrCode_t errCode = IAP_APP_ERR_NONE;

    // check param
    if(data == NULL || len < IAP_APP_RECV_CMD_MIN_SIZE){
        IAP_APP_ERROR("[CMD] error: param.\n");
        return IAP_APP_ERR_PARAM;
    }

    IAP_APP_cmd_t * APP_CMD = (IAP_APP_cmd_t *)data;

#if USER_IAP_APP_DEBUG_LOG_EN
    IAP_APP_DEBUG("CMD: 0x%02X\n", APP_CMD->CMD);
    IAP_APP_DEBUG("LEN: 0x%04X\n", APP_CMD->length);
    if(APP_CMD->length){
        IAP_APP_DEBUG("payload[%d]:\n", APP_CMD->length);
        printf_hexdump(APP_CMD->payload, APP_CMD->length);
    }
#endif

    // FOR RSP.
    cmdCtl.rspCmd = APP_CMD->CMD;

    // check CRC
    if (!IsAppCrcValid(data, len)){
        IAP_APP_ERROR("[CMD] error: CRC.\n");
        return IAP_APP_ERR_CRC;
    }

    // check payload LEN
    if (APP_CMD->length != (len-IAP_APP_RECV_CMD_MIN_SIZE)){
        IAP_APP_ERROR("[CMD] error: payload length: %d, %d\n", APP_CMD->length, (len-IAP_APP_RECV_CMD_MIN_SIZE));
        return IAP_APP_ERR_LENGTH;
    }

    // setup response payload data.
    IAP_APP_PreparePayloadDataStart();

    // Process cmd.
    switch(APP_CMD->CMD){
        case IAP_CMD_START:{
            IAP_APP_DEBUG("CMD: START\n");
            errCode = IAP_CMD_Start_handler(APP_CMD->payload, APP_CMD->length);
        }break;
        
        case IAP_CMD_FLASH_WRITE:{
            IAP_APP_DEBUG("CMD: WRITE\n");
            errCode = IAP_CMD_FlashWrite_handler(APP_CMD->payload, APP_CMD->length);
        }break;

        case IAP_CMD_FLASH_READ:{
            IAP_APP_DEBUG("CMD: READ\n");
            uint8_t *pReadData = NULL;
            uint16_t readLen = 0;
            errCode = IAP_CMD_FlashRead_handler(APP_CMD->payload, APP_CMD->length, pReadData, &readLen);
            if(errCode == IAP_APP_ERR_NONE){
                IAP_APP_AddPayloadData(pReadData, readLen);
            }
        }break;

        case IAP_CMD_REBOOT:{
            IAP_APP_DEBUG("CMD: REBOOT\n");
            errCode = IAP_CMD_Reboot_handler(APP_CMD->payload, APP_CMD->length);
        }break;

        case IAP_CMD_SWITCH_APP:{
            IAP_APP_DEBUG("CMD: SWTICH APP\n");
            errCode = IAP_CMD_SwitchApp_handler(APP_CMD->payload, APP_CMD->length);
        }break;

        default:
            IAP_APP_ERROR("[CMD] error: unsupported cmd: %d\n", APP_CMD->CMD);
            return IAP_APP_ERR_INVALID_CMD;
    }

    return errCode;
}

/*整个应用包数据处理完之后返回完整的ACK*/
static void IAP_APP_recv_cmd_callback(uint8_t *recvData, uint16_t recvLen){
#if USER_IAP_APP_DEBUG_LOG_EN
    platform_printf("\n\n========>>>>>APP RECV[%d]: ", recvLen); printf_hexdump(recvData, recvLen);
#endif
    
    IAP_APP_ErrCode_t errCode = IAP_APP_cmd_dispatch(recvData, recvLen);
    if(IAP_APP_ERR_NONE != errCode){
        IAP_APP_ERROR("[CMD] error: 0x%02X\n", errCode);
    }

    IAP_APP_SendACK(errCode);
}


void IAP_Application_Init(void){
    IAP_Transport_recv_cmd_callback_register(IAP_APP_recv_cmd_callback);
}


