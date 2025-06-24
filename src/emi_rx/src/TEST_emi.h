#ifndef __TEST_EMI_H
#define __TEST_EMI_H

#include "stdint.h"

/**
 * @brief  reset the LLE hardware.
 */
void ing2p4g_rst(void);

/**
 * @brief Start a tx with payload for emi test.
 * @param ChID               acutal channel= 2400+ChID; 
 *                           range:0-80
 * @param PHY                0b00: 1M PHY ,0b01: 2M PHY
 * @param PktLen             payload length, range: 0-255
 * @param PktType            payload type:
 *                           0x0: PRBS9
 *                           0x1: Repeated 11110000
 *                           0x2: Repeated 10101010
 *                           0x3: PRBS15
 *                           0x4: Repeated 11111111
 *                           0x5:Repeated  00000000
 *                           0x6:Repeated 00001111
 *                           0x7:Repeated 01010101
 *                           0x8-0xF:reserved
 * @param TXPOW              Tx power, range:0-63
 */
void RFTestTx(uint8_t ChID, uint8_t PHY, uint8_t PktLen, uint8_t PktType, uint8_t TXPOW);

/**
 * @brief  Stop tx for emi test.
 */
void LLE_TX_STOP(void);


#endif
