/**********************************
 * rfm69_config.h
**********************************/

#ifndef RFM69CONFIG_H_
#define RFM69CONFIG_H_

#include "rfm69.h"
#include "rfm69_reg.h"

static const rfm_reg_t CONFIG[][2] = {
    /* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RFM69_MODE_RX },
    /* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00 },
    /* 0x03 */ { REG_BITRATEMSB, 0x3E}, // 2000 bps
    /* 0x04 */ { REG_BITRATELSB, 0x80},
    /* 0x05 */ { REG_FDEVMSB, 0x00}, // 12000 hz (24000 hz shift)
    /* 0x06 */ { REG_FDEVLSB, 0xC5},

    /* 0x07 */{ REG_FRFMSB, 0xE4 }, // default value
    /* 0x08 */ { REG_FRFMID, 0xC0 }, // default value
    /* 0x09 */ { REG_FRFLSB, 0x12 },
    
    /* 0x0B */ { REG_AFCCTRL, RF_AFCCTRL_LOWBETA_OFF }, // AFC Offset On
    
    // PA Settings
    // +20dBm formula: Pout=-11+OutputPower[dBmW] (with PA1 and PA2)** and high power PA settings (section 3.3.7 in datasheet)
    // Without extra flags: Pout=-14+OutputPower[dBmW]
    /* 0x11 */ { REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | 0x1f},  // 10mW
    //{ RFM69_REG_11_PA_LEVEL, RF_PALEVEL_PA0_OFF | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON | 0x1f},// 50mW
    
    { REG_PARAMP, RF_PARAMP_500 }, // 500us PA ramp-up (1 bit)
    
    { REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 },
    
    { REG_LNA, RF_LNA_ZIN_50 }, // 50 ohm for matched antenna, 200 otherwise
    
    { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2}, // Rx Bandwidth: 128KHz
    
    { REG_AFCFEI, RF_AFCFEI_AFCAUTO_ON | RF_AFCFEI_AFCAUTOCLEAR_ON }, // Automatic AFC on, clear after each packet
    
    { REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 },
    { REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF }, // Switch off Clkout
    
    // { RFM69_REG_2D_PREAMBLE_LSB, RF_PREAMBLESIZE_LSB_VALUE } // default 3 preamble bytes 0xAAAAAA
    
    //{ RFM69_REG_2E_SYNC_CONFIG, RF_SYNC_OFF | RF_SYNC_FIFOFILL_MANUAL }, // Sync bytes off
    { REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0 },
    { REG_SYNCVALUE1, 0x2D },
    { REG_SYNCVALUE2, 0xAA },
    { REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF },
    { REG_PAYLOADLENGTH, RFM69_FIFO_SIZE }, // Full FIFO size for rx packet
    // { RFM69_REG_3B_AUTOMODES, RF_AUTOMODES_ENTER_FIFONOTEMPTY | RF_AUTOMODES_EXIT_PACKETSENT | RF_AUTOMODES_INTERMEDIATE_TRANSMITTER },
    { REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | 0x05 }, //TX on FIFO not empty
    { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, //RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
    { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // run DAGC continuously in RX mode, recommended default for AfcLowBetaOn=0
    // { RFM69_REG_71_TEST_AFC, 0x0E }, //14* 488hz = ~7KHz
    {255, 0}
  };

#endif /* __RFM69CONFIG_H__ */