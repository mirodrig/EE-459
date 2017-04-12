/*********************************
 * rfm69.c
*********************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#include "rfm69.h"
#include "rfm69_reg.h"
#include "../SPI/spi.h"
#include "../serial_test/serial.h"

rfm_status_t rf69_init(void){
	const uint8_t CONFIG[][2] = {
   		/* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RFM69_MODE_RX },
    	/* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00 },
    	/* 0x03 */ { REG_BITRATEMSB, 0x3E}, // 2000 bps
    	/* 0x04 */ { REG_BITRATELSB, 0x80},
   		/* 0x05 */ { REG_FDEVMSB, 0x00}, // 12000 hz (24000 hz shift)
   		/* 0x06 */ { REG_FDEVLSB, 0xC5},
   		/* 0x07 */ { REG_FRFMSB, 0xE4 }, // default value
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
 	  	//RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
 	   { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF },
 	    // run DAGC continuously in RX mode, recommended default for AfcLowBetaOn=0
    	{ REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 },
    	// { RFM69_REG_71_TEST_AFC, 0x0E }, //14* 488hz = ~7KHz
		{255, 0}
	};
	uint8_t i; // for looping

	/* Call the user setup function to configure the SPI peripheral */
    if(spi_init() != RFM_OK){
        //serial_outs("\rSPI failed to setup\n\r");
        return RFM_FAIL;
    }
	/* Zero version number, RFM probably not connected/functioning */
    uint8_t res = _rf69_read(REG_VERSION);
    if (res != 0x24){
        return RFM_FAIL;
    }
    // serial_outs("\rpassed setup\n\r");
    /* Set up device */
    for (i = 0; CONFIG[i][0] != 255; i++){
    	_rf69_write(CONFIG[i][0], CONFIG[i][1]);
    }
    
    rf69_set_mode(RFM69_MODE_STDBY); // TODO: determine if this should be SLEEP or STDBY

    //char buf[30];
    //snprintf(buf, 30, "\r0x%02X mode\n\r", radio._mode); // supposed to be 0x04
    //serial_outs(buf);

    return RFM_OK;
}

uint8_t _rf69_read(uint8_t addr){
	cli();
	//char buffer[50];
	spi_ss_assert(); // set SS pin to LOW
	spi_transfer(addr & 0x7F);
	uint8_t regVal = spi_transfer(0);
	spi_ss_deassert(); // set SS pin to HIGH
	//snprintf(buffer, 50, "\rRead 0x%02X and recv 0x%02X\n\r", addr, regVal);
	//serial_outs(buffer);
	sei();
	return regVal;
}

void _rf69_write(uint8_t addr, uint8_t value){
	cli();
	//char buffer[50];
	spi_ss_assert();
	spi_transfer(addr | 0x80);
	spi_transfer(value);
	spi_ss_deassert();
	//snprintf(buffer, 50, "\rWrite 0x%02X with 0x%02X\n\r", addr, value);
	//serial_outs(buffer);
	sei();
}

void rf69_set_mode(uint8_t newMode){
    switch(newMode){
	  case RFM69_MODE_TX:
	   _rf69_write(REG_OPMODE, (_rf69_read(REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER);
	   break;
	  case RFM69_MODE_RX:
	   _rf69_write(REG_OPMODE, (_rf69_read(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);
	   break;
	  case RFM69_MODE_STDBY:
	   _rf69_write(REG_OPMODE, (_rf69_read(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
	   break;
	  case RFM69_MODE_SLEEP:
	   _rf69_write(REG_OPMODE, (_rf69_read(REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP);
	   break;
	  default:
	   return;
	}
}

/*rfm_status_t rf69_set_mode(const uint8_t newMode){
    uint8_t res;
    _rf69_read(REG_OPMODE, &res);
    _rf69_write(REG_OPMODE, (res & 0xE3) | newMode);
    _mode = newMode;
    return RFM_OK;
}*/

/*void receiveBegin(struct rfm69 *radio){
	radio->DATALEN = 0; // DATALEN
	radio->senderID = 0; // SENDERID
	radio->targetID = 0; // TARGETID
	radio->PAYLOADLEN = 0; // PAYLOADLENGTH
	radio->ACK_REQ = 0; // ACK_REQUESTED
	radio->ACK_RECV = 0; // ACK_RECEIVED
	radio->RSSI = 0; // RSSI
	// determine if registers are ready
	if(_rf69_read(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY){ // 0x04
		// avoid RX deadlocks
		_rf69_write(REG_PACKETCONFIG2, (_rf69_read(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART);
	}
	_rf69_write(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01); // set DIO0 to PAYLOADREADY in recv mode
	rf69_set_mode(radio, RFM69_MODE_RX);
}*/

/*bool receiveDone(struct rfm69 *radio){
	cli(); // no interrupts
	if(radio->_mode == RFM69_MODE_STDBY){
		return true;
	}
	else if(radio->_mode == RFM69_MODE_RX){
		sei();
		return false;
	}
	receiveBegin(radio);
	return false;
}*/

/*rfm_status_t rf69_receive(uint8_t* buf, uint8_t* len, int16_t* lastrssi, bool* rfm_packet_wait){
    uint8_t res;
    // set the mode of the transceiver to RX
    if(_mode != RFM69_MODE_RX){
        rf69_set_mode(RFM69_MODE_RX);
    }
    // Check IRQ register for payloadready flag
    // (indicates RXed packet waiting in FIFO)
    _rf69_read(REG_IRQFLAGS2, &res);

    if (res & RF_IRQFLAGS2_PAYLOADREADY){
        // Get packet length from first byte of FIFO
        _rf69_read(REG_FIFO, len);
        *len += 1;
        // Read FIFO into our Buffer
        _rf69_burst_read(REG_FIFO, buf, RFM69_FIFO_SIZE);
        // Read RSSI register (should be of the packet? - TEST THIS)
        _rf69_read(REG_RSSIVALUE, &res);
        *lastrssi = -(res/2);
        // Clear the radio FIFO (found in HopeRF demo code)
        _rf69_clear_fifo();
        *rfm_packet_wait = true;
        return RFM_OK;
    }
    *rfm_packet_wait = false;
    return RFM_OK;
}*/

/*bool canSend(struct rfm69 *radio){
	// if payload is ready and signal strength is enough
	if(radio->_mode == RFM69_MODE_TX && PAYLOADLENGTH == 0 && readRSSI() < CSMA_LIMIT){
		rf69_set_mode(radio, RFM69_MODE_STDBY);
		return true;
	}
	return false;
}*/

/*void rf69_sendFrame(uint8_t addr, const void* buffer, uint8_t bufferSize, bool requestACK, bool sendACK){
//void rf69_sendFrame(struct rfm69* radio, uint8_t addr, const void* buffer, uint8_t bufferSize){
	uint8_t i;

	rf69_set_mode(RFM69_MODE_STDBY); // turn off the receiver
	while((_rf69_read(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for MODEREADY
	_rf69_write(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00); // DIO0 is "packet sent"
	if(bufferSize > RFM69_MAX_MESSAGE_LEN){
		bufferSize = RFM69_MAX_MESSAGE_LEN;
	}
	// control byte
	//uint8_t CTLbyte = 0x00;
	//if(sendACK){
		//CTLbyte = RFM69_CTL_SENDACK;
	//}
	spi_ss_assert();
	spi_transfer(REG_FIFO | 0x80);
	spi_transfer(bufferSize + 3);
	spi_transfer(addr);

	for(i=0; i<bufferSize; i++){
		spi_transfer(((uint8_t*)buffer)[i]);
	}
	spi_ss_deassert();

	// set to transmit mode
	rf69_set_mode(RFM69_MODE_TX);
	// wait for MODEREADY
	while((_rf69_read(REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT) == 0x00);
	rf69_set_mode(RFM69_MODE_STDBY);
}*/

//void rf69_send(uint8_t addr, const void* buffer, uint8_t bufferSize, bool requestACK){
/*void rf69_send(char *data, struct rfm69 *radio){
	uint8_t dataLength = sizeof(data)/sizeof(uint8_t);
    if(dataLength > RFM69_MAX_MESSAGE_LEN){
        return;
    }
    cli();
    rf69_set_mode(RFM69_MODE_STDBY);
    

    // avoid RX deadlock
	_rf69_write(REG_PACKETCONFIG2, (_rf69_read(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART);
	while(!canSend()){
		//receiveDone();
	}
	rf69_sendFrame(addr, buffer, bufferSize, requestACK, false);
}*/

/*void rf69_send(char *data){
    char length = sizeof(data);
    
    // if length exceeds maximum data length
    if(length > RFM69_MAX_MESSAGE_LEN){
        return;
    }
    cli(); // turn off interrupts

    // rf69_set_mode(RFM69_MODE_TX); // set transmitter
    rf69_set_mode(RFM69_MODE_STDBY); // set the standby mode
    // wait for MODEREADY
    while((_rf69_read(REG_IRQFLAGS1) & 0x80) == 0x00){} // stuck here

    spi_ss_assert(); // set SS pin to LOW
    //char message[2] = {REG_FIFO | 0x80}; // 0x80 being the write mask
    spi_transfer(REG_FIFO | 0x80);

    while(length--){
        spi_transfer(*data++);
    }
    char buf[25];
    snprintf(buf, 25, "\rsent %s\n\r", data);
    serial_outs(buf);

    spi_ss_deassert();
    sei();
    rf69_set_mode(RFM69_MODE_TX); // set to Tx mode
    // wait until the mode of operation is set to idle
    while((_rf69_read(REG_OPMODE) & 0x1C) != 0x04);
}*/

/*rfm_status_t rf69_send(const uint8_t* data, uint8_t len, const uint8_t power){
    uint8_t oldMode, res;
    uint8_t paLevel;
    // power is TX Power in dBmW (valid values are 2dBmW-20dBmW)
    if (power < 2 || power > 20){
        // Could be dangerous, so let's check this
        serial_outs("\rERROR: power failure\n\r");
        return RFM_FAIL;
    }
    oldMode = _mode;
    // Start transmitter
    rf69_set_mode(RFM69_MODE_TX); // 0x01 written with 0x0C
    serial_outs("\rRFM69 in Tx mode\n\r");
    // Set up PA
    if (power <= 17) {
        // Set PA Level
        paLevel = power + 28;
        // enables PA0, disables PA1 and PA2. The PA0 output is on pin RFIO.
        // This provides power to the LNA of the transmitter.
        _rf69_write(REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF |
            RF_PALEVEL_PA2_OFF | paLevel);        
    }
    else {
        // Disable Over Current Protection
        _rf69_write(REG_OCP, RF_OCP_OFF);
        // Enable High Power Registers
        _rf69_write(REG_TESTPA1, 0x5D); // set to 20 dBm mode
        _rf69_write(REG_TESTPA2, 0x7C); // set to 20 dBm mode
        // Set PA Level
        paLevel = power + 11;
        // enables both PA1 and PA2 while disabling PA0. This delivers
        // 20 dBm to the antenna.
        _rf69_write(REG_PALEVEL, RF_PALEVEL_PA0_OFF |
            RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON | paLevel);
    }
    // Wait for PA ramp-up
    res = 0;
    // wait until the TXREADY interrupt is sent
    while (!(res & RF_IRQFLAGS1_TXREADY)){
        //serial_outs("\r\nTransmitter is ready to transmit\n\r");
        _rf69_read(REG_IRQFLAGS1, &res);
    }
    // Throw Buffer into FIFO, packet transmission will start 
    //  automatically
    _rf69_fifo_write(data, len);

    res = 0;
    // wait until the complete packet has been sent. Set to HIGH
    // when the last bit of the shift register has been sent
    while (!(res & RF_IRQFLAGS2_PACKETSENT))
        _rf69_read(REG_IRQFLAGS2, &res);

    // Return Transceiver to original mode
    rf69_set_mode(oldMode);

    // If we were in high power, switch off High Power Registers 
    if (power > 17) {
        // Disable High Power Registers. Set to normal setting and
        // for Rx mode.
        _rf69_write(REG_TESTPA1, 0x55);
        _rf69_write(REG_TESTPA2, 0x70);
        // Enable Over Current Protection
        _rf69_write(REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95);
    }
    return RFM_OK;
}*/

/*int16_t readRSSI(){
	uint16_t rssi = 0;
	//if(forceTrigger){
		// RSSI trigger not needed if in continuous mode
		//_rf69_write(REG_RSSICONFIG, RF_RSSI_START);
		// wait for RSSI ready
		//while((_rf69_read(REG_RSSICONFIG) & RF_RSSI_DONE) == 0x00);
	//}
	rssi = -_rf69_read(REG_RSSIVALUE);
	rssi >>= 1; // perform bit shifting
	return rssi;
}*/

/*void setNodeAddress(uint8_t addr){
    radio._addr = addr;
    _rf69_write(REG_NODEADRS, radio._addr);
}

void setNetwork(uint8_t networkID){
    _rf69_write(REG_SYNCVALUE2, networkID);
}*/

/*uint8_t rf69_fifo_read()
static rfm_status_t _rf69_fifo_write(const uint8_t* src, uint8_t len){
    uint8_t dummy;
    spi_ss_assert();
    // Send the start address with the write mask on
    spi_exchange_single(REG_FIFO | RFM69_SPI_WRITE_MASK, &dummy);
    // First byte is packet length
    spi_exchange_single(len, &dummy);
    // Then write the packet
    while (len--)
        spi_exchange_single(*src++, &dummy);
    spi_ss_deassert();
    return RFM_OK;
}*/

/*void rf69_interruptHandler(){
	if(_mode == RFM69_MODE_TX && (_rf69_read(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)){
		rf69_set_mode(RFM69_MODE_STDBY);
		spi_ss_assert();
		spi_transfer(REG_FIFO & 0x7F);
		PAYLOADLENGTH = spi_transfer(0);
		PAYLOADLENGTH = PAYLOADLENGTH > 66 ? 66 : PAYLOADLENGTH; // precaution (from library)
		TARGETID = spi_transfer(0);

		DATALEN - PAYLOADLENGTH - 3;
		SENDERID = spi_transfer(0);
		uint8_t CTLbyte = spi_transfer(0);

		ACK_RECEIVED = CTLbyte & RFM69_CTL_SENDACK;
		ACK_REQUESTED = CTLbyte & RFM69_CTL_REQACK;

		uint8_t i;
		for(i=0; i<DATALEN; i++){
			DATA[i] = spi_transfer(0);
		}
		if(DATALEN < RFM69_MAX_MESSAGE_LEN){
			DATA[DATALEN] = 0;
		}
		spi_ss_deassert();
		rf69_set_mode(RFM69_MODE_RX);
	}
	RSSI = readRSSI();
}*/

//rfm_status_t rf69_interruptHandler(uint8_t currentMode){
/*uint8_t rf69_interruptHandler(uint8_t* mode){
    uint8_t res;
    res = _rf69_read(REG_IRQFLAGS2);
    // serial_outs("\rinterrupt handler\n\r");
    if(*mode == RFM69_MODE_TX && (res & 0x04)){
        serial_outs("\rpackets are sent\r\n");
        return RFM_OK;
    }
    //else if(currentMode == RFM69_MODE_TX){
    else if(*mode == RFM69_MODE_TX){
        rf69_set_mode(RFM69_MODE_STDBY);
        return RFM_OK;
    }
    return RFM_FAIL;
}*/

rfm_status_t spi_init(void){
   /* Set up the SPI IO as appropriate */
	SPI_DDR |= SPI_SS | SPI_MOSI | SPI_SCK;
	SPI_DDR &= ~(SPI_MISO);
	SPI_PORT |= SPI_SS; // set SS HIGH
    /* SPI should be mode (0,0), MSB first, double clock rate. Change the
    values of the SPI registers */
	SPCR &= ~(_BV(CPOL) | _BV(CPHA) | _BV(DORD));
	SPSR |= _BV(SPI2X);
	SPCR |= _BV(MSTR); // become master
	SPCR |= _BV(SPE); // enable the SPI peripheral
	return RFM_OK;
}

rfm_status_t spi_exchange_single(const uint8_t out, uint8_t* in){
    SPDR = out; // load SPI data register with first parameter
    /* SPIF - interrupt indicating that serial transfer is complete. The SPIF
    will be loaded onto the SPSR register. */
    while(!(SPSR & (1 << SPIF)));
    *in = SPDR; // SPI data register is now filled with second parameter
    return RFM_OK;
}

rfm_status_t spi_ss_assert(void){
    SPI_PORT &= ~(SPI_SS);
    return RFM_OK;
}

rfm_status_t spi_ss_deassert(void){
    SPI_PORT |= (SPI_SS);
    return RFM_OK;
}