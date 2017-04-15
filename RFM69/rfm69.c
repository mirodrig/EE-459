/***********************************************
 * author: Marlin Rodriguez
 * rfm69.c
***********************************************/
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#include "rfm69.h"
#include <stdio.h>
#include "../SPI/spi.h"
#include "rfm69_reg.h"

// #include "serial.h"

//radio reset pin
#define RF_reset_pin 11 

// The crystal oscillator frequency of the RF69 module
#define RH_RF69_FXOSC 32000000.0

// The Frequency Synthesizer step = RH_RF69_FXOSC / 2^^19
#define RH_RF69_FSTEP  (RH_RF69_FXOSC / 524288)

#define Max_Message_length 60

void RFM_init(){
	// manually reset the transceiver
	DDRC |= DDC1;
	PORTC |= (1 << PC1); // pull to HIGH
	_delay_ms(100);
	PORTC &= ~(1 << PC1); // set to LOW
	_delay_ms(100);

	// Configure important RH_RF69 registers
	// defaults to fixed packet format 
    // Here we set up the standard packet format for use by the RH_RF69 library:
    // 4 bytes preamble
    // 2 SYNC words 2d, d4
    // 2 CRC CCITT octets computed on the header, length and data (this in the modem config data)
    // 0 to 60 bytes data
    // RSSI Threshold -114dBm
    // We dont use the RH_RF69s address filtering: instead we prepend our own headers to the beginning
    // of the RH_RF69 payload

	// RH_RF69_REG_3C_FIFOTHRESH : 0x3c == Tx start 
	// RH_RF69_FIFOTHRESH_TXSTARTCONDITION_NOTEMPTY : 0x80  setting threshold for fif0 to 0x8f as recommended 

	rfm_write(REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | 0x0F); 

    rfm_write(REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0);

    rfm_write(REG_TESTPA1, RH_RF69_TESTPA1_NORMAL);
    rfm_write(REG_TESTPA2, RH_RF69_TESTPA2_NORMAL);

    char syncwords[] = {0x2d, 0x4d};
    RFM_setSyncWords(syncwords);
    RHFM_setPreambleLength(4); 
    RFM_setFrequency(434.0);
}

char RFM_Read_FIFO(char* buffer, char* currentMode){
	cli(); // turn off interrupts
	RFM_setMode(currentMode,0); // set to idle
	
	DDRB |= 1 << DDB2;
	PORTB &= ~(1 << PB2); // set CS pin to LOW
	
	spi_transfer(REG_FIFO);

	char payload = spi_transfer(0x00); //get length of bytes 
	uint8_t i;

	if(payload != 0){
		for(i = 0; i <payload; i++){
			buffer[i] = spi_transfer(0x00);
		}
	}
	PORTB |= (1 << PB2);

	sei(); 
	return payload; // the length of the message 
}

// complete
char RFM_recieve(struct RFM69* radio){
	if (radio->receiveDataFlag){
		// serial_outputString("got");
		radio->receiveDataFlag = 0;
		// char buf[60];
		radio->buffer_length = RFM_Read_FIFO(radio->buffer, &(radio->currentMode)); //getting the length of the message 
		RFM_setMode(&(radio->currentMode),1); // set mode to RX 
		// serial_outputString(buf);
		return 1; // Note that the mode will be in idle at the end 
	}
	else {
		return 0; 
	}
}

/* Call this function to send data via the radio communication 
	- the mode will change:: ->idle -> TX 
	- When the package is sent, an interrupt will happen on pin D0 (G0)
	- Parameters : 	
		- data :: pointer to the data that is going to be sent 
		- currentMode :: currentMode of the radio, is in radio struct 
*/
// complete
void RFM_send(char* data, char* currentMode, char dataSize){
	if(dataSize > Max_Message_length){
		return; 
	}
	cli(); 

	RFM_setMode(currentMode,0); // set mode to idle 
	
	// wait for MODEREADY interrupt
	while ( (rfm_read(REG_IRQFLAGS1) & 0x80) == 0x00);

	DDRB |= 1 << DDB2;
	PORTB &= ~(1 << PB2);

	char message[2] = {REG_FIFO | 0x80, dataSize};
	
	spi_multiWrite(message, 2);
	//serial_outs("\rwrote to SPI\n\r");
	uint8_t i;
	for(i=0; i<dataSize; i++){
		spi_transfer(data[i]);
		serial_out(data[i]);
	}
	//serial_outs("\rstep1\n\r");
	PORTB |= (1 << PB2);

	sei();

	RFM_setMode(currentMode,2); //TX 

	// wait until the PACKETSENT interrupt is set. If so, set the operating
	// mode to STDBY mode.
	while((rfm_read(REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT) != RF_IRQFLAGS2_PACKETSENT);
	//RFM_setMode(currentMode, 0);
	//serial_outs("\rpackets sent\n\r");
}

// complete
char RFM_interruptHandler(struct RFM69 radio, char* currentMode) {
	// serial_outputString("interrupt handeler");
	//char buf[50];
	//snprintf(buf, 50, "\rflag1: 0x%02X, flag2: 0x%02X\n\r", rfm_read(REG_IRQFLAGS1), rfm_read(REG_IRQFLAGS2));
	//serial_outs(buf);

	// if we are in RXMODE and PAYLOAD is ready
	if (*currentMode == 1 && (rfm_read(REG_IRQFLAGS2) & 0x04)){
		//serial_outs("\rnew data\n\r");
		return 1;
	}
	// transmit
	else if (*currentMode == 1){
		serial_outs("\rset to idle\n\r");
		RFM_setMode(&radio.currentMode,0);
		return 1;
	}
	else{
		//serial_outs("\rreturn zero\n\r");
		return 0;
	}
}

// complete
void RFM_spiConfig(){
	spi_init();
	cli();

	SPCR = (SPCR & ~0x0C) | 0x00;
	
	SPCR &= ~(1 << DORD); // transmit MSB first
	
	SPCR = (SPCR & ~0x03) | (0x00 & 0x03); // this is just changing the first two bits of SPCR 
    SPSR = (SPSR & ~0x01) | ((0x00 >> 2) & 0x01); // this is just chaning the 2X bit 
    
    sei();
} 

void rfm_write(char addr, char value){
	cli();
	spi_ss_assert(); // set SS pin to LOW
	char message[2] = {addr | 0x80, value};
	addr |= 0x80; // put a 1 in MSB
	spi_multiWrite(message, 2);
	spi_ss_deassert(); // set SS pin to HIGH
	sei(); // enable interrupts
}

char rfm_read(char addr){
	cli();
	DDRB |= DDB2;
	PORTB &= ~(1 << PB2); // set SS pin to LOW
	//spi_ss_assert(); // set SS pin to LOW
	addr &= ~(0x80); // set 0 as MSB
	spi_transfer(addr);

	char regVal = spi_transfer(0x00);
	PORTB |= (1 << PB2); // set SS pin to HIGH
	//spi_ss_deassert(); // set SS pin to HIGH
	
	// char buf[50];
	// snprintf(buf, 50, "\rsrc: 0x%02X, val: 0x%02X\n\r", addr, regVal);
	// serial_outs(buf);

	sei();
	return regVal;
}

/*char RFM_readReg(char address){
	cli(); 
	digitalWrite(cs, 0);
	address &= ~RH_SPI_WRITE_MASK; // putting 0 in MSB
// 	char message[] = {address, 0x00};
	SPI_transfer(address); 
	char new = SPI_transfer(0x00); 
// 	SPI_multiTransfer(message,2); 
	digitalWrite(cs, 1);
	sei(); 
	return new ; 
}*/

// complete
void RFM_setSyncWords(char* syncwords){
	// restricting number of sync words to 2 for now 
	// getting the current syncConfig
	// default number of sync words is 2 
	// syncwords is on by default  

	// currently not changing any of the default values 
	// char synConfig = RFM_read(RH_RF69_REG_2E_SYNCCONFIG,cs) ; 

	// setting the sync words 
	rfm_write(0x2F, syncwords[0]);
	rfm_write(0x30, syncwords[1]);
	//RFM_writeReg(0x2f,syncwords[0]);
	//RFM_writeReg(0x30,syncwords[1]);
}

// complete
void RHFM_setPreambleLength(uint16_t bytes){
    rfm_write(REG_PREAMBLEMSB, bytes >> 8);
    rfm_write(REG_PREAMBLELSB, bytes & 0xFF);

    // RFM_writeReg(RH_RF69_REG_2C_PREAMBLEMSB, bytes >> 8);
    // RFM_writeReg(RH_RF69_REG_2D_PREAMBLELSB, bytes & 0xff);
}

// complete
void RFM_setFrequency(float freq){
	uint32_t frf = (uint32_t)((freq * 1000000.0) / RH_RF69_FSTEP);
	rfm_write(REG_FRFMSB, (frf >> 16) & 0xFF);
	rfm_write(REG_FRFMID, (frf >> 8) & 0xFF);
	rfm_write(REG_FRFLSB, frf & 0xFF);

	// RFM_writeReg(RH_RF69_REG_07_FRFMSB, (frf >> 16) & 0xff);
    // RFM_writeReg(RH_RF69_REG_08_FRFMID, (frf >> 8) & 0xff);
    // RFM_writeReg(RH_RF69_REG_09_FRFLSB, frf & 0xff);
}

/*	Modes of operation 
000 → Sleep mode (SLEEP) 			   :: RH_RF69_OPMODE_MODE_SLEEP  
001 → Standby mode (STDBY) 			   :: RH_RF69_OPMODE_MODE_STDBY  
010 → Frequency Synthesizer mode (FS)  :: RH_RF69_OPMODE_MODE_FS 
011 → Transmitter mode (TX) 		   :: RH_RF69_OPMODE_MODE_TX   
100 → Receiver mode (RX) 			   :: RH_RF69_OPMODE_MODE_RX   
*/ 
// complete
void RFM_modeSetter(char mode){
	char opmode = rfm_read(REG_OPMODE); // access 0x01 register which holds operation mode 
    //char opmode = RFM_readReg(RH_RF69_REG_01_OPMODE);
    opmode &= ~(RH_RF69_OPMODE_MODE); // set bits 4-2 to zero
    //opmode &= ~RH_RF69_OPMODE_MODE; // setting bits 4-2 to zero 
    opmode |= (mode & RH_RF69_OPMODE_MODE); // setting bits 4-2 to the mode we want 
    rfm_write(REG_OPMODE, opmode);
    //RFM_writeReg(RH_RF69_REG_01_OPMODE, opmode);
    // Wait for mode to change. this could cause problems 
    // while (!(RFM_readReg(RH_RF69_REG_27_IRQFLAGS1,cs) & RH_RF69_IRQFLAGS1_MODEREADY));
}

// complete
void RFM_setMode(char* currentMode, char mode){
	// already in RX mode ?
	if (*currentMode == mode){
		return ; 
	}
	// STDBYMODE
	if (mode == 0){
		*currentMode = 0; 
		rfm_write(REG_TESTPA1, 0x55); // boosts power to transmitter
		//RFM_writeReg(RH_RF69_REG_5A_TESTPA1, 0x55); // used to boost power to transmitter / reciever 
		rfm_write(REG_TESTPA2, 0x70);
		// RFM_writeReg(RH_RF69_REG_5C_TESTPA2, 0x70); 
		RFM_modeSetter(RH_RF69_OPMODE_MODE_STDBY);
	}
	// RXMODE
	else if (mode == 1) {
		rfm_write(REG_TESTPA1, 0x55); // used to boost power to transmitter / reciever 
		//RFM_writeReg(RH_RF69_REG_5A_TESTPA1, 0x55);
		rfm_write(REG_TESTPA2, 0x70);
		//RFM_writeReg(RH_RF69_REG_5C_TESTPA2, 0x70); 
		RFM_modeSetter(RH_RF69_OPMODE_MODE_RX); 
		rfm_write(REG_DIOMAPPING1, 0x40); // set DIO0 to "PAYLOADREADY" in receive mode
		//RFM_writeReg(RH_RF69_REG_25_DIOMAPPING1, 0x40);
		RFM_setHighPower(0);
		*currentMode = 1 ; 
	}
	// TXMODE
	else if (mode == 2){
		rfm_write(REG_TESTPA1, 0x5D); // used to boost power to transmitter / reciever 
		//RFM_writeReg(RH_RF69_REG_5A_TESTPA1, 0x5d);
		rfm_write(REG_TESTPA2, 0x7C);
		//RFM_writeReg(RH_RF69_REG_5C_TESTPA2, 0x7c); 
		RFM_modeSetter(RH_RF69_OPMODE_MODE_TX); 
		rfm_write(REG_DIOMAPPING1, 0x00); // setting DIO0 to packetsent for TX 
		//RFM_writeReg(RH_RF69_REG_25_DIOMAPPING1, 0x00);
		RFM_setHighPower(1);
		*currentMode = 2; 
	}
}

// set *transmit/TX* output power: 0=min, 31=max
// this results in a "weaker" transmitted signal, and directly results in a lower RSSI at the receiver
// the power configurations are explained in the SX1231H datasheet (Table 10 on p21; RegPaLevel p66): http://www.semtech.com/images/datasheet/sx1231h.pdf
// valid powerLevel parameter values are 0-31 and result in a directly proportional effect on the output/transmission power
// this function implements 2 modes as follows:
//       - for RFM69W the range is from 0-31 [-18dBm to 13dBm] (PA0 only on RFIO pin)
//       - for RFM69HW the range is from 0-31 [5dBm to 20dBm]  (PA1 & PA2 on PA_BOOST pin & high Power PA settings - see section 3.3.7 in datasheet, p22)
// complete
void RFM_setPowerLevel(char powerLevel){
  powerLevel = (powerLevel > 31 ? 31 : powerLevel);
  powerLevel /= 2;
  rfm_write(REG_PALEVEL, (rfm_read(REG_PALEVEL) & 0xE0) | powerLevel);
  //RFM_writeReg(RH_RF69_REG_11_PALEVEL, (RFM_readReg(RH_RF69_REG_11_PALEVEL) & 0xE0) | powerLevel);
}

// for RFM69HW only: you must call setHighPower(true) after initialize() or else transmission won't work
// complete
void RFM_setHighPower(char onOff){
	rfm_write(REG_OCP, onOff ? 0x0F : 0x1A); // turning off the overload current protection for PA 
	//RFM_writeReg(RH_RF69_REG_13_OCP,onOff ? 0x0F : 0x1A);
	if (onOff){
		rfm_write(REG_PALEVEL, (rfm_read(REG_PALEVEL) & 0x1F) | 0x40 | 0x20);
		//RFM_writeReg(RH_RF69_REG_11_PALEVEL, (RFM_readReg(RH_RF69_REG_11_PALEVEL) & 0x1F) | 0x40 | 0x20);
	}
	else {
		rfm_write(REG_PALEVEL, (rfm_read(REG_PALEVEL) & 0x1F & ~0x40 & ~0x20));
		//RFM_writeReg(RH_RF69_REG_11_PALEVEL,(RFM_readReg(RH_RF69_REG_11_PALEVEL) & 0x1F & ~0x40 & ~0x20 ));
	}
}

// get the received signal strength indicator (RSSI)
// complete
int RFM_readRSSI(){
  int rssi = 0;
  rfm_write(REG_RSSICONFIG, 0x01); // start the measurements
  // RFM_writeReg(RH_RF69_REG_23_RSSICONFIG, 0x01);
  while((rfm_read(REG_RSSICONFIG) & 0x02) == 0x00); // wait for RSSI_READY
  //while ((RFM_readReg(RH_RF69_REG_23_RSSICONFIG) & 0x02) == 0x00);
  rssi = -rfm_read(REG_RSSIVALUE);
  //rssi = -RFM_readReg(RH_RF69_REG_24_RSSIVALUE);
  rssi >>= 1;
  return rssi;
}
