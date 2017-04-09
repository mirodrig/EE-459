/*********************************
 * rfm69.h
*********************************/
#ifndef RFM69_H_
#define RFM69_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "../serial_test/serial.h"
#include "../SPI/spi.h"
#include "rfm69_reg.h"

/* Status codes for return values from library functions. These will
determine if an operation is OK, fails, or timesout */
typedef enum rfm_status_t { RFM_OK, RFM_FAIL, RFM_TIMEOUT } rfm_status_t;

/* Write commands to the RFM have this bit set */
#define RFM69_SPI_WRITE_MASK 0x80 // first bit is 1 followed by all zeros
/* Maximum message length that can be supported by this library. 
 * Interrupts will refill the Tx FIFO during transmission and to empty 
 * Rx FIFO during reception */
#define RFM69_MAX_MESSAGE_LEN 64
/* Max number of octets the RFM69 FIFO can hold */
#define RFM69_FIFO_SIZE 64

#define RFM69_MODE_SLEEP    0x00 /* 0.1uA  */
#define RFM69_MODE_STDBY    0x04 /* 1.25mA */
#define RFM69_MODE_RX       0x10 /* 16mA   */
#define RFM69_MODE_TX       0x0C /* >33mA  */

/* These values we set for FIFO thresholds are actually the same as the 
 * POR values */
#define RF22_TXFFAEM_THRESHOLD 4
#define RF22_RXFFAFULL_THRESHOLD 55

/* definitions with respect to interrupts */
#define RF69_IRQ_PIN 4 // PD2
#define RF69_IRQ_NUM 0

static uint8_t _mode;

uint8_t DATA[RFM69_MAX_MESSAGE_LEN];

/* declare function prototypes here */
rfm_status_t rf69_init(void);
static rfm_status_t _rf69_read(const uint8_t, uint8_t*);
static rfm_status_t _rf69_write(const uint8_t, const uint8_t);
static rfm_status_t _rf69_burst_read(const uint8_t, uint8_t*, uint8_t);
static rfm_status_t _rf69_fifo_write(const uint8_t*, uint8_t);
rfm_status_t rf69_set_mode(const uint8_t);
rfm_status_t rf69_receive(uint8_t*, uint8_t*, int16_t*, bool*);
rfm_status_t rf69_send(const uint8_t*, uint8_t, const uint8_t);
static rfm_status_t _rf69_clear_fifo(void);
rfm_status_t rf69_read_temp(int8_t*);
rfm_status_t rf69_sample_rssi(int16_t*);
rfm_status_t interruptHandler(void);
bool rf69_can_send();
bool rf69_receive_done();

rfm_status_t spi_init(void);
rfm_status_t spi_exchange_single(const uint8_t, uint8_t*);
rfm_status_t spi_ss_assert(void);
rfm_status_t spi_ss_deassert(void);

/** Initialise the RFM69 device and set into SLEEP mode (0.1uA)
 * @returns RFM_OK for success, RFM_FAIL for failure. */
rfm_status_t rf69_init(void){
	serial_outs("\r\nCalling rf69_init() function\n\r");
	static const uint8_t CONFIG[][2] = {
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
    	//RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
        { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF },
    	 // run DAGC continuously in RX mode, recommended default for AfcLowBetaOn=0
        { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 },
    	// { RFM69_REG_71_TEST_AFC, 0x0E }, //14* 488hz = ~7KHz
    	{255, 0}
	};

	uint8_t i;
    uint8_t res;
    /* Call the user setup function to configure the SPI peripheral */
    if(spi_init() != RFM_OK){
        //serial_outs("\rSPI failed to setup\n\r");
        return RFM_FAIL;
    }
    /* Zero version number, RFM probably not connected/functioning */
    _rf69_read(REG_VERSION, &res); // default of REG_VERSION is 0x24
    if (!res){
        //serial_outs("\rERROR: Zero Version Number\n\r");
        return RFM_FAIL;
    }
    /* Set up device */
    for (i = 0; CONFIG[i][0] != 255; i++)
        _rf69_write(CONFIG[i][0], CONFIG[i][1]);
    /* Set initial mode */
    rf69_set_mode(RFM69_MODE_SLEEP);
    serial_outs("\rRFM69 is now in sleep mode\n\r");
    return RFM_OK;
}

/* Read a single byte from a register in the RFM69. Transmit the (one byte)
 * address of the register to be read, then read the (one byte) response.
 * @param reg The register address to be read
 * @param result A pointer to where to put the result
 * @returns RFM_OK for success, RFM_FAIL for failure. */
static rfm_status_t _rf69_read(const uint8_t reg, uint8_t* result){
    cli();
    char buffer[50];
    uint8_t data;
    spi_ss_assert(); // sets the SS (pin 16) to LOW
    /* transmit the reg address to the RFM69 */
    spi_exchange_single(reg, &data); // make sure 1st bit is a zero
    /* Read the data back. Result will be received by the uC
    over MISO. */
    spi_exchange_single(0xFF, result);

    snprintf(buffer, 50, "\rRead from reg 0x%02X and received 0x%02X\n\r", reg, *result);
    serial_outs(buffer);

    spi_ss_deassert(); // sets the SS (pin 16) to HIGH
    sei();
    return RFM_OK;
}

/* Write a single byte to a register in the RFM69. Transmit the register
 * address (one byte) with the write mask RFM_SPI_WRITE_MASK on, and then the
 * value of the register to be written.
 * @param reg The address of the register to write
 * @param val The value for the address
 * @returns RFM_OK for success, RFM_FAIL for failure. */
static rfm_status_t _rf69_write(const uint8_t reg, const uint8_t val){
    cli();
    char buffer[50];
    uint8_t dummy;
    spi_ss_assert();
    /* Transmit the reg address to the RFM69. */
    spi_exchange_single(reg | RFM69_SPI_WRITE_MASK, &dummy);
    /* Transmit the value for this address */
    spi_exchange_single(val, &dummy);
    snprintf(buffer, 50, "\rWrite to 0x%02X with 0x%02X\n\r", reg, val);
    serial_outs(buffer);

    spi_ss_deassert();
    sei();
    return RFM_OK;
}

/* Read a given number of bytes from the given register address into a 
 * provided buffer
 * @param reg The address of the register to start from
 * @param dest A pointer into the destination buffer
 * @param len The number of bytes to read
 * @returns RFM_OK for success, RFM_FAIL for failure. */
static rfm_status_t _rf69_burst_read(const uint8_t reg, uint8_t* dest, uint8_t len){
    uint8_t dummy;
    spi_ss_assert();
    /* Send the start address with the write mask off */
    spi_exchange_single(reg & ~RFM69_SPI_WRITE_MASK, &dummy);
    while (len--) {
        spi_exchange_single(0xFF, dest);
        dest++;
    }
    spi_ss_deassert();
    return RFM_OK;
}

/* Write data into the FIFO on the RFM69
 * @param src The source data comes from this buffer
 * @param len Write this number of bytes from the buffer into the FIFO
 * @returns RFM_OK for success, RFM_FAIL for failure. */
static rfm_status_t _rf69_fifo_write(const uint8_t* src, uint8_t len){
    uint8_t dummy;
    spi_ss_assert();
    /* Send the start address with the write mask on */
    spi_exchange_single(REG_FIFO | RFM69_SPI_WRITE_MASK, &dummy);
    /* First byte is packet length */
    spi_exchange_single(len, &dummy);
    /* Then write the packet */
    while (len--)
        spi_exchange_single(*src++, &dummy);
    spi_ss_deassert();
    return RFM_OK;
}
/* Change the RFM69 operating mode to a new one.
 * @param newMode The value representing the new mode (see datasheet for
 * further information). The MODE bits are masked in the register, i.e. only
 * bits 2-4 of newMode are ovewritten in the register.
 * @returns RFM_OK for success, RFM_FAIL for failure. */
rfm_status_t rf69_set_mode(const uint8_t newMode){
    uint8_t res;
    _rf69_read(REG_OPMODE, &res);
    _rf69_write(REG_OPMODE, (res & 0xE3) | newMode);
    _mode = newMode;
    return RFM_OK;
}

/* Get data from the RFM69 receive buffer.
 * @param buf A pointer into the local buffer in which we would like the data.
 * @param len The length of the data
 * @param lastrssi The RSSI of the packet we're getting
 * @param rfm_packet_waiting A boolean pointer which is true if a packet was
 * received and has been put into the buffer buf, false if there was no packet
 * to get from the RFM69.
 * @returns RFM_OK for success, RFM_FAIL for failure. */
rfm_status_t rf69_receive(uint8_t* buf, uint8_t* len, int16_t* lastrssi, bool* rfm_packet_wait){
    uint8_t res;
    /* set the mode of the transceiver to RX */
    if(_mode != RFM69_MODE_RX){
        rf69_set_mode(RFM69_MODE_RX);
    }
    /* Check IRQ register for payloadready flag
     * (indicates RXed packet waiting in FIFO) */
    _rf69_read(REG_IRQFLAGS2, &res);

    if (res & RF_IRQFLAGS2_PAYLOADREADY){
        /* Get packet length from first byte of FIFO */
        _rf69_read(REG_FIFO, len);
        *len += 1;
        /* Read FIFO into our Buffer */
        _rf69_burst_read(REG_FIFO, buf, RFM69_FIFO_SIZE);
        /* Read RSSI register (should be of the packet? - TEST THIS) */
        _rf69_read(REG_RSSIVALUE, &res);
        *lastrssi = -(res/2);
        /* Clear the radio FIFO (found in HopeRF demo code) */
        _rf69_clear_fifo();
        *rfm_packet_wait = true;
        return RFM_OK;
    }
    *rfm_packet_wait = false;
    return RFM_OK;
}

/* Send a packet using the RFM69 radio.
 * @param data The data buffer that contains the string to transmit
 * @param len The number of bytes in the data packet (excluding preamble, sync
 * and checksum)
 * @param power The transmit power to be used in dBm
 * @returns RFM_OK for success, RFM_FAIL for failure. */
rfm_status_t rf69_send(const uint8_t* data, uint8_t len, const uint8_t power){
    uint8_t oldMode, res;
    uint8_t paLevel;
    /* power is TX Power in dBmW (valid values are 2dBmW-20dBmW) */
    if (power < 2 || power > 20){
        /* Could be dangerous, so let's check this */
        serial_outs("\rERROR: power failure\n\r");
        return RFM_FAIL;
    }
    oldMode = _mode;
    /* Start transmitter */
    rf69_set_mode(RFM69_MODE_TX); // 0x01 written with 0x0C
    serial_outs("\rRFM69 in Tx mode\n\r");
    /* Set up PA */
    if (power <= 17) {
        /* Set PA Level */
        paLevel = power + 28;
        /* enables PA0, disables PA1 and PA2. The PA0 output is on pin RFIO.
        This provides power to the LNA of the transmitter. */
        _rf69_write(REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF |
            RF_PALEVEL_PA2_OFF | paLevel);        
    }
    else {
        /* Disable Over Current Protection */
        _rf69_write(REG_OCP, RF_OCP_OFF);
        /* Enable High Power Registers */
        _rf69_write(REG_TESTPA1, 0x5D); // set to 20 dBm mode
        _rf69_write(REG_TESTPA2, 0x7C); // set to 20 dBm mode
        /* Set PA Level */
        paLevel = power + 11;
        /* enables both PA1 and PA2 while disabling PA0. This delivers
        20 dBm to the antenna. */
        _rf69_write(REG_PALEVEL, RF_PALEVEL_PA0_OFF |
            RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON | paLevel);
    }
    /* Wait for PA ramp-up */
    res = 0;
    /* wait until the TXREADY interrupt is sent */
    while (!(res & RF_IRQFLAGS1_TXREADY)){
        //serial_outs("\r\nTransmitter is ready to transmit\n\r");
        _rf69_read(REG_IRQFLAGS1, &res);
    }
    /* Throw Buffer into FIFO, packet transmission will start 
     * automatically */
    _rf69_fifo_write(data, len);

    res = 0;
    /* wait until the complete packet has been sent. Set to HIGH
    when the last bit of the shift register has been sent */
    while (!(res & RF_IRQFLAGS2_PACKETSENT))
        _rf69_read(REG_IRQFLAGS2, &res);

    /* Return Transceiver to original mode */
    rf69_set_mode(oldMode);

    /* If we were in high power, switch off High Power Registers */
    if (power > 17) {
        /* Disable High Power Registers. Set to normal setting and
        for Rx mode. */
        _rf69_write(REG_TESTPA1, 0x55);
        _rf69_write(REG_TESTPA2, 0x70);
        /* Enable Over Current Protection */
        _rf69_write(REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95);
    }
    return RFM_OK;
}

/* Clear the FIFO in the RFM69. We do this by entering STBY mode and then
 * returing to RX mode.
 * @warning Must only be called in RX Mode
 * @note Apparently this works... found in HopeRF demo code
 * @returns RFM_OK for success, RFM_FAIL for failure. */
static rfm_status_t _rf69_clear_fifo(void){
    rf69_set_mode(RFM69_MODE_STDBY);
    rf69_set_mode(RFM69_MODE_RX);
    return RFM_OK;
}


/* The RFM69 has an onboard temperature sensor, read its value
 * @warning RFM69 must be in one of the active modes for temp sensor to work.
 * @param temperature A pointer to the variable into which the temperature will
 * be read by this method.
 * @returns RFM_OK for success, RFM_FAIL for failure, RFM_TIMEOUT if there is a
 * timeout due to the sensor on the RFM not starting and/or finishing a
 * conversion. */
rfm_status_t rf69_read_temp(int8_t* temperature){
    /* Store current transceiver mode */
    uint8_t oldMode, temp;
    uint8_t timeout;
    oldMode = _mode;
    /* Set mode into Standby (required for temperature measurement) */
    rf69_set_mode(RFM69_MODE_STDBY);
    /* Trigger Temperature Measurement (optional for this case) */
    _rf69_write(REG_TEMP1, RF_TEMP1_MEAS_START);

    /* Check Temperature Measurement has started */
    timeout = 0;
    temp = 0;
    while (!(RF_TEMP1_MEAS_RUNNING & temp)) {
        _rf69_read(REG_TEMP1, &temp);
        _delay_ms(1);
        if(++timeout > 50){
            *temperature = -127.0;
            return RFM_TIMEOUT;
        }
        _rf69_write(REG_TEMP1, RF_TEMP1_MEAS_START);
    }

    /* Wait for Measurement to complete */
    timeout = 0;
    temp = 0;
    while (RF_TEMP1_MEAS_RUNNING & temp) {
        _rf69_read(REG_TEMP1, &temp);
        _delay_ms(1);
        if(++timeout > 10){
            *temperature = -127.0;
            return RFM_TIMEOUT;
        }
    }
    /* Read raw ADC value */
    temp = 0;
    _rf69_read(REG_TEMP2, &temp);
    /* Set transceiver back to original mode */
    rf69_set_mode(oldMode);
    /* Return processed temperature value */
    *temperature = 161 - (int8_t)temp;
    return RFM_OK;
}

/* Get the last RSSI value from the RFM69
 * @warning Must only be called when the RFM69 is in rx mode
 * @param rssi A pointer to an int16_t where we will place the RSSI value
 * @returns RFM_OK for success, RFM_FAIL for failure. */
rfm_status_t rf69_sample_rssi(int16_t* rssi){
    uint8_t res;
    /* Must only be called in RX mode */
    if (_mode != RFM69_MODE_RX)
        return RFM_FAIL;
    /* Trigger RSSI Measurement */
    _rf69_write(REG_RSSICONFIG, RF_RSSI_START);

    /* Wait for Measurement to complete */
    while (!(RF_RSSI_DONE & res))
        _rf69_read(REG_RSSICONFIG, &res);

    /* Read, store in _lastRssi and return RSSI Value */
    res = 0;
    _rf69_read(REG_RSSIVALUE, &res);
    *rssi = -(res/2);

    return RFM_OK;
}

rfm_status_t interruptHandler(void){
	// DDRD |= DDD2; // set pin 4 as output
	// PORTD |= (1 << PD2); // set the pin to HIGH
	// TODO: figure out what the "reg" and "result" are.
    uint8_t res;
    /* perform action if we are in Rx mode and the payload interrupr is set */
	if(_mode == RFM69_MODE_RX && (_rf69_read(REG_IRQFLAGS2, &res) & RF_IRQFLAGS2_PAYLOADREADY )){
		rf69_set_mode(RFM69_MODE_STDBY);
		spi_ss_assert(); // set slave select pin low // select()
		spi_transfer(REG_FIFO & 0x7F);
		
		uint8_t PAYLOADLEN = spi_transfer(0);
		PAYLOADLEN = PAYLOADLEN > 66 ? 66 : PAYLOADLEN; // precaution

		uint8_t DATALEN = PAYLOADLEN - 3;

		uint8_t i;
		for(i=0; i<DATALEN; i++){
			DATA[i] = spi_transfer(0);
		}
		if(DATALEN < RFM69_MAX_MESSAGE_LEN){
			DATA[DATALEN] = 0; // add NULL at the end of the string
		}
    	spi_ss_deassert(); // unselect();
    	rf69_set_mode(RFM69_MODE_RX);
  	}
  	spi_ss_deassert();
  	return RFM_OK;
}

bool rf69_can_send(){
    /* if we are in both Rx mode and the payload is ready, return true */
    if(_mode == RFM69_MODE_RX && RF_IRQFLAGS2_PAYLOADREADY){
        rf69_set_mode(RFM69_MODE_STDBY); // set mode to standby
        return true;
    }
    return false;
}

bool rf69_receive_done(){
    /* if we are in both Rx mode and the payload is ready */
    if(_mode == RFM69_MODE_RX && RF_IRQFLAGS2_PAYLOADREADY){
        rf69_set_mode(RFM69_MODE_STDBY);
        return true;
    }
    else if(_mode == RFM69_MODE_RX){
        return false;
    }
    _rf69_write(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01);
    rf69_set_mode(RFM69_MODE_RX);
    return false;
}

/* User SPI setup function. Use this function to set up the SPI peripheral
 * on the microcontroller, such as to setup the IO, set the mode (0,0) for the
 * RFM69, and become a master.
 * @returns True on success, false on failure. */
rfm_status_t spi_init(void){
   /* Set up the SPI IO as appropriate */
	SPI_DDR |= SPI_SS | SPI_MOSI | SPI_SCK;
	SPI_DDR &= ~(SPI_MISO);

    /* Set SS high */
	SPI_PORT |= SPI_SS;

    /* SPI should be mode (0,0), MSB first, double clock rate. Change the
    values of the SPI registers */
	SPCR &= ~(_BV(CPOL) | _BV(CPHA) | _BV(DORD));
	SPSR |= _BV(SPI2X);

    /* Become master */
	SPCR |= _BV(MSTR);

    /* Finally, enable the SPI periph */
	SPCR |= _BV(SPE);

    /* Return RFM_OK if everything went ok, otherwise RFM_FAIL */
	return RFM_OK;
}

/* User function to exchange a single byte over the SPI interface
 * @warn This does not handle SS, since higher level functions might want to do
 * burst read and writes
 * @param out The byte to be sent
 * @returns The byte received */
rfm_status_t spi_exchange_single(const uint8_t out, uint8_t* in){
    SPDR = out; // load SPI data register with first parameter
    /* SPIF - interrupt indicating that serial transfer is complete. The SPIF
    will be loaded onto the SPSR register. */
    while(!(SPSR & (1 << SPIF)));
    *in = SPDR; // SPI data register is now filled with second parameter
    return RFM_OK;
}

/* User function to assert the slave select pin */
rfm_status_t spi_ss_assert(void){
    SPI_PORT &= ~(SPI_SS);
    return RFM_OK;
}

rfm_status_t spi_ss_deassert(void){
    SPI_PORT |= (SPI_SS);
    return RFM_OK;
}

#endif
