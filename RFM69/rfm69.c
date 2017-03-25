/*************************************
 * rfm69.c
*************************************/
#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>

#include "rfm69.h"
#include "rfm69_config.h"
#include "rfm69_reg.h"
#include "spi.h"

static rfm_reg_t _mode; // track current mode of the radio

/* Private functions */
static rfm_status_t _rf69_read(const rfm_reg_t reg, rfm_reg_t* result);
static rfm_status_t _rf69_write(const rfm_reg_t reg, const rfm_reg_t val);
static rfm_status_t _rf69_burst_read(const rfm_reg_t reg, rfm_reg_t* dest, uint8_t len);
static rfm_status_t _rf69_fifo_write(const rfm_reg_t* src, uint8_t len);
static rfm_status_t _rf69_clear_fifo(void);

/** Initialise the RFM69 device and set into SLEEP mode (0.1uA)
 * @returns RFM_OK for success, RFM_FAIL for failure. */
rfm_status_t rf69_init(void){
    uint8_t i;
    rfm_reg_t res;
    /* Call the user setup function to configure the SPI peripheral */
    if (spi_init() != RFM_OK)
        return RFM_FAIL;
    /* Zero version number, RFM probably not connected/functioning */
    _rf69_read(REG_VERSION, &res);
    if (!res){
        return RFM_FAIL;
    }
    /* Set up device */
    for (i = 0; CONFIG[i][0] != 255; i++)
        _rf69_write(CONFIG[i][0], CONFIG[i][1]);
    /* Set initial mode */
    rf69_set_mode(RFM69_MODE_SLEEP);
    return RFM_OK;
}

/* Read a single byte from a register in the RFM69. Transmit the (one byte)
 * address of the register to be read, then read the (one byte) response.
 * @param reg The register address to be read
 * @param result A pointer to where to put the result
 * @returns RFM_OK for success, RFM_FAIL for failure. */
static rfm_status_t _rf69_read(const rfm_reg_t reg, rfm_reg_t* result){
    rfm_reg_t data;
    spi_ss_assert();
    /* Transmit the reg we want to read from */
    spi_exchange_single(reg, &data);
    /* Read the data back */
    spi_exchange_single(0xFF, result);
    spi_ss_deassert();
    return RFM_OK;
}

/* Write a single byte to a register in the RFM69. Transmit the register
 * address (one byte) with the write mask RFM_SPI_WRITE_MASK on, and then the
 * value of the register to be written.
 * @param reg The address of the register to write
 * @param val The value for the address
 * @returns RFM_OK for success, RFM_FAIL for failure. */
static rfm_status_t _rf69_write(const rfm_reg_t reg, const rfm_reg_t val){
    rfm_reg_t dummy;
    spi_ss_assert();
    /* Transmit the reg address */
    spi_exchange_single(reg | RFM69_SPI_WRITE_MASK, &dummy);
    /* Transmit the value for this address */
    spi_exchange_single(val, &dummy);
    spi_ss_deassert();
    return RFM_OK;
}

/* Read a given number of bytes from the given register address into a 
 * provided buffer
 * @param reg The address of the register to start from
 * @param dest A pointer into the destination buffer
 * @param len The number of bytes to read
 * @returns RFM_OK for success, RFM_FAIL for failure. */
static rfm_status_t _rf69_burst_read(const rfm_reg_t reg, rfm_reg_t* dest, uint8_t len){
    rfm_reg_t dummy;
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
static rfm_status_t _rf69_fifo_write(const rfm_reg_t* src, uint8_t len){
    rfm_reg_t dummy;
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
rfm_status_t rf69_set_mode(const rfm_reg_t newMode){
    rfm_reg_t res;
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
rfm_status_t rf69_receive(rfm_reg_t* buf, rfm_reg_t* len, int16_t* lastrssi, bool* rfm_packet_wait){
    rfm_reg_t res;
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
rfm_status_t rf69_send(const rfm_reg_t* data, uint8_t len, const uint8_t power){
    rfm_reg_t oldMode, res;
    uint8_t paLevel;
    /* power is TX Power in dBmW (valid values are 2dBmW-20dBmW) */
    if (power < 2 || power > 20){
        /* Could be dangerous, so let's check this */
        return RFM_FAIL;
    }
    oldMode = _mode;
    /* Start transmitter */
    rf69_set_mode(RFM69_MODE_TX);
    /* Set up PA */
    if (power <= 17) {
        /* Set PA Level */
        paLevel = power + 28;
        _rf69_write(REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | paLevel);        
    }
    else {
        /* Disable Over Current Protection */
        _rf69_write(REG_OCP, RF_OCP_OFF);
        /* Enable High Power Registers */
        _rf69_write(REG_TESTPA1, 0x5D);
        _rf69_write(REG_TESTPA2, 0x7C);
        /* Set PA Level */
        paLevel = power + 11;
        _rf69_write(REG_PALEVEL, RF_PALEVEL_PA0_OFF | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON | paLevel);
    }

    /* Wait for PA ramp-up */
    res = 0;
    while (!(res & RF_IRQFLAGS1_TXREADY))
        _rf69_read(REG_IRQFLAGS1, &res);

    /* Throw Buffer into FIFO, packet transmission will start 
     * automatically */
    _rf69_fifo_write(data, len);

    /* Wait for packet to be sent */
    res = 0;
    while (!(res & RF_IRQFLAGS2_PACKETSENT))
        _rf69_read(REG_IRQFLAGS2, &res);

    /* Return Transceiver to original mode */
    rf69_set_mode(oldMode);

    /* If we were in high power, switch off High Power Registers */
    if (power > 17) {
        /* Disable High Power Registers */
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
    rfm_reg_t oldMode, temp;
    uint8_t timeout;
    oldMode = _mode;
    /* Set mode into Standby (required for temperature measurement) */
    rf69_set_mode(RFM69_MODE_STDBY);
    /* Trigger Temperature Measurement */
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
    rfm_reg_t res;
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
	DDRD |= DDD2; // set pin 4 as output
	PORTD |= 1 << PD2; // set the pin to HIGH
	// TODO: figure out what the "reg" and "result" are.
	if(_mode == RFM69_MODE_RX && (_rf69_read(REG_IRQFLAGS2, 0) & RF_IRQFLAGS2_PAYLOADREADY )){
		//RSSI = readRSSI();
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

// To enable encryption: radio.encrypt("ABCDEFGHIJKLMNOP");
// To disable encryption: radio.encrypt(null) or radio.encrypt(0)
// KEY HAS TO BE 16 bytes !!!
void rf69_encrypt(const char *key){
	rf69_set_mode(RFM69_MODE_STDBY);
	if(key != 0){
		spi_ss_assert();
		spi_transfer(REG_AESKEY1 | 0x80);
		uint8_t i;
		for(i=0; i<16; i++){
			spi_transfer(key[i]);
		}
		spi_ss_deassert();
	}
	_rf69_write(REG_PACKETCONFIG2, (_rf69_read(REG_PACKETCONFIG2, 0) & 0xFE) | (key ? 1 : 0));
}

