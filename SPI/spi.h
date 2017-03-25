/*********************************************
 * spi.h
 * this file will be a library of useful spi functions
*********************************************/
#ifndef SPI_H_
#define SPI_H_

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>

#include "rfm69.h"
#include "rfm69_reg.h"

/* define SPI pins and ports */
#define SPI_DDR DDRB
#define SPI_PORT PORTB

/* define bit values */
#define SPI_DDR     DDRB
#define SPI_PORT    PORTB
#define SPI_SS      _BV(2) // pin 16
#define SPI_MOSI    _BV(3) // pin 17
#define SPI_MISO    _BV(4) // pin 18
#define SPI_SCK     _BV(5) // pin 19

void SPI_MasterInit(void){
	// set the MOSI and SCK for output. All others input
	DDRB |= 1 << DDB3;
	DDRB |= 1 << DDB5;
	// enable SPI, Master, set clock rate fck/16
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
}

void SPI_MasterTransmit(char cData){
	/* Start transmission */
	SPDR = cData; // SPI data register
	/* Wait for transmission complete */
	while(!(SPSR & (1<<SPIF)));
}

void SPI_SlaveInit(void){
   /* Set MISO output, all others input */
   DDRB = 1 << DDB4;
   /* Enable SPI */
   SPCR = 1 << SPE;
}

char SPI_SlaveReceive(void){
   /* Wait for reception complete */
   while(!(SPSR & (1<<SPIF)));
   /* Return Data Register */
   return SPDR;
}

uint8_t spi_transfer(uint8_t data){
    SPDR = data;
    while (!(SPSR & _BV(SPIF)));
    return SPDR;
}

/********* functions specific to the RFM69 ************/

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
rfm_status_t spi_exchange_single(const rfm_reg_t out, rfm_reg_t* in){
    SPDR = out;
    while(!(SPSR & (1<<SPIF)));
    *in = SPDR;
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