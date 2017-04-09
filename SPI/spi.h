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
    /* wait until all data is sent through serial */
    while (!(SPSR & _BV(SPIF)));
    return SPDR;
}

#endif