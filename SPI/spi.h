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
#include <avr/interrupt.h>

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
	cli();
  DDRB |= (1 << DDB3) | (1 << DDB3) | (1 << PB2);
	// DDRB |= 1 << DDB5;
	// enable SPI, Master, set clock rate fck/16
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
  sei();
}

void spi_init(void){
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
}

char spi_transfer(char data){
    SPDR = data;
    /* wait until all data is sent through serial */
    while (!(SPSR & _BV(SPIF)));
    return SPDR;
}

// transfer a list of characters
void SPI_multiTransfer(void *data, size_t count){
    if (count == 0){
      return;
    } 
    char *buf = (char *)data; // loading in the buffer
    SPDR = *buf; // SPDR is the registered that the data will be transfered from
    while (--count > 0){
        char out = *(buf+1);
        while(!(SPSR & (1<<SPIF) )); // where the actual transfer happens
        char in = SPDR; // getting the input
        SPDR = out; // loading the next byte to pushed out
        *buf += in; // loading into the buffer the recieved data
    }
    while(!(SPSR & (1<<SPIF) )); ; // get the last bit 
    *buf = SPDR;
}

void spi_multiWrite(void *data, size_t count){
    if (count == 0){
      return;
    }
    
    char *buf = (char *)data; // loading in the buffer
    char out = 0; 
    SPDR = *buf; // SPDR is the registered that the data will be transfered from
    while (--count > 0){
        out = *(buf+1);
        while(!(SPSR & (1<<SPIF) )); // where the actual transfer happens
        SPDR = out; // loading the next byte to pushed out
    }
    while(!(SPSR & (1<<SPIF) )); ; // get the last bit 
}

void spi_ss_assert(void){
    //PORTB &= ~(_BV(2));
    SPI_PORT &= ~(SPI_SS);
}

void spi_ss_deassert(void){
    SPI_PORT |= (SPI_SS);
    //PORTB |= _BV(2);
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



#endif
