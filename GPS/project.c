/********************************************
*
*  Name: Andrew Prajogi
*  Section: 31395 (Weber)
*  Assignment: Final Project - Thermostat with Remote Sensor
*
********************************************/

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "ds1631.h"

// define parameters
#define FOSC 9830400 // clock frequency
#define BAUD 9600 // for serial interface
//#define MYUBRR FOSC/16/BAUD-1
#define MYUBRR 47

void serial_init(uint8_t ubrr);

int main(void) {
  DDRD |= (1 << PD1); /* Set PD1 for output */ 
  PORTD |= (1 << PD0); /* Set PD0 for input */
  
  //Set Interrupts
  sei();
  
  //Define MYUBRR
  #define FOSC 9830400 // clock frequency
  #define BAUD 9600 // for serial interface
  //#define MYUBRR FOSC/16/BAUD-1
  #define MYUBRR 47
 
  serial_init(MYUBRR);
              // 1 stop bit, 8 data bits
  
  while (1) {
  }               // Loop forever
    return 0;   /* never reached */
}

// initialize the serial port. Takes in no parameters.
void serial_init(uint8_t ubrr){
  UBRR0 = ubrr; // set the BAUD rate
  UCSR0B |= (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0); // turn on the transmitter
  //UBRR0H = (MYUBRR >> 8); // load upper 8 bits to high byte
  //UBRR0L = MYUBRR; // load lower 8 bits of BUAD to low byte
  UCSR0C = (1 << UCSZ01) | (3 << UCSZ00); // set for async operation 8 data bits, one stop bit
}

// output a byte to the serial port
void serial_out(char ch){
  while((UCSR0A & (1 << UDRE0)) == 0);
  UDR0 = ch;
  //while((UCSR0A & (1 << UDRE0)) == 0);
}

ISR(USART_RX_vect){
//   // character has been received
//   unsigned char receiver = UDR0;
  serial_out(UDR0);
}


