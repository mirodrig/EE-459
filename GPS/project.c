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

// void ds1631_init ( void );
// void ds1631_conv ( void );
// void ds1631_temp ( unsigned char *);

//Global Variables
// volatile int temp = 0;
// volatile int tempT = 0;
// volatile int lastTemp = 0;
// volatile int low = 70;
// volatile int lastLow = 70;
// volatile int high = 100;
// volatile int lastHigh = 100;
// volatile int adjHigh = 0;
// volatile char A;
// volatile char B;
// volatile int rTemp = 0;
// volatile int lastRTemp = 0;
// volatile char retVal[4];
// volatile char retVal[4];
// volatile int count;

//Transmission Function
// void tx_char(char ch){
//   // Wait for transmitter data register empty
//   // while ((UCSR0A & (1<<UDRE0)) == 0) {}
//   // UDR0 = ch;
// }

int main(void) {
  DDRD |= (1 << PD1); /* Set PD1 for output */ 
  PORTD |= (1 << PD0); /* Set PD0 for input */
  
  //Set Interrupts
  PCICR |= (1 << PCIE1);
  PCMSK1 |= ((1 << PCINT9)|(1 << PCINT10));
  sei();
  
  //Define MYUBRR
  #define FOSC 9830400 // clock frequency
  #define BAUD 9600 // for serial interface
  //#define MYUBRR FOSC/16/BAUD-1
  #define MYUBRR 47
  UBRR0 = MYUBRR; // Set baud rate\
  
  UCSR0A |= (1 << TXEN0 | 1 << RXEN0 | 1 << RXCIE0); // Enable RX and TX
  UCSR0C = (3 << UCSZ00); // Async., no parity,
              // 1 stop bit, 8 data bits
  
  while (1) {

  }               // Loop forever
    return 0;   /* never reached */
}

ISR(USART_RX_vect){
  // character has been received
  unsigned char receiver = UDR0;
}