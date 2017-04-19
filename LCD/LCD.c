#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h> 
#include <stdlib.h>
#include <avr/interrupt.h>
#include <stdbool.h>

// define parameters
#define FOSC 9830400 // clock frequency
#define BAUD 9600 // for serial interface
//#define MYUBRR FOSC/16/BAUD-1
#define MYUBRR 47

// initialize the serial port. Takes in no parameters.
void serial_init(uint8_t ubrr){
  UBRR0 = ubrr; // set the BAUD rate
  UCSR0B |= (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0); // turn on the transmitter
  UCSR0C = (3 << UCSZ00); // set for async operation 8 data bits, one stop bit
}

char serial_in(){
  while (!(UCSR0A & (1 << RXC0)));
  return UDR0;
}

void serial_out(char ch){
  while((UCSR0A & (1 << UDRE0)) == 0);
  UDR0 = ch;
}

void serial_outputString (char* text){
    char i; 
    for (i = 0; i < strlen(text); i++){
        serial_out(text[i]);
    }
    serial_out(0x0D);
    serial_out(0x0A);
}

int main(void){
	serial_init(MYUBRR);
    //serial_out(0xFE);
    //_delay_us(100);
    //serial_out(0x41);
    while (1){
    	serial_out(0x41);
        _delay_ms(2000);
 	}
    return 0;   /* never reached */
}