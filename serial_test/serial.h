/********************************************
 * serial.h
********************************************/
#ifndef SERIAL_H_
#define SERIAL_H_


#include <avr/io.h>
#include <stdio.h>
#include <string.h>

// initialize the serial port. Takes in no parameters.
void serial_init(uint8_t ubrr){
	UBRR0 = ubrr; // set the BAUD rate
	UCSR0B |= (1 << TXEN0); // turn on the transmitter
	UCSR0C = (1 << UCSZ01) | (3 << UCSZ00); // set for async operation 8 data bits, one stop bit
}

// read a byte from the USART0 and return it
char serial_in(){
	while(!(UCSR0A & (1 << RXC0)));
	return UDR0;
}

// output a byte to the serial port
void serial_out(char ch){
	while((UCSR0A & (1 << UDRE0)) == 0);
	UDR0 = ch;
}

// prints the contents of the character string s out of the serial
// port. The string must be terminated by a zero byte.
void serial_outs(char *s){
	int i = 0;
	while(s[i] != '\0'){
		serial_out(s[i]);
		i++;
	}
}

#endif
