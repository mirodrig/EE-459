/************************************************
 * serial.c
 * This tests the functionality of the serial connection
************************************************/
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>

// define parameters
#define FOSC 9830400 // clock frequency
#define BAUD 9600 // for serial interface
//#define MYUBRR FOSC/16/BAUD-1
#define MYUBRR 47

void serial_init(uint8_t ubrr);
char serial_in();
void serial_out(char);
void serial_outs(char*);

// PD4 outputs to serial
// Test string to test on the screen
char *strs = "Testing the serial connection";

int main(void){
	serial_init(MYUBRR);
	

	char str[] = "hello";
	serial_outs(str);

	//char dummy;
	while(1){
		//serial_out(serial_in());
	}
	return 0;
}

// initialize the serial port. Takes in no parameters.
void serial_init(uint8_t ubrr){
	UBRR0 = ubrr; // set the BAUD rate
	UCSR0B |= (1 << TXEN0); // turn on the transmitter
	UBRR0H = (MYUBRR >> 8); // load upper 8 bits to high byte
	UBRR0L = MYUBRR; // load lower 8 bits of BUAD to low byte
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
	while((UCSR0A & (1 << UDRE0)) == 0);
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