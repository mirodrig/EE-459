/************************************************
 * serial.c
 * This tests the functionality of the serial connection
************************************************/
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include "serial.h"

// define parameters
//#define FOSC 9830400 // clock frequency
#define BAUD 9600 // for serial interface
//#define MYUBRR FOSC/16/BAUD-1
#define MYUBRR 47

int main(void){
	DDRB |= DDB2; // set as output
	PORTB |= 1 << PB2;

	serial_init(MYUBRR);
	
	char str[] = "\r\nhello\n\r";
	serial_outs(str);

	while(1){
	}
	return 0;
}
