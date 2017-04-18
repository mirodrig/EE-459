/**************************************
 * main.c
 * ***********************************/

#include <avr/io.h>
#include <util/delay.h>
#include "../serial_test/serial.h"

void lcd_init(void);
void lcd_out(int, int, char *);

unsigned char str1[] = "12345678901234567890";
unsigned char str2[] = ">> USC EE459L <<";

#define FOSC 9830400
#define BAUD 19200 // baud rate used by the LCD display

#define MYUBRR FOSC/16/BAUD-1 // value for UBRR0 register

int main(void){
	serial_init(MYUBRR); // Initialize the SCI port
	lcd_init(); // Initialize the LCD
	lcd_out(1, 1, (char *) str1); // Print string on line 1
	lcd_out(3, 2, (char *) str2); // Print string on line 2
	while (1) { // Loop forever
	}
	return 0;   /* never reached */
}
void lcd_init(){
	_delay_ms(250);
	_delay_ms(250);
	serial_out(0xFE); // clears the screen
	serial_out(0x58);
}

void lcd_out(int col, int row, char *s){
	serial_out((char) 0xFE); // Set the cursor position
	serial_out((char) 0x47);
	serial_out((char) col);
	serial_out((char) row);
	serial_outs(s);
}
