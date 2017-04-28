/**************************************
 * main.c
 * ***********************************/

#include <avr/io.h>
#include <util/delay.h>
#include "../serial_test/serial.h"

void lcd_init(void);
//void lcd_out(int, int, char *);
void lcd_out(char, char *);

unsigned char str1[] = "12345678901234567";
unsigned char str2[] = ">> USC EE459L <<";
unsigned char str3[] = "me la pelas";

#define FOSC 9830400
#define BAUD 9600
#define MYUBRR 47

#define line1_col1	0x00
#define line1_col20	0x13
#define line2_col1	0x40
#define line2_col20	0x53
#define line3_col1	0x14
#define line3_col20	0x27
#define line4_col1	0x54
#define line4_col20	0x67

int main(void){
	serial_init(47); // Initialize the SCI port
	lcd_init(); // Initialize the LCD
	//lcd_out(1, 1, (char *) str1); // Print string on line 1
	//lcd_out(3, 3, (char *) str2); // Print string on line 2

	lcd_out(line1_col1, (char*)str1);
	lcd_out(line2_col1, (char*)str2);
	lcd_out(line3_col1, (char*)str3);

	while (1) { // Loop forever
	}
	return 0;   /* never reached */
}
void lcd_init(){
	serial_out(0xFE); // turns on the display
	serial_out(0x41);	
	_delay_ms(10);

	serial_out(0xFE); // clears the display and moves cursor to top left corner
	serial_out(0x51);
	_delay_ms(10);
}

void lcd_out(char rowCol, char *s){
//void lcd_out(int col, int row, char *s){
	serial_out(0xFE); // Set the cursor position
	serial_out(0x45);

	serial_out((char) rowCol); // see if this works
	//serial_out((char) row);
	
	serial_outs(s); // outputs the string
}
