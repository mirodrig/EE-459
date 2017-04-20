#include "../serial_test/serial.h"

#define row1_col1	0x00
#define row1_col20	0x13
#define row2_col1	0x40
#define row2_col20	0x53
#define row3_col1	0x14
#define row3_col20	0x27
#define row4_col1	0x54
#define row4_col20	0x67

void lcd_init(){
	serial_out(0xFE); // turns on the display
	serial_out(0x41);
	_delay_ms(10); // wait 10 ms
	serial_out(0xFE); // clears the screen
	serial_out(0x51);
	_delay_ms(10);
}

void lcd_clear(){
	serial_out(0xFE); // just clears the display
	serial_out(0x51);
	_delay_ms(10);
}

void lcd_out(char pos, char *s){
	serial_out(0xFE); // set the cursor position
	serial_out(0x45);
	serial_out((char) pos); // positions cursor
	serial_outs(s); // outputs the string
}
