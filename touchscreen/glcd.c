/***********************************************
 * glcd.c
 * this file was obtained from an external source. I'll
 * determine which modifications have to be made to get
 * this to work with the project board.
***********************************************/
#define F_CPU 8000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <stdlib.h>	// abs()
#include <string.h>	// strlen()
#include "../SPI/spi.h" // we might have to check SPI functions at compile time
#include "font.h"
#include "glcd.h"
#include "ili9341_reg.h"

unsigned int bgcolor = black;

void glcd_cs_low(){
	DDRD |= 0b100000;
	//DDRD |= (1 << DDD5); // set PD5 as output
	PORTD &=~ 0b100000;
	//PORTD &= ~(1 << PD5); // set PD5 to LOW
}
void glcd_cs_high(){
	DDRD |= 0b100000;
	//DDRD |= (1 << DDD5); // set PD5 as output
	PORTD |=  0b100000;
	//PORTD |= (1 << PD5); // set PD5 to HIGH
}
void glcd_dc_low(){
	DDRD |= 0b1000000;
	//DDRD |= (1 << DDD6); // set PD6 as output
	PORTD &=~ 0b1000000;
	//PORTD &= ~(1 << PD6); // set PD6 to LOW
}
void glcd_dc_high(){
	DDRD |= 0b1000000;
	//DDRD |= (1 << DDD6); // set PD6 as output
	PORTD |=  0b1000000;
	//PORTD |= (1 << PD6); // set PD6 to HIGH
}

// TODO: determine if this is needed
void glcd_led_off(){
	DDRD |= 0b10000000;
	//DDRD |= (1 << DDD7); // set PD7 as output
	PORTD &=~ 0b10000000;
	//PORTD &= ~(1 << PD7); // set PD7 as LOW
}

// TODO: determine if this is needed
void glcd_led_on(){
	DDRD |= 0b10000000;
	//DDRD |= (1 << DDD7);
	PORTD |=  0b10000000;
	//PORTD |= (1 << PD7);
}

void glcd_rst_off(){
	DDRD |= 0b10000;	// PD4
	//DDRD |= (1 << DDD4); // set PD4 as output
	PORTD |=  0b10000;
	//PORTD |= (1 << PD4); // set PD4 to LOW
}

void glcd_rst_on(){
	DDRD |= 0b10000;
	//DDRD |= (1 << DDD4);
	PORTD &=~ 0b10000;
	//PORTD &= ~(1 << PD4); // set PD4 to HIGH
}

void glcd_sendCmd(unsigned char data){
	glcd_dc_low();
	glcd_cs_low();
	spi_transfer(data);
	//SPI_MasterTransmit(data); // check this function
	glcd_cs_high();
	glcd_dc_high(); // added this
}

void glcd_sendData(unsigned char data){
	glcd_dc_high();
	glcd_cs_low();
	spi_transfer(data);
	//SPI_MasterTransmit(data); // check this function
	glcd_cs_high();
}

void glcd_sendData16(unsigned int data){
	unsigned char data1 = data >> 8;
	unsigned char data2 = data & 0xFF;
	glcd_dc_high();
	glcd_cs_low();
	spi_transfer(data1);
	spi_transfer(data2);
	//SPI_MasterTransmit(data1); // check
	//SPI_MasterTransmit(data2); // check
	glcd_cs_high();
}

void glcd_init(void){
	SPI_MasterInit(); // check
	glcd_cs_high();
	glcd_dc_high();

	glcd_rst_on();
	_delay_ms(10); // wait 10 ms
	glcd_rst_off();

	glcd_setOrientation(PORTRAIT);	// default

	/*glcd_sendCmd(ILI9341_CMD_POWER_ON_SEQ_CONTROL);
	glcd_sendData(ILI9341_CMD_IDLE_MODE_ON);
	glcd_sendData(ILI9341_CMD_MEMORY_WRITE);
	glcd_sendData(ILI9341_CMD_NOP);
	glcd_sendData(ILI9341_CMD_TEARING_EFFECT_LINE_OFF);
	glcd_sendData(0x02); 	// XXX

	glcd_sendCmd(ILI9341_CMD_POWER_CONTROL_B);
	glcd_sendData(ILI9341_CMD_NOP);
	glcd_sendData(ILI9341_CMD_POWER_CONTROL_2);
	glcd_sendData(ILI9341_CMD_PARTIAL_AREA);

	glcd_sendCmd(ILI9341_CMD_DRIVER_TIMING_CONTROL_A);
	glcd_sendData(0x85); 	// XXX
	glcd_sendData(ILI9341_CMD_NOP);
	glcd_sendData(0x78); 	// XXX

	glcd_sendCmd(ILI9341_CMD_DRIVER_TIMING_CONTROL_B);
	glcd_sendData(ILI9341_CMD_NOP);
	glcd_sendData(ILI9341_CMD_NOP);

	glcd_sendCmd(0xED);	// XXX
	glcd_sendData(0x64); 	// XXX
	glcd_sendData(0x03);	// XXX
	glcd_sendData(ILI9341_CMD_PARTIAL_MODE_ON);
	glcd_sendData(0X81); 	// XXX

	glcd_sendCmd(ILI9341_CMD_PUMP_RATIO_CONTROL);
	glcd_sendData(ILI9341_CMD_DISP_INVERSION_OFF);

	glcd_sendCmd(ILI9341_CMD_POWER_CONTROL_1);
	glcd_sendData(0x23);	//VRH[5:0] 	// XXX

	glcd_sendCmd(ILI9341_CMD_POWER_CONTROL_2);
	glcd_sendData(ILI9341_CMD_ENTER_SLEEP_MODE);

	glcd_sendCmd(ILI9341_CMD_VCOM_CONTROL_1);
	glcd_sendData(ILI9341_CMD_READ_MEMORY_CONTINUE);
	glcd_sendData(ILI9341_CMD_DISPLAY_OFF);

	glcd_sendCmd(ILI9341_CMD_VCOM_CONTROL_2);
	glcd_sendData(0x86);	//--	// XXX

	glcd_sendCmd(ILI9341_CMD_MEMORY_ACCESS_CONTROL);
	glcd_sendData(0x48);	//C8	//48 68gal.gal.gal.//28 E8 gal.gal.gal.	// XXX

	glcd_sendCmd(ILI9341_CMD_COLMOD_PIXEL_FORMAT_SET);
	glcd_sendData(ILI9341_CMD_WRITE_CONTENT_ADAPT_BRIGHTNESS);

	glcd_sendCmd(ILI9341_CMD_FRAME_RATE_CONTROL_NORMAL);
	glcd_sendData(ILI9341_CMD_NOP);
	glcd_sendData(0x18); 	// XXX

	glcd_sendCmd(ILI9341_CMD_DISPLAY_FUNCTION_CONTROL);
	glcd_sendData(0x08); 	// XXX
	glcd_sendData(0x82);	// XXX
	glcd_sendData(0x27);	// XXX

	glcd_sendCmd(ILI9341_CMD_ENABLE_3_GAMMA_CONTROL);
	glcd_sendData(ILI9341_CMD_NOP);

	glcd_sendCmd(0x26);	//Gamma curve selected 	// XXX
	glcd_sendData(ILI9341_CMD_SOFTWARE_RESET);

	glcd_sendCmd(ILI9341_CMD_POSITIVE_GAMMA_CORRECTION);
	glcd_sendData(0x0F); 	// XXX
	glcd_sendData(0x31);	// XXX
	glcd_sendData(ILI9341_CMD_PAGE_ADDRESS_SET);
	glcd_sendData(ILI9341_CMD_READ_DISP_PIXEL_FORMAT);
	glcd_sendData(ILI9341_CMD_READ_DISP_SIGNAL_MODE);
	glcd_sendData(0x08); 	// XXX
	glcd_sendData(0x4E); 	// XXX
	glcd_sendData(0xF1); 	// XXX
	glcd_sendData(ILI9341_CMD_VERT_SCROLL_START_ADDRESS);
	glcd_sendData(0x07); 	// XXX
	glcd_sendData(ILI9341_CMD_ENTER_SLEEP_MODE);
	glcd_sendData(0x03);	// XXX
	glcd_sendData(ILI9341_CMD_READ_DISP_SIGNAL_MODE);
	glcd_sendData(ILI9341_CMD_READ_DISP_STATUS);
	glcd_sendData(ILI9341_CMD_NOP);

	glcd_sendCmd(ILI9341_CMD_NEGATIVE_GAMMA_CORRECTION);
	glcd_sendData(ILI9341_CMD_NOP);
	glcd_sendData(ILI9341_CMD_READ_DISP_SIGNAL_MODE);
	glcd_sendData(0x14); 	// XXX
	glcd_sendData(0x03);	// XXX
	glcd_sendData(ILI9341_CMD_SLEEP_OUT);
	glcd_sendData(0x07); 	// XXX
	glcd_sendData(0x31); 	// XXX
	glcd_sendData(ILI9341_CMD_POWER_CONTROL_2);
	glcd_sendData(0x48); 	// XXX
	glcd_sendData(0x08); 	// XXX
	glcd_sendData(0x0F); 	// XXX
	glcd_sendData(ILI9341_CMD_READ_DISP_PIXEL_FORMAT);
	glcd_sendData(0x31); 	// XXX
	glcd_sendData(ILI9341_CMD_MEMORY_ACCESS_CONTROL);
	glcd_sendData(ILI9341_CMD_READ_DISP_SELF_DIAGNOSTIC);

	glcd_sendCmd(ILI9341_CMD_SLEEP_OUT);
	_delay_ms(120); 

	glcd_sendCmd(ILI9341_CMD_DISPLAY_ON);
	glcd_sendCmd(ILI9341_CMD_MEMORY_WRITE);
	//glcd_bg(bgcolor);
	glcd_bg(black);
	*/

    
    glcd_sendCmd(0xEF);
    glcd_sendData(0x03);
    glcd_sendData(0x80);
    glcd_sendData(0x02);

    glcd_sendCmd(0xCF);
    glcd_sendData(0x00);
    glcd_sendData(0XC1);
    glcd_sendData(0X30);

    glcd_sendCmd(0xED);
    glcd_sendData(0x64);
    glcd_sendData(0x03);
    glcd_sendData(0X12);
    glcd_sendData(0X81);

    glcd_sendCmd(0xE8);
    glcd_sendData(0x85);
    glcd_sendData(0x00);
    glcd_sendData(0x78);

    glcd_sendCmd(0xCB);
    glcd_sendData(0x39);
    glcd_sendData(0x2C);
    glcd_sendData(0x00);
    glcd_sendData(0x34);
    glcd_sendData(0x02);

    glcd_sendCmd(0xF7);
    glcd_sendData(0x20);

    glcd_sendCmd(0xEA);
    glcd_sendData(0x00);
    glcd_sendData(0x00);

    glcd_sendCmd(ILI9341_CMD_POWER_CONTROL_1);    //Power control
    glcd_sendData(0x23);   //VRH[5:0]

    glcd_sendCmd(ILI9341_CMD_POWER_CONTROL_2);    //Power control
    glcd_sendData(0x10);   //SAP[2:0];BT[3:0]

    glcd_sendCmd(ILI9341_CMD_VCOM_CONTROL_1);    //VCM control
    glcd_sendData(0x3e);
    glcd_sendData(0x28);

    glcd_sendCmd(ILI9341_CMD_VCOM_CONTROL_2);    //VCM control2
    glcd_sendData(0x86);  //--

    glcd_sendCmd(ILI9341_CMD_MEMORY_ACCESS_CONTROL);    // Memory Access Control
    glcd_sendData(0x48);

    glcd_sendCmd(ILI9341_CMD_VERT_SCROLL_START_ADDRESS); // Vertical scroll
    glcd_sendData(0);                 // Zero

    glcd_sendCmd(ILI9341_CMD_READ_DISP_PIXEL_FORMAT);
    glcd_sendData(0x55);

    glcd_sendCmd(ILI9341_CMD_FRAME_RATE_CONTROL_NORMAL);
    glcd_sendData(0x00);
    glcd_sendData(0x18);

    glcd_sendCmd(ILI9341_CMD_DISPLAY_FUNCTION_CONTROL);    // Display Function Control
    glcd_sendData(0x08);
    glcd_sendData(0x82);
    glcd_sendData(0x27);

    glcd_sendCmd(0xF2);    // 3Gamma Function Disable
    glcd_sendData(0x00);

    glcd_sendCmd(ILI9341_CMD_GAMMA_SET);    //Gamma curve selected
    glcd_sendData(0x01);

    glcd_sendCmd(ILI9341_CMD_POSITIVE_GAMMA_CORRECTION);    //Set Gamma
    glcd_sendData(0x0F);
    glcd_sendData(0x31);
    glcd_sendData(0x2B);
    glcd_sendData(0x0C);
    glcd_sendData(0x0E);
    glcd_sendData(0x08);
    glcd_sendData(0x4E);
    glcd_sendData(0xF1);
    glcd_sendData(0x37);
    glcd_sendData(0x07);
    glcd_sendData(0x10);
    glcd_sendData(0x03);
    glcd_sendData(0x0E);
    glcd_sendData(0x09);
    glcd_sendData(0x00);

    glcd_sendCmd(ILI9341_CMD_NEGATIVE_GAMMA_CORRECTION);    //Set Gamma
    glcd_sendData(0x00);
    glcd_sendData(0x0E);
    glcd_sendData(0x14);
    glcd_sendData(0x03);
    glcd_sendData(0x11);
    glcd_sendData(0x07);
    glcd_sendData(0x31);
    glcd_sendData(0xC1);
    glcd_sendData(0x48);
    glcd_sendData(0x08);
    glcd_sendData(0x0F);
    glcd_sendData(0x0C);
    glcd_sendData(0x31);
    glcd_sendData(0x36);
    glcd_sendData(0x0F);

    glcd_sendCmd(ILI9341_CMD_SLEEP_OUT);    //Exit Sleep
    _delay_ms(120);
    glcd_sendCmd(ILI9341_CMD_DISPLAY_ON);    //Display on
    _delay_ms(120);
}

void glcd_setX(unsigned int x0,unsigned int x1){
	glcd_sendCmd(ILI9341_CMD_COLUMN_ADDRESS_SET);
	glcd_sendData16(x0);
	glcd_sendData16(x1);
}

void glcd_setY(unsigned int y0,unsigned int y1){
	glcd_sendCmd(ILI9341_CMD_PAGE_ADDRESS_SET);
	glcd_sendData16(y0);
	glcd_sendData16(y1);
}

// this function determines if we are using
// PORTRAIT or LANDSCAPE orientation
void glcd_setOrientation(char orientation){
	glcd_orientation = orientation;
	if(glcd_orientation == PORTRAIT){
		max_x = 240;
		max_y = 320;
	}
	else if(glcd_orientation == LANDSCAPE){
		max_x = 320;
		max_y = 240;
	}
}

void glcd_setXY(unsigned int x0, unsigned int y0){
	glcd_setX(x0, x0);
	glcd_setY(y0, y0);
	glcd_sendCmd(ILI9341_CMD_MEMORY_WRITE);
}

void glcd_pixel(unsigned int x0, unsigned int y0, unsigned int color){
	if(glcd_orientation == PORTRAIT){
		glcd_setXY(x0, y0);
		glcd_sendData16(color);
	}
	else if(glcd_orientation == LANDSCAPE){
		glcd_setXY(y0, max_x-x0);
		glcd_sendData16(color);
	}
}

void glcd_line(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1, unsigned int color){
	int x = x1-x0;
	int y = y1-y0;
	int dx = abs(x), sx = x0<x1 ? 1 : -1;
	int dy = -abs(y), sy = y0<y1 ? 1 : -1;
	int err = dx+dy, e2;			// error value e_xy
	for (;;){
		glcd_pixel(x0,y0,color);
		e2 = 2*err;
		if (e2 >= dy){			// e_xy+e_x > 0
			if (x0 == x1)
				break;
			err += dy;
			x0 += sx;
		}
		if (e2 <= dx){	 		// e_xy+e_y < 0
			if (y0 == y1)
				break;
			err += dx;
			y0 += sy;
		}
	}
}

// function for a horizontal line
void glcd_hline(unsigned int x0, unsigned int y0, unsigned int length, unsigned int color){
	if(glcd_orientation == PORTRAIT){
		glcd_setX(x0,x0 + length);
		glcd_setY(y0,y0);
		glcd_sendCmd(ILI9341_CMD_MEMORY_WRITE);
		int i;
		for(i=0; i<length; i++)
			glcd_sendData16(color);
	}
	else if (glcd_orientation ==  LANDSCAPE){
		glcd_setX(y0, y0);
		glcd_setY(max_x-x0-length, max_x-x0);
		glcd_sendCmd(ILI9341_CMD_MEMORY_WRITE);
		int i;
		for(i=0;i<length;i++)
			glcd_sendData16(color);
	}
}

// function for a vertical line
void glcd_vline(unsigned int x0, unsigned int y0, unsigned int length, unsigned int color){
	if(glcd_orientation == PORTRAIT)
	{
		glcd_setX(x0,x0);
		glcd_setY(y0,y0+length);
		glcd_sendCmd(ILI9341_CMD_MEMORY_WRITE);
		int i=0;
		for(i=0; i<length; i++)
			glcd_sendData16(color);
	}
	else if (glcd_orientation ==  LANDSCAPE){
		glcd_setX(y0,y0 + length);
		glcd_setY(max_x-x0,max_x-x0);
		glcd_sendCmd(ILI9341_CMD_MEMORY_WRITE);
		int i=0;
		for(i=0; i<length; i++)
			glcd_sendData16(color);
	}
}

// function for a rectangle
void glcd_rectangle(unsigned int x0, unsigned int y0, unsigned int length, unsigned int width, unsigned int color){
	glcd_hline(x0, y0, length, color);
	glcd_hline(x0, y0+width, length, color);
	glcd_vline(x0, y0, width, color);
	glcd_vline(x0+length, y0, width, color);
}

// function fills the rectangle with a certain color
void glcd_fillRectangle(unsigned int x0, unsigned int y0, unsigned int length, unsigned int width, unsigned int color){
	int i;
	for(i=0; i<width; i++)
		glcd_hline(x0,y0+i,length,color);
}

// function for a circle
void glcd_circle(int x0, int y0, int r, unsigned int color){
	int x = -r, y = 0, err = 2-2*r, e2;
	do {
		glcd_pixel(x0-x, y0+y,color);
		glcd_pixel(x0+x, y0+y,color);
		glcd_pixel(x0+x, y0-y,color);
		glcd_pixel(x0-x, y0-y,color);
		e2 = err;
		if (e2 <= y){
			err += ++y*2+1;
			if (-x == y && e2 <= x)
				e2 = 0;
		}
		if (e2 > x)
			err += ++x*2+1;
	} while (x <= 0);
}

// this function fills the circle with a color
void glcd_fillCircle(int x0, int y0, int r, unsigned int color){
	int x = -r, y = 0, err = 2-2*r, e2;
	do{
		glcd_vline(x0-x, y0-y, 2*y, color);
		glcd_vline(x0+x, y0-y, 2*y, color);

		e2 = err;
		if (e2 <= y){
			err += ++y*2+1;
			if (-x == y && e2 <= x) e2 = 0;
		}
		if (e2 > x)
			err += ++x*2+1;
	} while (x <= 0);
}

// this function creates a character. Takes in the ASCII value
void glcd_char(unsigned char ascii, unsigned int x0, unsigned int y0, unsigned int size, unsigned int fgcolor){
	if((ascii>=32)&&(ascii<=127))	// Characters below 32 are not printable, there are 127 of them
	{
		;	// If printable do nothing
	}	
	else{
		// If not printable write junk
		ascii = '?'-32;		// ascii = 63-32=31
	}
	//glcd_clr(x0, y0, size);	// delete before printing
	int i;
	for (i =0; i<FONT_X; i++){
		// pgm_read_byte - Read a byte from the program space with a 16-bit (near) address. 
		unsigned char temp = pgm_read_byte(&simpleFont[ascii-32][i]);	// because font.c starts at 0, not 32
		unsigned char f=0;
		for(f=0;f<8;f++){
			if((temp>>f)&0x01)
				glcd_fillRectangle(x0+i*size, y0+f*size, size, size, fgcolor);
		}
	}
}

// this function creates a string in the screen. Takes in a character array
void glcd_string(char *string, unsigned int x0, unsigned int y0, unsigned int size, unsigned int fgcolor){
	while(*string){
		glcd_char(*string, x0, y0, size, fgcolor);
		string++;

		if(x0 < MAX_X)
			x0 += FONT_SPACE*size;
	}
}

// this function creates a number
unsigned char glcd_number(unsigned int long_num,unsigned int x0, unsigned int y0,unsigned int size,unsigned int fgcolor){
	unsigned char char_buffer[10] = "";
	unsigned char i = 0;
	unsigned char f = 0;

	if (long_num < 0){
		f=1;
		glcd_char('-',x0, y0, size, fgcolor);
		long_num = -long_num;
			if(x0 < MAX_X)
				x0 += FONT_SPACE*size;
	}
	else if (long_num == 0){
		f=1;
		glcd_char('0',x0, y0, size, fgcolor);
		return f;
			if(x0 < MAX_X)
				x0 += FONT_SPACE*size;
	}

	// XXX If number has multiple digits and is written trough loop then last digit has to be deleted manually before reset 

	while (long_num > 0){
		char_buffer[i++] = long_num % 10;
		long_num /= 10;
	}

	f = f+i;
	for(; i > 0; i--){
		glcd_char('0'+ char_buffer[i - 1],x0, y0, size, fgcolor);
			if(x0 < MAX_X)
				x0 += FONT_SPACE*size;
	}
	return f;
}

// this function creates a float in the screen
unsigned char glcd_float(float floatNumber, unsigned int x0, unsigned int y0, unsigned int size, unsigned int fgcolor){
	unsigned char decimal=1;
	unsigned int temp=0;
	float decy=0.0;
	float rounding = 0.5;
	unsigned char f=0;
	if(floatNumber<0.0){
		glcd_char('-',x0, y0, size, fgcolor);
		floatNumber = -floatNumber;
		if(x0 < MAX_X)
			x0 += FONT_SPACE*size;
		f =1;
	}
	unsigned char i;
	for (i=0; i < decimal; ++i)
		rounding /= 10.0;
	floatNumber += rounding;

	temp = (unsigned int)floatNumber;
	unsigned char howlong=glcd_number(temp,x0, y0, size, fgcolor);
	f += howlong;
	if((x0+8*size*howlong) < MAX_X)
		x0+=FONT_SPACE*size*howlong;
	if(decimal>0){
		glcd_char('.',x0, y0, size, fgcolor);
		if(x0 < MAX_X)
			x0 += FONT_SPACE*size;
		f +=1;
	}
	decy = floatNumber-temp;	// decimal part,
	for(i=0;i<decimal;i++){
		decy *=10;	// for the next decimal
		temp = decy;	// get the decimal
		glcd_number(temp,x0, y0, size, fgcolor);
		floatNumber = -floatNumber;
			if(x0 < MAX_X)
				x0 += FONT_SPACE*size;
		decy -= temp;
	}
	f += decimal;
	return f;
}

// determine what this function does
void glcd_kocka(unsigned int x0, unsigned int y0, unsigned int howmany, unsigned int size, unsigned int fgcolor){
	int j;
	for(j=0;j<howmany;j++){
		int i;
		for (i =0; i<FONT_X; i++){
			unsigned char temp = pgm_read_byte(&simpleFont[96][i]);	// kocka
			unsigned char f=0;
			for(f=0;f<8;f++){
				if((temp>>f)&0x01)
					glcd_fillRectangle(x0+i*size, y0+f*size, size, size, fgcolor);
			}
		}
		if(x0 < MAX_X)
			x0 += FONT_SPACE*size;
	}
}

// this clears the screen
void glcd_clr(unsigned int x0, unsigned int y0, unsigned int size){
	glcd_fillRectangle(x0, y0, size*10, size*10, yellow);
}

// clears a line
void glcd_clrLine(unsigned int y0, unsigned int size){
	glcd_fillRectangle(0, y0-size, max_x, size*10, bgcolor);
}

// this function creates a bar
void glcd_bar(unsigned int x0, unsigned int y0, unsigned int howmany, unsigned int bar_max, unsigned int size, unsigned int fgcolor, unsigned int rectangle_color){
	glcd_rectangle(x0, y0-size, FONT_SPACE*size*bar_max+size, FONT_Y*size+2*size-1, red);
	glcd_kocka(x0, y0, howmany, size, fgcolor);
}

// TODO: determine what this function is used for
void glcd_eq(char *string, unsigned int x0, unsigned int y0, float howmany, unsigned int size, unsigned int fgcolor, unsigned int rectangle_color){
	glcd_string(string,x0,y0,size,fgcolor);

	unsigned int bar_max = 10;	// Not for Soldano
	unsigned int duzina = strlen(string)*size*FONT_SPACE;
	unsigned int nx0 = x0 + duzina;	// Shift for the length of „string"
	//glcd_clr2(nx0,y0,10,size);
	glcd_bar(nx0, y0, howmany, bar_max, size, fgcolor, rectangle_color);

	unsigned int duzina2 = duzina + FONT_SPACE*size*bar_max+4*size;
	unsigned int nnx0 = x0 + duzina2;
	glcd_float(howmany,nnx0,y0,size,fgcolor);
}

// TODO: determine what this function is used for
void glcd_bg(unsigned int color){
	glcd_fillRectangle(0,0,max_x,max_y,color);
}

// this function creates a arc on the screen
void glcd_arc(int x, int y, int r, int startAngle, int endAngle, int thickness, unsigned int color){
        int rDelta = -(thickness/2);
        int px, py, cx, cy; 

        startAngle -= 90; 
        endAngle   -= 90; 
    
        if (startAngle!=endAngle){   
                int i;
                for(i=0; i<thickness; i++){
                        px = x + cos((startAngle*3.14)/180) * (r+rDelta+i);
                        py = y + sin((startAngle*3.14)/180) * (r+rDelta+i);
                        int d;
                        for (d = startAngle + 1; d < endAngle + 1; d++){   
                                cx = x + cos((d*3.14)/180) * (r+rDelta+i);
                                cy = y + sin((d*3.14)/180) * (r+rDelta+i);
				glcd_line(px, py, cx, cy, color);
                                px = cx; 
                                py = cy; 
                        }   
                }   
        }   
        else{   
                px = x + cos((startAngle*3.14)/180) * (r+rDelta);
                py = y + sin((startAngle*3.14)/180) * (r+rDelta);
                cx = x + cos((startAngle*3.14)/180) * (r-rDelta);
                cy = y + sin((startAngle*3.14)/180) * (r-rDelta);
		glcd_line(px, py, cx, cy, color);
        }   
}

// this function creates a title on the screen
void glcd_title(char *string, unsigned int size, unsigned int fgcolor, unsigned int bgcolor){
	unsigned int x0 = max_x/2 - (strlen(string)*size*FONT_SPACE)/2;
	unsigned int y0 = 0.5*size*FONT_SPACE;
	glcd_fillRectangle(0,0,max_x,2*size*FONT_SPACE,bgcolor);
	glcd_string(string,x0,y0,size,fgcolor);
}