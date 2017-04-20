#ifndef GLCD_H_
#define GLCD_H_

/*******************
 * GLCD: AT8, AT32
 * GND:
 * VDD:
 * CS: PD5, pin 11
 * RST: PD4, pin 6
 * D/C: PD6, pin 12
 * MOSI: PB3, pin 17
 * SCK: PB5, pin 19
 * MISO: PB4, pin 18 (not needed according to library: test this)
*******************/
#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include "color.h"

#define MAX_X   239 // max pixel location in X-direction
#define MAX_Y   319 // max pixel location in Y-direction

#define FONT_SPACE	6
#define FONT_X		8
#define FONT_Y		8

#define PORTRAIT	0
#define LANDSCAPE	1

unsigned int max_x, max_y;
// unsigned int bgcolor = black;

void glcd_cs_low(); //{
	//DDRD |= 0b100000;
	//DDRD |= (1 << DDD5); // set PD5 as output
	//PORTD &=~ 0b100000;
	//PORTD &= ~(1 << PD5); // set PD5 to LOW
//}
void glcd_cs_high(); //{
	//DDRD |= 0b100000;
	//DDRD |= (1 << DDD5); // set PD5 as output
	//PORTD |=  0b100000;
	//PORTD |= (1 << PD5); // set PD5 to HIGH
//}
void glcd_dc_low(); //{
	//DDRD |= 0b1000000;
	//DDRD |= (1 << DDD6); // set PD6 as output
	//PORTD &=~ 0b1000000;
	//PORTD &= ~(1 << PD6); // set PD6 to LOW
//}
void glcd_dc_high(); //{
	//DDRD |= 0b1000000;
	//DDRD |= (1 << DDD6); // set PD6 as output
	//PORTD |=  0b1000000;
	//PORTD |= (1 << PD6); // set PD6 to HIGH
//}

// TODO: determine if this is needed
void glcd_led_off(); //{
	//DDRD |= 0b10000000;
	//DDRD |= (1 << DDD7); // set PD7 as output
	//PORTD &=~ 0b10000000;
	//PORTD &= ~(1 << PD7); // set PD7 as LOW
//}

// TODO: determine if this is needed
void glcd_led_on(); //{
	//DDRD |= 0b10000000;
	//DDRD |= (1 << DDD7);
	//PORTD |=  0b10000000;
	//PORTD |= (1 << PD7);
//}

void glcd_rst_off(); //{
	//DDRD |= 0b10000;	// PD4
	//DDRD |= (1 << DDD4); // set PD4 as output
	//PORTD |=  0b10000;
	//PORTD |= (1 << PD4); // set PD4 to LOW
	//PORTD &= ~(1 << PD4);
//}

void glcd_rst_on(); //{
	//DDRD |= 0b10000;
	//DDRD |= (1 << DDD4);
	//PORTD &=~ 0b10000;
	//PORTD &= ~(1 << PD4); // set PD4 to HIGH
	//PORTD |= (1 << PD4);
//}

const unsigned char simpleFont[][8];
unsigned char glcd_orientation;

void glcd_sendCmd(unsigned char data);
void glcd_sendData(unsigned char data);
void glcd_sendData16(unsigned int data);
void glcd_init(void);
void glcd_setX(unsigned int x0,unsigned int x1);
void glcd_setY(unsigned int y0,unsigned int y1);
void glcd_setOrientation(char orientation);
void glcd_setXY(unsigned int x0, unsigned int y0);
void glcd_line(unsigned int x0,unsigned int y0,unsigned int x1, unsigned int y1, unsigned int color);
void glcd_hline(unsigned int x0, unsigned int y0, unsigned int length, unsigned int color);
void glcd_vline(unsigned int x0, unsigned int y0, unsigned int length, unsigned int color);
void glcd_rectangle(unsigned int x0, unsigned int y0, unsigned int length, unsigned int width, unsigned int color);
void glcd_fillRectangle(unsigned int x0, unsigned int y0, unsigned int length, unsigned int width, unsigned int color);
void glcd_circle(int x0, int y0, int r, unsigned int color);
void glcd_fillCircle(int x0, int y0, int r, unsigned int color);
void glcd_char(unsigned char ascii, unsigned int x0, unsigned int y0,unsigned int size, unsigned int fgcolor);
void glcd_string(char *argstring, unsigned int x0, unsigned int y0, unsigned int size,unsigned int fgcolor);
unsigned char glcd_number(unsigned int long_num,unsigned int x0, unsigned int y0,unsigned int size,unsigned int fgcolor);
unsigned char glcd_float(float floatNumber, unsigned int x0, unsigned int y0, unsigned int size, unsigned int fgcolor);
void glcd_kocka(unsigned int x0, unsigned int y0, unsigned int koliko, unsigned int size, unsigned int fgcolor);
void glcd_clr(unsigned int x0, unsigned int y0, unsigned int size);
void glcd_clrLine(unsigned int y0, unsigned int size);
void glcd_pixel(unsigned int x0, unsigned int y0, unsigned int color);
void glcd_bar(unsigned int x0, unsigned int y0, unsigned int koliko, unsigned int bar_max, unsigned int size, unsigned int fgcolor, unsigned int rectangle_color);
void glcd_eq(char *string, unsigned int x0, unsigned int y0, float koliko, unsigned int size, unsigned int fgcolor, unsigned int rectangle_color);
void glcd_bg(unsigned int color);
void glcd_arc(int x, int y, int r, int startAngle, int endAngle, int thickness, unsigned int color);
void glcd_title(char *string, unsigned int size, unsigned int fgcolor, unsigned int bgcolor);

#endif