/******************************************
 * main.c
******************************************/

//#include "glcd.c"
#include "glcd.h"

void glcd_test(){
	glcd_string("abcd",180,0,2,green);
	glcd_fillRectangle(20,10,100,20,red);
	glcd_fillCircle(20,60,20,blue);
	glcd_char('a',0,80,2,red);
	glcd_pixel(10,10,green);
	//glcd_kocka(180,40,3,2,yellow);
	glcd_bar(10,160,2,3,8,blue,red);
	glcd_float(-123.56,120,60,2,blue);
	glcd_number(456,200,60,2,white);
	glcd_line(0,0,max_x,max_y,green);
	glcd_line(max_x,0,0,max_y,green);
	glcd_eq("Gain ",20,100,7.5, 2, blue, red);
	glcd_rectangle(0,0,240,240,green);
}

void glcd_test2(){
	glcd_title("Naslov",2,white,gray);
	glcd_pixel(max_x/2,max_y/2,red);
	glcd_arc(max_x/2,max_y/2,50,0,90,8,yellow);
	glcd_arc(max_x/2,max_y/2,50,90,180,8,magenta);
	glcd_arc(max_x/2,max_y/2,50,180,270,8,yellow);
	glcd_arc(max_x/2,max_y/2,50,270,360,8,magenta);
	glcd_arc(max_x/2,max_y/2,46,0,360,2,blue);
	glcd_arc(max_x/2,max_y/2,55,0,360,2,blue);
	glcd_arc(max_x/2,max_y/2,46,0,360,2,black);
	glcd_arc(max_x/2,max_y/2,55,0,360,2,black);

	glcd_arc(max_x/2,max_y/2,50,0,90,8,red);
	glcd_arc(max_x/2,max_y/2,50,90,180,8,blue);
	glcd_arc(max_x/2,max_y/2,50,180,270,8,red);
	glcd_arc(max_x/2,max_y/2,50,270,360,8,blue);
	glcd_arc(max_x/2,max_y/2,46,0,360,2,green);
	glcd_arc(max_x/2,max_y/2,55,0,360,2,green);
}

void glcd_test0(){
	// set screen to landscape and perform initial test
	glcd_setOrientation(LANDSCAPE);
	glcd_test();
	_delay_ms(2000);
	unsigned int bgcolor = black;
	glcd_bg(bgcolor);

	// set screen to portrait and perform initial test
	//glcd_setOrientation(PORTRAIT);
	//glcd_test();
	//_delay_ms(2000);
	//glcd_bg(bgcolor);

	//glcd_setOrientation(LANDSCAPE);
	//glcd_test2();
	//glcd_bg(bgcolor);

	//glcd_setOrientation(PORTRAIT);
	//glcd_test2();
	//glcd_bg(bgcolor);
}

int main() {
	// enable the screen RESET and the disable
	//DDRD |= (1 << DDD4); // set PD4 as output
	//PORTD &= ~(1 << PD4);
	//_delay_ms(100);
	//PORTD |= (1 << PD4);
	//_delay_ms(100);
	glcd_init();
	//glcd_led_on();



	DDRB |= (1 << DDB2);
	PORTB |= (1 << PB2);

	//CS: PD5, pin 11
	//DDRD |= (1 << DDD5);
	//PORTD &= ~(1 << PD5);

	while(1){
		//glcd_test0();
	}
	return 0;
}