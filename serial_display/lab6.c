/********************************************
*
*  Name: SORUSH RAEISIAN
*  Section: 31007
*  Assignment: Lab 6 - Write to LCD display
*
********************************************/

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>

void init_lcd(void);
void stringout(char *);
void moveto(unsigned char);

void writecommand(unsigned char);
void writedata(unsigned char);
void writenibble(unsigned char);

int main(void) {

DDRD |= ( (1 << PD7) | (1 << PD6) | (1 << PD5) | (1 << PD4));   // set PD[4:7] as output   
DDRB |= ((1 << PB0) | (1 << PB1));                             // set PB[0:1] as output
	
ADMUX |= (1<< REFS0);       // set high voltage source to AVCC
ADMUX |= (1<< ADLAR);       // Left adjusted for 8 bit results        
    
ADCSRA|= (1<< ADPS0);       // choose 128 as the prescaler divisor value
ADCSRA |= (1<< ADPS1);
ADCSRA |= (1<< ADPS2);
ADCSRA |= (1<< ADEN);       //ready to initiate a conversion                             
	
char class[16];
char voltage [16];
unsigned char section;
int number = 0;               
int oldNumber=2;
	
char *name = "EE";               //name variable is "EE"
char *day1 = "T";				 //day1 variable is "T" 
char *day2 = "TH";				 //day2 variable is "TH"
section = 109;
	
init_lcd();				         //start the lcd initialization 
	
moveto(0x00);                    //move cursor to the first column, first row
stringout("i love EE");     // show "Sorush Raeisian"
	
snprintf(class, 16, "class:%s%d%s/%s", name, section, day1, day2);   // class = name,class,day1,day2
	
moveto(0x40);                      //move cursor to the first column, second row
stringout(class);                   // show Class
	
_delay_ms(2000);                   // wait for 2000ms
   
writecommand(0x01);               //Clear the LCD screen
   
moveto(0x00);                     //move cursor to the first column, first row
  
	while (1) 
	 {
     
   	(ADCSRA |= (1<< ADSC));                                   // Start an ADC conversion
   			
   				while((ADCSRA & 1<<ADSC)==(0x40)){ }       //Read ADCSRA, If ADSC is a one, stay in the loop,If ADSC is a zero, break out of the loop

   	if( ADCH > 180 && ADCH < 220 ){ number= 0;}               // if button select is pressed, set the number to zero
   	else if( ADCH > 120 && ADCH < 179 ){ number++;}           // if button left is pressed, add 1 to the number
   	else if( ADCH > -50 && ADCH < 20 ){ number--;}            // if button right is pressed, subtract 1 from the number
   	else if( ADCH > 21 && ADCH < 70 ){ number+=10;}           // if button up is pressed, add 10 to the number
   	else if( ADCH > 71 && ADCH < 119 ){ number-=10;}          // if button down is pressed, subtract 10 from the number

   
   	snprintf(voltage,16,"value: %d",number);          // voltage : number                        
 
  	
   	if(number != oldNumber)            // if number is not equal to old number
   	{   
   	moveto(0x03);                     //move cursor to the third column, first row
   	writecommand(0x01);               //Clear the LCD screen
   	stringout(voltage);               // show voltage on lcd
   	_delay_ms(180);                   // wait for 180ms
   	oldNumber = number;              
   	}
	}
   
   
	
    

return 0;   /* never reached */
}

		/*
  		init_lcd - Do various things to initialize the LCD display
		*/
		void init_lcd()
		{
   		 _delay_ms(15);              // Delay at least 15ms
  		  writenibble(0x30);         // Use writenibble to send 0011
   		 _delay_ms(5);               // Delay at least 4msec
   		 writenibble(0x30);          // Use writenibble to send 0011
   		 _delay_us(120);             // Delay at least 100usec
   		 writenibble(0x30);          // Use writenibble to send 0011, no delay needed
    	 writenibble(0x20);          // Use writenibble to send 0010    // Function Set: 4-bit interface
   		 _delay_ms(2);
    
    	 writecommand(0x28);         // Function Set: 4-bit interface, 2 lines

         writecommand(0x0f);         // Display and cursor on

		}
		

		/*
  		stringout - Print the contents of the character string "str"
  		at the current cursor position.
		*/
		void stringout(char *str)
		{
			while((*str)!='\0')      // if str variable is not zero, add one to it
			{
			writedata(*str);
			str++;
			}
		}
		

		/*
  		moveto - Move the cursor to the postion "pos"
		*/
		void moveto(unsigned char pos)
		{
		writecommand(0x80+pos);       // cursor position= 0x00 + position on screen
		}
		

		/*
  		writecommand - Output a byte to the LCD display instruction register.
		*/
		void writecommand(unsigned char x)
		{
		PORTB &= ~(1<<PB0);        //Set Rs=0, send command
		writenibble(x);            
		x &= 0x0f;
		x = (x << 4);
		writenibble(x);
		_delay_ms(2);              // wait 2ms
		}


		/*
  		writedata - Output a byte to the LCD display data register
		*/
		void writedata(unsigned char x)
		{
		PORTB |= (1<<PB0);        //Set Rs=1, send data
		writenibble(x);
		x &= 0x0f;
		x = (x << 4);                 
		writenibble(x);
		_delay_ms(2);             // wait 2ms
		}

		/*
  		writenibble - Output four bits from "x" to the display
		*/
		void writenibble(unsigned char x)
		{
		PORTD &= 0x0f;           
		x &= 0xf0;
		PORTD |= x;              // put upper 4 bit of x on upper 4 bits of PORTD
		PORTB |= (1<<PB1);      // set E to 1
		PORTB |= (1<<PB1);      // set E to 1
		PORTB &= ~(1<<PB1);     // set E to 0
	
	}
