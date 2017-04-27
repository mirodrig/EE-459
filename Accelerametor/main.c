#include <stdio.h>
#include <util/delay.h>
#include <avr/io.h> 
#include <avr/interrupt.h>
#include <math.h> 
#include "i2c.h"
#include "LSM9DS0.h"
#include "serial.h"
#include <util/twi.h>
#include <stdint.h>
#include <float.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
int steps = 0; 
int count =2;


int main(int argc, const char * argv[]) {
	serial_init(47);
	
	DDRD &= ~(1 <<DDD3) ; 
  PORTD |= (1<<PORTD3); 
  EICRA |= (1<<ISC00) | (1<<ISC10); 
  EIMSK |= (1 << INT1); 
  cli();  
  sei();
	
	LSM_begin();
	_delay_ms(1000);
	while(1)
	{
		_delay_ms(500);
		if(count < steps)
		{
			serial_outputString("\r\nMOVING\r\n");
			count = steps;
			_delay_ms(500);

		}
		else
		{
			serial_outputString("\r\nNOT MOVING\r\n");
		}

	}

    return 0;
}


//Hardware interrupt
ISR(INT1_vect)
{
	
	  steps++ ; 
	 
	char buf[20] ; 
	sprintf(buf,"steps : %d",steps);
	serial_outputString(buf);

		_delay_ms(20);
		
}
