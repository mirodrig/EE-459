#include <avr/io.h>
#include <util/twi.h>
#include <stdint.h>
#include "MPL3115A2.h"
#include "I2C.h"
#include "sci.h"
#include <avr/Interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "serial.h"
#include <math.h>

//#define MPL3115A2_I2C_ADDRESS 0xC0
#define FOSC 9830400            // Clock frequency = Oscillator freq.
#define BAUD 9600               // UART0 baud rate
#define MYUBRR 47   // Value for UBRR0 register
#define BDIV ((FOSC / 100000 - 16) / 2)+ 1    // Puts I2C rate just below 100kHz

int main() 
{
    sci_init(MYUBRR);
	  MPLinit();
    _delay_ms(100);
   while(1){
          float pascals = getPressure();
          char buffer9[50];
          pascals /= 3377.0;
          FloatToStringNew(buffer9,pascals, 2);
          sci_outs("\r\nPressure: ");
          sci_outs(buffer9);
          sci_outs(" Inches HG");
      
          float alt = getAltitude();
          char buffer10[50];
           FloatToStringNew(buffer10,alt, 2);
          sci_outs("\r\nAltitude: ");
          sci_outs(buffer10);
          sci_outs(" Meters");

          float tempC = getTemperature();
          char buffer11[50];
           FloatToStringNew(buffer11,tempC, 2);
          sci_outs("\r\nTemp: ");
          sci_outs(buffer11);
          sci_outs(" C");
           

   }
    return 0;
}

 
