#include <avr/io.h>
#include <avr/Interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
//#include "twi.h"
#include "MPL3115A2.h"
#include "I2C.h"
#include <stdbool.h>
#include "sci.h"
#include <util/twi.h>

void sci_init(uint8_t ubrr) {
    UBRR0 = ubrr;            // Set baud rate
    UCSR0B |= (1 << TXEN0);  // Turn on transmitter
    UCSR0C = (1<<UCSZ01) | (3 << UCSZ00);  // Set for asynchronous operation, no parity, 
                             // one stop bit, 8 data bits
}

/*
  sci_out - Output a byte to SCI port
*/
void sci_out(char ch)
{
    while ((UCSR0A & (1<<UDRE0)) == 0);
    UDR0 = ch;
}

/*
  sci_outs - Print the contents of the character string "s" out the SCI
  port. The string must be terminated by a zero byte.
*/
void sci_outs(char *s)
{
    char ch;

    while ((ch = *s++) != (char) '\0')
        sci_out(ch);
}
