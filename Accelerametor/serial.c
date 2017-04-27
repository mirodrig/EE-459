//initialize the USART port
#include <avr/io.h>
#include <util/delay.h>
#include <string.h> 

#include "serial.h"

// void serial_init(unsigned short ubrr)	{
// 	UBRR0 = ubrr;
// 	UCSR0B |= (1 << TXEN0);
// 	UCSR0B |= (1 << RXEN0);
// 	UCSR0C = (3 << UCSZ00);
// }
// //output a byte to the USART0 port
// void serial_out(char ch)	{
// 	while ((UCSR0A & (1 << UDRE0)) == 0);
// 	UDR0 = ch;
// }
// //read a byte from the USART0 and return it
// char serial_in()	{
// 	while (!(UCSR0A & (1<<RXC0)));
// 	return UDR0;
// }

// void serial_outputString (char* in)
// {
// 	char i = 0; 
// 	for (i = 0; i <= strlen(in) ; i ++)
// 	{
// 		serial_out(in[i]);
// 	}
// 	serial_out(0x0D);
//     serial_out(0x0A);
// }

void FloatToStringNew(char *str, float f, char size)

{

    char pos;  // position in string

    char len;  // length of decimal part of result

    char* curr;  // temp holder for next digit

    int value;  // decimal digit(s) to convert

    pos = 0;  // initialize pos, just to be sure

    value = (int)f;  // truncate the floating point number
    itoa(value,str,10);  // this is kinda dangerous depending on the length of str
    // now str array has the digits before the decimal

    if (f < 0 )  // handle negative numbers
    {
        f *= -1;
        value *= -1;
    }

     len = strlen(str);  // find out how big the integer part was
    pos = len;  // position the pointer to the end of the integer part
    str[pos++] = '.';  // add decimal point to string

    while(pos < (size + len + 1) )  // process remaining digits
    {
        f = f - (float)value;  // hack off the whole part of the number
        f *= 10;  // move next digit over
        value = (int)f;  // get next digit
        itoa(value, curr,10); // convert digit to string
        str[pos++] = *curr; // add digit to result string and increment pointer
    }
 }