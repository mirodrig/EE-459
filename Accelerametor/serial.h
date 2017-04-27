#ifndef SERIAL_H
#define SERIAL_H


void serial_init(unsigned short ubrr); 
void serial_out(char ch);
char serial_in(); 
void serial_outputString (char* in); 
void FloatToStringNew(char *str, float f, char size);
#endif 