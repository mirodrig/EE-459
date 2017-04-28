#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdbool.h>

// define parameters
#define FOSC 9830400 // clock frequency
#define BAUD 9600 // for serial interface
//#define MYUBRR FOSC/16/BAUD-1
#define MYUBRR 47

char buffer[100];
int count = 0;
bool flag = false;

void serial_init(uint8_t ubrr);
void serial_out(char);
char serial_in(void);

int main(void) {
  DDRD |= (1 << PD1); /* Set PD1 for output */ 
  PORTD |= (1 << PD0); /* Set PD0 for input */
  
  //Set Interrupts
  sei();
  
  //Define MYUBRR
  #define FOSC 9830400 // clock frequency
  #define BAUD 9600 // for serial interface
  //#define MYUBRR FOSC/16/BAUD-1
  #define MYUBRR 47
 
  serial_init(MYUBRR);
              // 1 stop bit, 8 data bits
  
  while (1) {
    /*
    if(flag == true){
      buffer[count] = '\0';
      serial_outs(buffer);
      int c;
      for(c = 0; c < 100; c++){
        buffer[c] = '\0';
      }
      count = 0;
      flag = false;
    }
    */
    //serial_out(serial_in());  
  }               // Loop forever
    return 0;   /* never reached */
}

// initialize the serial port. Takes in no parameters.
void serial_init(uint8_t ubrr){
  UBRR0 = ubrr; // set the BAUD rate
  UCSR0B |= (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0); // turn on the transmitter
  //UBRR0H = (MYUBRR >> 8); // load upper 8 bits to high byte
  //UBRR0L = MYUBRR; // load lower 8 bits of BUAD to low byte
  UCSR0C = (1 << UCSZ01) | (3 << UCSZ00); // set for async operation 8 data bits, one stop bit
}

// output a byte to the serial port
void serial_out(char ch){
  while((UCSR0A & (1 << UDRE0)) == 0);
  UDR0 = ch;
}

char serial_in()
{
  while (!(UCSR0A & (1 << RXC0)));
  return UDR0;
}

void serial_outs(char s[]){
  int i = 0;
  while(s[i] != '\0'){
    serial_out(s[i]);
    i++;
  }
}

void storeCh(char ch){
  if(ch == '$'){
    flag = true;
  }
  buffer[count] = ch;
  count = count + 1;
}

ISR(USART_RX_vect){
  storeCh(UDR0);
  serial_out(UDR0);
}