/******************************************
  * author: Marlin Ivan Rodriguez
  * main.c
  * this program integrates the RF transceiver
  * with the LCD display.
******************************************/
#include <stdio.h>
#include <avr/io.h> 
#include <util/delay.h>
#include <avr/interrupt.h>
#include "../RFM69/rfm69.h"
#include "../serial_display/lcd.h"
#include "../serial_test/serial.h"

#define Serial_rate 47

struct RFM69 radio; // rfm69 object

void interruptInit(){
    DDRD &= ~(1 <<DDD2) ; 
    PORTD |= (1<<PORTD2); 
    EICRA |= (1<<ISC00) | (1<<ISC01); // set it for rising edge 
    EIMSK |= (1 << INT0); 
    sei();
    PCMSK0 |= 0x80;
}

int main(){
    // hardware initialization
    interruptInit(); // initialize interrupts
    serial_init(Serial_rate); // initialize serial
    SPI_MasterInit(); // initialize SPI for RFM69
    lcd_init(); // initializes LCD screen

    // Radio Initalize and constants 
    radio.slaveSelectPin = 16; 
    radio.currentMode = 0; 
    radio.buffer_length = 0;
    radio.packet_sent = 0;  
        
    // reset the pin
    DDRC |= 1 << DDC1;// set pin to output
    PORTC &= ~(1 << PC1); // set pin to LOW
    _delay_ms(100); // wait
    PORTC |= (1 << PC1); // set pin to HIGH
    _delay_ms(100); // wait
    
    // initialize RFM69
    RFM_spiConfig();
    RFM_init();

    lcd_out(row1_col1, "starting radio");
    _delay_ms(1000); // wait 1 sec
    lcd_clear();
    //serial_outs("\rinitializing receiver\n\r");

    RFM_setMode(&radio.currentMode,1); // set to RXMODE

    while(1){
        if(radio.receiveDataFlag){
            radio.receiveDataFlag = 0; // reset the receive flag
            radio.buffer_length = RFM_Read_FIFO(radio.buffer, &radio.currentMode);
            // set to RXMODE after receiving information
            RFM_setMode(&radio.currentMode, 1);
            
            lcd_out(row1_col1, radio.buffer);
            _delay_ms(2000);
            lcd_clear();

            //serial_out('\r');
            //serial_outs(radio.buffer);
            //serial_outs("\n\r");
        }
        _delay_ms(1000);
    }
    return 0;
}
//Hardware interrupt
ISR(INT0_vect){
    //serial_outs("interrupt");
    radio.receiveDataFlag = RFM_interruptHandler(radio, &radio.currentMode);
}
