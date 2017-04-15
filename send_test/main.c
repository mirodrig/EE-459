#include <stdio.h>
#include <util/delay.h>
#include <avr/io.h> 
#include <avr/interrupt.h>

#include "../RFM69/rfm69.h"
// #include "spi.h"
#include "../serial_test/serial.h"

#define Serial_rate 47

void interruptInit(){
	DDRD &= ~(1 <<DDD2) ; 
	PORTD |= (1<<PORTD2); 
	EICRA |= (1<<ISC00) | (1<<ISC01); // set it for rising edge 
	EIMSK |= (1 << INT0); 
	sei(); 
	PCMSK0 |= 0x80;
}

struct RFM69 radio; 

int main(void){
	interruptInit();  // Int errupts
	serial_init(Serial_rate); //Serial 
	SPI_MasterInit(); // SPI
	// spi_init_master(); // SPI 
	
	// Radio Initalize and constants 
	radio.slaveSelectPin = 16; 
	radio.currentMode = 0;
	radio.buffer_length = 0;
	radio.packet_sent = 0; 	
	
	// reset the transceiver
	DDRC |= 1 << DDC1;// set pin to output
	PORTC &= ~(1 << PC1); // set pin to LOW
	_delay_ms(100); // wait
	PORTC |= (1 << PC1); // set pin to HIGH
	_delay_ms(100); // wait
	
	RFM_spiConfig();
	RFM_init();

	sei();
	char message[] = "What's up";
	char size = sizeof(message);

	while (1){
		// serial_outputString(radio.buffer);
		RFM_send(message, &radio.currentMode, size);
		_delay_ms(10);
	}
    return 0;
}

//Hardware interrupt
ISR(INT0_vect){
	//serial_outs("\rInterrupt\n\r");
	// set to idle, needs to do this in order to know the package was sent. 
	radio.packet_sent = RFM_interruptHandler(radio, &radio.currentMode);
}

