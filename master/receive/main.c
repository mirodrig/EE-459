#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdio.h>
#include <string.h> 
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <ctype.h>
#include "../../RFM69/rfm69.h"
#include "../../serial_test/serial.h"
#include "../../serial_display/lcd.h"
#include "../../Barometric_Sensor/MPL3115A2.h"

// define parameters
#define FOSC 9830400 // clock frequency
#define BAUD 9600 // for serial interface
#define MYUBRR 47
//#define BDIV ((FOSC/10000 - 16) / 2)+1 // puts I2C rate just below 100kHz

struct GPS{
    char buffer[100], index;
    int quality, satellites; 
    float latitude, longitude, altitude; 
};

// initialize the serial port. Takes in no parameters.
void gps_serial_init(uint8_t ubrr){
  UBRR0 = ubrr; // set the BAUD rate
  // UCSR0B |= (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0); // turn on the transmitter
  UCSR0B |= (1 << TXEN0) | (1 << RXEN0);
  UCSR0C = (3 << UCSZ00); // set for async operation 8 data bits, one stop bit
}

char gps_serial_in(){
  while (!(UCSR0A & (1 << RXC0)));
  return UDR0;
}

void gps_serial_out(char ch){
  while((UCSR0A & (1 << UDRE0)) == 0);
  UDR0 = ch;
}

void serial_outputString (char* text){
    //char i; 
    int8_t i;
    for (i = 0; i < strlen(text); i++){
        gps_serial_out(text[i]);
    }
    gps_serial_out(0x0D);
    gps_serial_out(0x0A);
}

void convertGPPGA(struct GPS* gps){
    char* section;
    char* GPSString[14];
    //char i = 0;
    int8_t i = 0;
    section = strtok(gps->buffer, ",");
    while(section != NULL){
        GPSString[i++] = section;
        section = strtok(NULL, ",");
        //GPSString[i++] = section;
    }
    
    gps->quality = atoi(GPSString[5]);
    gps->satellites = atoi(GPSString[6]);
    gps->altitude = atof(GPSString[8]);

    //If data invalid, return NULL
    if (gps->quality == 0)
        return;

    //Parse Coordinates
    int degrees;
    float minutes;
    //Latitude
    char lat[3] = {GPSString[1][0], GPSString[1][1]};
    degrees = atoi(lat); 
    minutes = atof(&(GPSString[1][2])) / 60;
    if (strcmp(GPSString[2],"S"))
        gps->latitude = degrees + minutes;
    else
        gps->latitude = degrees + minutes;

    //Longitude
    char log[4] = {GPSString[3][0], GPSString[3][1], GPSString[3][2]};
    degrees = atoi(log); 
    minutes = atof(&(GPSString[3][3])) / 60;
    if (strcmp(GPSString[4],"W"))
        gps->longitude = (degrees + minutes) * -1;
    else
        gps->longitude = (degrees + minutes) * -1;
    return;
}

void readSerial(struct GPS* gps){
    char serialInput;
    int GPGGA = 0;
    while (!(GPGGA == 6 && serialInput == 0x0D)){
        serialInput = gps_serial_in(); 
        if(serialInput == 0x24 & GPGGA == 0) //$ Check
            GPGGA++;
        else if(serialInput == 0x47 && GPGGA == 1) //G Check
            GPGGA++;
        else if(serialInput == 0x50 && GPGGA == 2) //P Check
            GPGGA++;
        else if(serialInput == 0x47 && GPGGA == 3) //G Check
            GPGGA++;
        else if(serialInput == 0x47 && GPGGA == 4) //G Check
            GPGGA++;
        else if(serialInput == 0x41 && GPGGA == 5) //A Check
            GPGGA++;
        else if(GPGGA == 6){
            gps->buffer[gps->index++] = serialInput; 
        }
        else
            GPGGA = 0;
    }
    convertGPPGA(gps); 
    gps->index = 0; 
}

void stringConvert(float f, char *str, char size){
    //char pos = 0;
    int8_t pos = 0;
    char len;
    char* curr;
    int value;

    value = (int)f;
    itoa(value,str,10);

    if (f < 0){
        f *= -1;
        value *= -1;
    }

    len = strlen(str); 
    pos = len;
    str[pos++] = '.';

    while(pos < (size + len + 1)){
        f = f - (float)value;
        f *= 10;
        value = (int)f;
        itoa(value, curr,10);
        str[pos++] = *curr;
    }
 }

void printData(struct GPS* gps){
    lcd_clear();
    char buffer[100];
    stringConvert(gps->latitude, buffer, 3);
    lcd_out(row1_col1, "Latitude:");
    lcd_out(0x0A, buffer);

    stringConvert(gps->longitude, buffer, 3);
    lcd_out(row2_col1, "Longitude:");
    lcd_out(0x4B, buffer);

    lcd_out(row3_col1, "Satellites: ");
    sprintf(buffer, "%d", gps->satellites);
    lcd_out(0x20, buffer);

    lcd_out(0x67, "");
}

struct RFM69 radio; // radio object

void interruptInit(){
	DDRD &= ~(1 <<DDD2) ; 
    PORTD |= (1<<PORTD2); 
    EICRA |= (1<<ISC00) | (1<<ISC01); // set it for rising edge 
    EIMSK |= (1 << INT0); 
    sei();
    PCMSK0 |= 0x80;
}

int main(void){
    lcd_init();
    serial_out(0xFE); // clears the screen
	interruptInit(); // initialize interrupt
	gps_serial_init(MYUBRR);
	SPI_MasterInit(); // initialize SPI
	
	// set pins that are connected to buttons as input
	DDRD &= ~(1 << PD5); // set PD5 (pin 11) as input
	DDRD &= ~(1 << PD6); // set PD6 (pin 12) as input
	DDRD &= ~(1 << PD7); // set PD7 (pin 13) as input
	DDRB &= ~(1 << PB0); // set PB0 (pin 14) as input
	
	// initial GPS parameters
    struct GPS gps;
	gps.index = 0; 
	
	// initial radio parameters
	radio.slaveSelectPin = 16;
    radio.currentMode = 0;
    radio.buffer_length = 0;
    radio.packet_sent = 0;
    
    // reset the RFM69
    DDRC |= (1 << DDC1);
    PORTC &= ~(1 << PC1); // set pin to LOW
    _delay_ms(100);
    PORTC |= (1 << PC1); // set pin to HIGH
    _delay_ms(100);
    
    // initialize the radio
    RFM_spiConfig(); // configure SPI for RFM69
    RFM_init(); // initialize RFM69
	RFM_setMode(&radio.currentMode, 1); // set to RXMODE
	
	// initialize the barometric sensors
	MPLinit();
	_delay_ms(100); // wait (found in the code)
	
    //States
    int state = 3;

    while (1){
	    // if no reception, record own GPS position
		// else{
	       	//cli(); // disable interrupts
	       	// UCSR0B |= (1 << RXCIE0); // enable RX interrupt
        _delay_ms(2000); // sample every 2 sec
    	serial_out(serial_in());
    	readSerial(&gps);
        // 	//printData(&gps);
        // 	UCSR0B &= ~(1 << RXCIE0); // disable RX interrupt
        // 	sei(); // enable interrupts
        // }
        
        // initialize variables for the sensors
        float pascals = getPressure();
        char buffer9[30];
        pascals /= 3377.0;
        FloatToStringNew(buffer9, pascals, 2);
        //snprintf(buffer9, 30, "Pressure: %f In. Hg", pascals);
        
        float alt = getAltitude();
        char buffer10[30];
        FloatToStringNew(buffer10, alt, 2);
        //snprintf(buffer10, 30, "Altitude: %f meters", alt);
        
        float tempC = getTemperature();
        char buffer11[30];
        FloatToStringNew(buffer11, tempC, 2);
        //snprintf(buffer11, 30, "Temp: %f C", tempC);
        
        // State Change poll all pins connected to buttons
        if((PIND & (1 << PD5)) == 0){
            state = 1;
        }
        else if((PIND & (1 << PD6)) == 0){
        	state = 2;
        }
        else if((PIND & (1 << PD7)) == 0){
            state = 3;
        }
        else if((PINB & (1 << PB0)) == 0){
        	state = 4;
        }

        // States
        // if the state is 1, then we will send a msg requesting for other's location
        if(state == 1){
            lcd_clear(); // clear old screen content
            
			if(radio.receiveDataFlag){
				_delay_ms(500);
				radio.receiveDataFlag = 0; // reset the receive flag
				radio.buffer_length = RFM_Read_FIFO(radio.buffer, &radio.currentMode);
				//set to RXMODE after receiving information
				RFM_setMode(&radio.currentMode, 1);
				
				// iterate through the radio to determine what to print on LCD screen
				//int8_t i;
				//char buf1[20];
				//char buf2[20];
				//char buf3[20];
				//char buf4[20];
				//for(i=0; i<sizeof(radio.buffer); i++){
					// TODO: get four separate char[] from the received char[]
					// while there is no digit, print the element onto the LCD screen
				//}
				//lcd_out(row1_col1, );
				//lcd_out(row2_col1, );
				//lcd_out(row3_col1, );
				//lcd_out(row4_col1, );
				
				lcd_out(row1_col1, radio.buffer); // TODO: get it to print correctly
			}
            _delay_ms(5000);
            lcd_clear();
        }
        // if the state is 2, we can display sensor data
        else if(state == 2){
        	lcd_clear(); // clear old screen content
        	lcd_out(row1_col1, "Press.(In. Hg): ");
        	lcd_out(0x10, buffer9);
        	
        	lcd_out(row2_col1, "Altitude(m): ");
        	lcd_out(0x4D, buffer10);
        	
        	lcd_out(row3_col1, "Temp(C): ");
        	lcd_out(0x1D, buffer11);
            _delay_ms(5000);
            lcd_clear();
        }
        else if(state == 3){
            lcd_clear(); // clear old content
            printData(&gps);
            _delay_ms(1000);
        }
        else if(state == 4){
        	lcd_clear(); // clear old content
            lcd_out(row1_col1, "four");
            _delay_ms(5000);
            lcd_clear();
        }
        else
            lcd_out(row1_col1, "menu");
            _delay_ms(1000); // wait 1 sec

    }
    return 0;   /* never reached */
}

ISR(INT0_vect){
	// serial_outputString("interrupt");
	radio.receiveDataFlag = RFM_interruptHandler(radio, &radio.currentMode);
}
