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
#include "../../serial_display/lcd.h"
#include "../../Barometric_Sensor/MPL3115A2.h"
#include "../../Accelerametor/LSM9DS0.h"

#define pi 3.14159265358979323846 // do we need it???

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

void printData(struct GPS* gps, char buffer[]){
    //lcd_clear();
    //char buffer[100];
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

double deg2rad(double);
double rad2deg(double);

double distance(double lat1, double lon1, double lat2, double lon2){
  double theta, dist;
  theta = lon1 - lon2;
  dist = sin(deg2rad(lat1)) * sin(deg2rad(lat2)) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(theta));
  dist = acos(dist);
  dist = rad2deg(dist);
  dist = dist * 60 * 1.1515; /// units are in miles
  return (dist);
}

double deg2rad(double deg) {
  return (deg * pi / 180);
}

double rad2deg(double rad) {
  return (rad * 180 / pi);
}

struct RFM69 radio; // radio object
int steps = -2;
int count = 0;
int state;

void interruptInit(){
	DDRD &= ~(1 << DDD2);
	DDRD &= ~(1 << DDD3);
	PORTD |= (1<<PORTD2);
	PORTD |= (1<<PORTD3); 
    EICRA |= (1<<ISC00) | (1<<ISC01) | (1<<ISC10); // set it for rising edge 
    EIMSK |= (1 << INT0) | (1 << INT1); 
    cli(); // just added
    sei();
    PCMSK0 |= 0x80;
}

int main(void){
    lcd_init();
    serial_out(0xFE); // clears the screen
	interruptInit(); // initialize interrupt
	gps_serial_init(MYUBRR);
	SPI_MasterInit(); // initialize SPI
	LSM_begin(); // start accelerometer
	
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
    state = 5; // this way, "menu" will be displayed
    lcd_clear();
    lcd_clear();
    _delay_ms(500);
    lcd_out(row1_col1, "Welcome");
    _delay_ms(3000);
    lcd_clear();

    while (1){
        _delay_ms(500); // sample every 2 sec
    	serial_out(serial_in());
    	readSerial(&gps);

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
            //lcd_clear();
			if(radio.receiveDataFlag){
				_delay_ms(500);
				char buf1[100] = "32.0204";
				char buf2[100] = "-118.2891";
				double lat2 = 34.0204;
				double long2 = -121.2891;
				
				radio.receiveDataFlag = 0; // reset the receive flag
				radio.buffer_length = RFM_Read_FIFO(radio.buffer, &radio.currentMode);
				//set to RXMODE after receiving information
				RFM_setMode(&radio.currentMode, 1);
				
				// iterate through the radio to determine what to print on LCD screen
				lcd_out(row1_col1, "Friend's Latitude:");
				lcd_out(row2_col1, buf1);
				lcd_out(row3_col1, "Friend's Longitude:");
				lcd_out(row4_col1, buf2);
				
				// lat1, long1, lat2, long2
				double dist = distance((double)gps.latitude, (double)gps.longitude, lat2, long2);
				char A[30];
				FloatToStringNew(A,(float)dist,2);
				_delay_ms(3000);
				lcd_clear();
				_delay_ms(100);
				lcd_out(row1_col1, A);
				lcd_out(0x09, "miles");
				//lcd_out(row1_col1, radio.buffer); // TODO: get it to print correctly
                lcd_out(0x67, "");
			}
            _delay_ms(1000);
        }
        // if the state is 2, we can display sensor data
        else if(state == 2){
        	lcd_clear(); // clear old screen content
        	//lcd_clear();
            // initialize variables for the sensors
        	cli();
        	float pascals = getPressure();
        	char buffer9[100];
        	pascals /= 3377.0;
        	FloatToStringNew(buffer9, pascals, 2);
        	//cli();
        	float alt = getAltitude();
        	char buffer10[100];
        	FloatToStringNew(buffer10, alt, 2);
        
        	float tempC = getTemperature();
        	char buffer11[100];
        	FloatToStringNew(buffer11, tempC, 2);
        
       		//_delay_ms(200);
        	lcd_out(row1_col1, "Press.(mmHg): ");
        	lcd_out(0x0E, buffer9);
        	
        	lcd_out(row2_col1, "Altitude(m): ");
        	lcd_out(0x4D, buffer10);
        	
        	lcd_out(row3_col1, "Temp(C): ");
        	lcd_out(0x1D, buffer11);
            lcd_out(0x67, "");
        	_delay_ms(1000);
            sei();
        }
        else if(state == 3){
            lcd_clear(); // clear old content
            //lcd_clear();
            cli();
            char buffer[100];
            printData(&gps, buffer);
            _delay_ms(1000);
            sei();
        }
        else if(state == 4){
        	lcd_clear(); // clear old content
        	//lcd_clear();
        	//_delay_ms(500);
            if(count < steps){
            	lcd_out(row1_col1, "MOVING");
                count = steps;
                _delay_ms(500);
            }
            else{
            	lcd_out(row1_col1, "NOT MOVING");
			}
			char stepBuf[30] ; 
    		sprintf(stepBuf,"steps : %d",steps);
			lcd_out(row2_col1, stepBuf);
            lcd_out(0x67, "");
            _delay_ms(1000);
        }
        else if(state == 5){
            lcd_out(row1_col1, "Menu");
            lcd_out(0x67, "");
            _delay_ms(3000); // wait 1 sec
            state = 0;
        }
        else
            lcd_out(0x67, "");
            _delay_ms(100);
    }
    return 0;   /* never reached */
}

ISR(INT0_vect){
	// serial_outputString("interrupt");
	radio.receiveDataFlag = RFM_interruptHandler(radio, &radio.currentMode);
}

ISR(INT1_vect){
	steps++ ;  
    char buf[30] ; 
    sprintf(buf,"steps : %d",steps);
    if(state == 4){
    	lcd_out(row2_col1, buf);
    }
    //serial_outputString(buf);
	_delay_ms(20);            
}
