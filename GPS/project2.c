#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h> 
#include <stdlib.h>
#include <avr/interrupt.h>
#include <stdbool.h>

// define parameters
#define FOSC 9830400 // clock frequency
#define BAUD 9600 // for serial interface
//#define MYUBRR FOSC/16/BAUD-1
#define MYUBRR 47

struct GPS{
    char buffer[100], index;
    int quality, satellites; 
    float latitude, longitude, altitude; 
};

// initialize the serial port. Takes in no parameters.
void serial_init(uint8_t ubrr){
  UBRR0 = ubrr; // set the BAUD rate
  UCSR0B |= (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0); // turn on the transmitter
  UCSR0C = (3 << UCSZ00); // set for async operation 8 data bits, one stop bit
}

char serial_in(){
  while (!(UCSR0A & (1 << RXC0)));
  return UDR0;
}

void serial_out(char ch){
  while((UCSR0A & (1 << UDRE0)) == 0);
  UDR0 = ch;
}

void serial_outputString (char* text){
    char i; 
    for (i = 0; i < strlen(text); i++){
        serial_out(text[i]);
    }
    serial_out(0x0D);
    serial_out(0x0A);
}

void convertGPPGA(struct GPS* gps){
    char* section;
    char* GPSString[14];
    char i = 0;
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
        serialInput = serial_in(); 
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
    char pos = 0;
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

    while(pos < (size + len + 1)){  // process remaining digits
        f = f - (float)value;  // hack off the whole part of the number
        f *= 10;  // move next digit over
        value = (int)f;  // get next digit
        itoa(value, curr,10); // convert digit to string
        str[pos++] = *curr; // add digit to result string and increment pointer
    }
 }

void printData(struct GPS* gps){
    char buffer[50];
    stringConvert(gps->latitude, buffer, 6);
    serial_outputString("Latitude: ");
    serial_outputString(buffer);

    stringConvert(gps->longitude, buffer, 6);
    serial_outputString("Longitude: ");
    serial_outputString(buffer);

    stringConvert(gps->altitude, buffer, 1);
    serial_outputString("Altitude: ");
    serial_outputString(buffer);

    sprintf(buffer, "Connected Satellites: %d", gps->satellites); 
    serial_outputString(buffer); 
}

int main(void){
	serial_init(MYUBRR);
    struct GPS gps;
	gps.index = 0; 
    while (1){
        _delay_ms(2000);
    	serial_out(serial_in());
    	readSerial(&gps);
        printData(&gps); 
 	}
    return 0;   /* never reached */
}

// void serial_outs(char s[]){
//   int i = 0;
//   while(s[i] != '\0'){
//     serial_out(s[i]);
//     i++;
//   }
// }

// void storeCh(char ch){
//   if(ch == '$'){
//     flag = true;
//   }
//   buffer[count] = ch;
//   count = count + 1;
// }

// ISR(USART_RX_vect){
//   storeCh(UDR0);
//   serial_out(UDR0);
// }