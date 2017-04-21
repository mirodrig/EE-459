/*************************************
 * gps.h
 * this program is designed to act as
 * a library for GPS related functions
 * **********************************/
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "../serial_test/serial.h"

#define FOSC 9330400 // clock frequency
#define BAUD 9600 // for serial
#define MYUBRR 47

struct GPS{
	char buffer[100], index;
	int quality, satellites;
	float latitude, longitude, altitude; // altitude will not be needed
}

void serial_outputString(char *text){
	char i;
	for(i=0; i<strlen(text); i++){
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
		// GPSString[i++] = section;
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

void gps_readSerial(struct GPS *gps){
	char serialInput;
	int GPGGA = 0;
	while(!(GPGGA == 6 && serialInput == 0x0D)){
		serialInput = serial_in();
		if(serialInput == 0x24 & GPGGA == 0) // $ check
			GPGGA++;
		else if(serialInput == 0x47 & GPGGA == 1) // G check
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
	while(pos < (size + len + 1)){
		f = f - (float)value;
		f *= 10;
		value = (int)f;
		itoa(value, curr,10);
		str[pos++] = *curr;
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
