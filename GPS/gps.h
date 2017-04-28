/*************************************
 * gps.h
 * this program is designed to act as
 * a library for GPS related functions
 * **********************************/
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#define FOSC 9830400 // clock frequency
#define BAUD 9600 // for serial
#define MYUBRR 47

struct GPS{
	char buffer[100];
	uint8_t index;
	int quality, satellites;
	float latitude, longitude, altitude; // altitude will not be needed
};

void serial_outputString(char *text);
void convertGPPGA(struct GPS* gps);
void gps_readSerial(struct GPS *gps);
void stringConvert(float f, char *str, char size);
void printData(struct GPS* gps);