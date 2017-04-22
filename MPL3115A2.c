#include <avr/io.h>
#include <util/twi.h>
#include <stdint.h>
#include <float.h>
#include "MPL3115A2.h"
#include "I2C.h"
#include "sci.h"
#include <util/delay.h>
# include <stddef.h>
#include <avr/Interrupt.h>
#include <stdlib.h>
#include <string.h>
#include "serial.h"

#define FOSC 9830400            // Clock frequency = Oscillator freq.
#define BAUD 9600               // UART0 baud rate
#define MYUBRR FOSC/16/BAUD-1   // Value for UBRR0 register
//#define BDIV (((FOSC / 100000 - 16) / 2 )+ 1)    // Puts I2C rate just below 100kHz
# define BDIV (((FOSC / 100000) - 16) / 2) + 1

void MPLinit()
{
    i2c_init();
    write8(0xC0,0X26,0xB8);
    write8(0xC0,0X13,0X07);
}

float getPressure(void)
{
  write8(SlaveAddressIIC, 0x26, 0x39);  
  unsigned char sta[1];
  unsigned char reg[1] = {0x00};
  while (!(sta[0]& 0x08)) {
    read8(0xC0, reg,1, sta,1);
    _delay_ms(10);
  }
   uint8_t pressure_MSB[1];
   uint8_t pressure_reg_MSB[1] = {0x01};
   read8(SlaveAddressIIC,pressure_reg_MSB,(uint16_t)1, pressure_MSB,(uint16_t)1);

   uint8_t pressure_CSB[1];
   uint8_t pressure_reg_CSB[1] = {0x02};
   read8(SlaveAddressIIC,pressure_reg_CSB,(uint16_t)1, pressure_CSB,(uint16_t)1); 

   uint8_t pressure_LSB[1];
   uint8_t pressure_reg_LSB[1] = {0x03};
   read8(SlaveAddressIIC,pressure_reg_LSB,(uint16_t)1, pressure_LSB,(uint16_t)1); 
    
   uint32_t l = pressure_MSB[0];
   l <<= 8;
   l |= pressure_CSB[0];
   l <<= 8;
   l |= pressure_LSB[0];
   l >>= 4;
   float baro = (float)l;
   baro /= 4.0;
  return baro;

}

float getAltitude(void)
{
	int32_t alt;
  write8(SlaveAddressIIC,0x26,0xB9);
  unsigned char sta[1];
  unsigned char reg[1] = {0x00};
  while (! (sta[0] & 0x08)) {
    read8(0xC0, reg,1, sta,1);
    _delay_ms(10);
  }
   uint8_t pressure_MSB[1];
   uint8_t pressure_reg_MSB[1] = {0x01};
   read8(SlaveAddressIIC,pressure_reg_MSB,(uint16_t)1, pressure_MSB,(uint16_t)1);

   uint8_t pressure_CSB[1];
   uint8_t pressure_reg_CSB[1] = {0x02};
   read8(SlaveAddressIIC,pressure_reg_CSB,(uint16_t)1, pressure_CSB,(uint16_t)1); 

   uint8_t pressure_LSB[1];
   uint8_t pressure_reg_LSB[1] = {0x03};
   read8(SlaveAddressIIC,pressure_reg_LSB,(uint16_t)1, pressure_LSB,(uint16_t)1);
  
   alt = pressure_MSB[0];
   alt <<= 8;
   alt |= pressure_CSB[0];
   alt <<= 8;
   alt |= pressure_LSB[0];
   alt >>= 4;

  if (alt & 0x80000) {
    alt |= 0xFFF00000;
  }

  float altitude = (float)alt;
  altitude /= 16.0;
  altitude += 20.0;
  
  return altitude;
}
float getTemperature(void)
{
	int16_t t;

  write8(SlaveAddressIIC,0x26,0xB9);
  unsigned char sta[1];
  unsigned char reg[1] = {0x00};
  while (! (sta[0] & 0x08)) {
    read8(0xC0, reg,1, sta,1);
    _delay_ms(10);
  }
  
  uint8_t T_MSB[1];
   uint8_t T_reg_MSB[1] = {0x04};
   read8(SlaveAddressIIC,T_reg_MSB,(uint16_t)1, T_MSB,(uint16_t)1);
  
  uint8_t T_LSB[1];
   uint8_t T_reg_LSB[1] = {0x05};
   read8(SlaveAddressIIC,T_reg_LSB,(uint16_t)1, T_LSB,(uint16_t)1);
  t = T_MSB[0];
  t <<=8;
  t |= T_LSB[0];
  t >>= 4;

  float temp = (float)t;
  temp /= 16.0;
  return temp;
}
void write8(char slave, char reg, char data)
{
	i2c_start(slave);
	i2c_write(reg);
	i2c_write(data);
	i2c_stop();
}

uint8_t read8(uint8_t device_addr,uint8_t *a,uint16_t size_a ,uint8_t *p, uint16_t n)
{
    uint8_t status;
    status = i2c_io(device_addr, a, size_a, NULL, 0,p, n);
    return(status);
}

