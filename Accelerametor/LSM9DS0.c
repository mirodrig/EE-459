#include "LSM9DS0.h"
#include "i2c.h"
#include <util/twi.h>
#include <stdint.h>
#include <float.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h> 

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/



void LSM_begin()
{
  
  i2c_init(); // initiate i2c ;
  write8(LSM9DS1_ADDRESS_ACCELGYRO_WRITE,LSM9DS1_REGISTER_CTRL_REG8, 0x05);
  _delay_ms(10);
  write8(LSM9DS1_ADDRESS_ACCELGYRO_WRITE,CTRL_REG5_XL,0x38); 
  write8(LSM9DS1_ADDRESS_ACCELGYRO_WRITE,CTRL_REG6_XL,0xC0);
  //set interrupts for x and y events 
  write8(LSM9DS1_ADDRESS_ACCELGYRO_WRITE,0x06,0x0A); 
  //Enable on interrupts events 
  write8(LSM9DS1_ADDRESS_ACCELGYRO_WRITE,0x0C,0x40); 
  //set interrupt thresholds 
  write8(LSM9DS1_ADDRESS_ACCELGYRO_WRITE,0x07,0x0F); 
  write8(LSM9DS1_ADDRESS_ACCELGYRO_WRITE,0x08,0x0F); 
  //set accelerometer wait 
  write8(LSM9DS1_ADDRESS_ACCELGYRO_WRITE,0x0A,0xB2); 
  // set wait to be 
   
}

void readAccel(float* X, float* Y, float* Z) {
cli();
  unsigned char buffer[6];
  unsigned char reg[6] = {0x28,0x29,0x2A,0x2B,0x2C,0x2D};
  uint8_t status;
  status = i2c_io(0xD6, reg,6 , NULL, 0,buffer, 6);
  uint8_t xlo = buffer[0];
  int16_t xhi = buffer[1];
  uint8_t ylo = buffer[2];
  int16_t yhi = buffer[3];
  uint8_t zlo = buffer[4];
  int16_t zhi = buffer[5];
  // Shift values to create properly formed integer (low byte first)
  xhi <<= 8; 
  xhi |= xlo;
  yhi <<= 8; 
  yhi |= ylo;
  zhi <<= 8; 
  zhi |= zlo;

  *X = xhi;
  *Y = yhi;
  *Z = zhi;
  sei();
}

void write8(char address, char reg, char data)
{
    i2c_start(address);
    i2c_write(reg);
    i2c_write(data);
    i2c_stop();

}


