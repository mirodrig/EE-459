#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H


#define LSM9DS1_ADDRESS_ACCELGYRO_READ     0xD7
#define LSM9DS1_ADDRESS_ACCELGYRO_WRITE    0xD6
#define CTRL_REG6_XL					   0x20 
#define CTRL_REG5_XL					   0x1F
#define LSM9DS1_REGISTER_CTRL_REG8         0x22

#define LSM9DS1_XG_ID                      0b01101000

void LSM_begin();

void write8(char address, char reg, char data);
void readAccel(float* X, float* Y, float* Z);

#endif 
