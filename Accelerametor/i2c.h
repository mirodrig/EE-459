#ifndef I2C_h
#define I2C_h

 #include <avr/io.h>
#include <util/twi.h>
#include <stdint.h>
# include <stddef.h>

void i2c_init(); 
uint8_t i2c_start(uint8_t address); // have to send it to every device 
uint8_t i2c_write(uint8_t data); 
void i2c_stop(void); 
uint8_t i2c_read_ack();
uint8_t i2c_read_nack(); 
uint8_t i2c_io(uint8_t device_addr, uint8_t *ap, uint16_t an, 
               uint8_t *wp, uint16_t wn, uint8_t *rp, uint16_t rn);

#endif 