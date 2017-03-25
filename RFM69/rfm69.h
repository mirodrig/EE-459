/*********************************
 * rfm69.h
*********************************/
#ifndef RFM69_H_
#define RFM69_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

typedef uint8_t rfm_reg_t; // make a typedef for a rfm69 register

/* Status codes for return values from library functions. These will
determine if an operation is OK, fails, or timesout */
typedef enum rfm_status_t { RFM_OK, RFM_FAIL, RFM_TIMEOUT } rfm_status_t;

/* Write commands to the RFM have this bit set */
#define RFM69_SPI_WRITE_MASK 0x80

/* Maximum message length that can be supported by this library. 
 * Interrupts will refill the Tx FIFO during transmission and to empty 
 * Rx FIFO during reception */
#define RFM69_MAX_MESSAGE_LEN 64

/* Max number of octets the RFM69 FIFO can hold */
#define RFM69_FIFO_SIZE 64

#define RFM69_MODE_SLEEP    0x00 /* 0.1uA  */
#define RFM69_MODE_STDBY    0x04 /* 1.25mA */
#define RFM69_MODE_RX       0x10 /* 16mA   */
#define RFM69_MODE_TX       0x0c /* >33mA  */

/* These values we set for FIFO thresholds are actually the same as the 
 * POR values */
#define RF22_TXFFAEM_THRESHOLD 4
#define RF22_RXFFAFULL_THRESHOLD 55

/* definitions with respect to interrupts */
#define RF69_IRQ_PIN 4 // PD2
#define RF69_IRQ_NUM 0

/* Public prototypes here */
rfm_status_t rf69_init(void);
rfm_status_t rf69_read_temp(int8_t* temperature);
rfm_status_t rf69_receive(rfm_reg_t* buf, rfm_reg_t* len, int16_t* lastrssi, bool* rfm_packet_wait);
rfm_status_t rf69_send(const rfm_reg_t* data, uint8_t len, const uint8_t power);
rfm_status_t rf69_set_mode(const rfm_reg_t newMode);
rfm_status_t rf69_sample_rssi(int16_t* rssi);
rfm_status_t interruptHandler(void);
void rf69_encrypt(const char *key);

/* SPI device driver functions. These are to be provided by the user.
 * Prototypes are provided here such that the library can be built.
 * Documentation can be found in spi_conf.c */
rfm_status_t spi_init(void);
rfm_status_t spi_exchange_single(const rfm_reg_t out, rfm_reg_t* in);
rfm_status_t spi_ss_assert(void);
rfm_status_t spi_ss_deassert(void);

uint8_t DATA[RFM69_MAX_MESSAGE_LEN];

#endif
