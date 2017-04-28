/*********************************
 * rfm69.h
*********************************/
#ifndef RFM69_H_
#define RFM69_H_

#include <avr/io.h>
#include <stdbool.h>

typedef enum rfm_status_t { RFM_OK, RFM_FAIL, RFM_TIMEOUT } rfm_status_t; // status codes for return values

#define RFM69_SPI_WRITE_MASK 0x80 // write commands must have this line
#define RFM69_MAX_MESSAGE_LEN 64 // max message length
#define RFM69_FIFO_SIZE 64 // max number of elements FIFO can hold

#define RFM69_MODE_SLEEP 0x00 /* 0.1uA  */
#define RFM69_MODE_STDBY 0x04 /* 1.25mA */
#define RFM69_MODE_RX 0x10 /* 16mA   */
#define RFM69_MODE_TX 0x0C /* >33mA  */

#define RF22_TXFFAEM_THRESHOLD 4
#define RF22_RXFFAFULL_THRESHOLD 55
#define CSMA_LIMIT -90

#define RF69_IRQ_PIN 4 // PD2 of ATMega328
#define RF69_IRQ_NUM 0 // DIO0 pin of RFM69

// TWS: define CTLbyte bits
#define RFM69_CTL_SENDACK 0x80
#define RFM69_CTL_REQACK 0x40

// create an object for the RFM69
struct rfm69{
    uint8_t DATA[RFM69_MAX_MESSAGE_LEN]; // message to be sent/received
    uint8_t DATALEN;
    uint8_t senderID;
    uint8_t targetID;
    uint8_t PAYLOADLEN;
    uint8_t ACK_REQ;
    uint8_t ACK_RECV;
    uint16_t RSSI; // RSSI during reception
    uint8_t _mode; // current mode;
    uint8_t sentPacket;
    uint8_t _addr; // address of the RFM69
};

/* declare function prototypes here */
rfm_status_t rf69_init(void);
uint8_t _rf69_read(uint8_t);
void _rf69_write(uint8_t, uint8_t);
//static rfm_status_t _rf69_burst_read(const uint8_t, uint8_t*, uint8_t);
//static rfm_status_t _rf69_fifo_write(const uint8_t*, uint8_t);
void rf69_set_mode(struct rfm69, uint8_t);
//void rf69_receiveBegin(struct rfm69*);
//bool receiveDone(struct rfm69*);
//bool canSend(struct rfm69*);
//void rf69_send(uint8_t, const void*, uint8_t, bool);
//void rf69_send(char *);

//void rf69_sendFrame(uint8_t, const void*, uint8_t, bool, bool);
//int16_t readRSSI(void);
//void setNodeAddress(uint8_t);
//void setNetwork(uint8_t);

//rfm_status_t rf69_receive(uint8_t*, uint8_t*, int16_t*, bool*);
//rfm_status_t rf69_send(const uint8_t*, uint8_t, const uint8_t);

//static rfm_status_t _rf69_clear_fifo(void);
//rfm_status_t rf69_read_temp(int8_t*);
//rfm_status_t rf69_sample_rssi(int16_t*);
uint8_t interruptHandler(uint8_t*);

// RFM69 functions relating to SPI
rfm_status_t spi_init(void);
rfm_status_t spi_exchange_single(const uint8_t, uint8_t*);
rfm_status_t spi_ss_assert(void);
rfm_status_t spi_ss_deassert(void);

#endif
