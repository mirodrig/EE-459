#ifndef RFM69_H_
#define RFM69_H_

// TODO: change later
#define cs 24

struct RFM69{
    char slaveSelectPin;
    char currentMode; // if 0 == sleep, 1 == rx, 2 == tx 
    char buffer[60];
    char buffer_length;
    volatile char receiveDataFlag; 
	volatile char packet_sent; 
    
};

void RFM_init(); 
char RFM_Read_FIFO(char* buffer, char* currentMode);
char RFM_recieve(struct RFM69* radio);
void RHFM_setPreambleLength(uint16_t bytes); 
void RFM_setSyncWords(char* syncwords);
void RFM_spiConfig() ; 
void rfm_write(char addr, char value);
char rfm_read(char addr);
void RFM_setFrequency(float centre); 
void RFM_modeSetter(char mode); 
void RFM_setMode(char* currentMode, char mode) ;
void RFM_send(char* data, char* currentMode, char dataSize);
void RFM_setPowerLevel(char powerLevel); 
void RFM_setHighPower(char onOff); 
int RFM_readRSSI() ; 
char RFM_interruptHandler(struct RFM69 radio, char* currentMode)  ;

#endif