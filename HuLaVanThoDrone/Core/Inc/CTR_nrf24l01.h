#ifndef CTR_NRF
#define CTR_NRF 100
#include "stm32f4xx_hal.h"
 
 // d�y l� phan dinh nghia chan ket noi voi module
#define NRF_MISO_Pin GPIO_PIN_6
#define NRF_MISO_GPIO_Port GPIOA
#define NRF_CE_Pin GPIO_PIN_5
#define NRF_CE_GPIO_Port GPIOC
#define NRF_CSN_Pin GPIO_PIN_4
#define NRF_CSN_GPIO_Port GPIOA
#define NRF_SCK_Pin GPIO_PIN_5
#define NRF_SCK_GPIO_Port GPIOA
#define NRF_MOSI_Pin GPIO_PIN_7
#define NRF_MOSI_GPIO_Port GPIOA

#define CTR_TX_ADR_WIDTH    5      
#define CTR_RX_ADR_WIDTH    5      
#define CTR_TX_PLOAD_WIDTH  32    
#define CTR_RX_PLOAD_WIDTH  32    


#define CTR_READ_RE        0x00     
#define CTR_WRITE_RE        0x20    
#define CTR_RD_RX_PLOAD     0x61     
#define CTR_WR_TX_PLOAD     0xA0     
#define CTR_FLUSH_TX        0xE1    
#define CTR_FLUSH_RX        0xE2     
#define CTR_REUSE_TX_PL     0xE3    
#define CTR_NOP             0xFF     

#define CTR_CONFIG          0x00  
#define CTR_EN_AA           0x01  
#define CTR_EN_RXADDR       0x02  
#define CTR_SETUP_AW        0x03 
#define CTR_SETUP_RETR      0x04  
#define CTR_RF_CH           0x05  
#define CTR_RF_SETUP        0x06  
#define CTR_STATUS          0x07  
#define CTR_OBSERVE_TX      0x08   �
#define CTR_CD              0x09  
#define CTR_RX_ADDR_P0      0x0A  
#define CTR_RX_ADDR_P1      0x0B  
#define CTR_RX_ADDR_P2      0x0C  
#define CTR_RX_ADDR_P3      0x0D  
#define CTR_RX_ADDR_P4      0x0E  
#define CTR_RX_ADDR_P5      0x0F  
#define CTR_TX_ADDR         0x10  
#define CTR_RX_PW_P0        0x11  
#define CTR_RX_PW_P1        0x12  
#define CTR_RX_PW_P2        0x13  
#define CTR_RX_PW_P3        0x14  
#define CTR_RX_PW_P4        0x15  
#define CTR_RX_PW_P5        0x16  
#define CTR_FIFO_STATUS     0x17  

void CTR_nrfPinConfig(void);

void CTR_nrfInit(void);
unsigned char CTR_spiRW(unsigned char Buff);
unsigned char CTR_spiRead(unsigned char reg);
unsigned char CTR_spiRWreg(unsigned char reg, unsigned char value);
unsigned char CTR_spiReadBuff(unsigned char reg, unsigned char *buff, unsigned char uchars);
unsigned char CTR_spiWriteBuff(unsigned char reg, unsigned char *buff, unsigned char uchars);
void CTR_nrfSetRX(void);
void CTR_nrfSetTX(void);
unsigned char CTR_nrfGetPacket(unsigned char* rx_buf);
void CTR_nrfSendPacket(unsigned char * tx_buf);
#endif
