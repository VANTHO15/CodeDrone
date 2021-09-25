#include "CTR_nrf24l01.h"
unsigned char  CTR_TX_ADDRESS[CTR_TX_ADR_WIDTH]= {0x68,0x31,0x08,0x10,0x01};   
unsigned char  CTR_RX_ADDRESS[CTR_RX_ADR_WIDTH]= {0x68,0x31,0x08,0x10,0x01};  

void CTR_nrfPinConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin = NRF_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NRF_MISO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NRF_CE_Pin NRF_CSN_Pin NRF_SCK_Pin NRF_MOSI_Pin */
  GPIO_InitStruct.Pin = NRF_CE_Pin|NRF_CSN_Pin|NRF_SCK_Pin|NRF_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

unsigned char CTR_spiRW(unsigned char Buff)
{
	uint8_t c=0;
	for(c=0;c<8;c++)
	{
		if( (Buff&0x80) == 0x80)
			HAL_GPIO_WritePin(NRF_MOSI_GPIO_Port,NRF_MOSI_Pin,GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(NRF_MOSI_GPIO_Port,NRF_MOSI_Pin,GPIO_PIN_RESET);
		HAL_Delay(5);
		Buff =  Buff<<1;
		HAL_GPIO_WritePin(NRF_SCK_GPIO_Port,NRF_SCK_Pin,GPIO_PIN_SET);
		HAL_Delay(5);
		Buff |= HAL_GPIO_ReadPin(NRF_MISO_GPIO_Port,NRF_MISO_Pin);
		HAL_GPIO_WritePin(NRF_SCK_GPIO_Port,NRF_SCK_Pin,GPIO_PIN_RESET);		
	}
	return (Buff);
}

unsigned char CTR_spiRead(unsigned char reg)
{
	uint8_t reg_value=0;
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port,NRF_CSN_Pin,GPIO_PIN_RESET);	
	CTR_spiRW(reg);
	reg_value = CTR_spiRW(0x00);
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port,NRF_CSN_Pin,GPIO_PIN_SET);	
	return reg_value;
}

unsigned char CTR_spiRWreg(unsigned char reg, unsigned char value)
{
	uint8_t status;
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port,NRF_CSN_Pin,GPIO_PIN_RESET);	
	status = CTR_spiRW(reg);
	CTR_spiRW(value);
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port,NRF_CSN_Pin,GPIO_PIN_SET);	
	return status;
}

unsigned char CTR_spiReadBuff(unsigned char reg, unsigned char *buff, unsigned char uchars)
{
	uint8_t status, c;
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port,NRF_CSN_Pin,GPIO_PIN_RESET);
	status = CTR_spiRW(reg);
	for(c=0;c<uchars;c++)
	{
		buff[c] = CTR_spiRW(0x00);
	}
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port,NRF_CSN_Pin,GPIO_PIN_SET);	
	return status;
}

unsigned char CTR_spiWriteBuff(unsigned char reg, unsigned char *buff, unsigned char uchars)
{
	uint8_t status,c;
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port,NRF_CSN_Pin,GPIO_PIN_RESET);
	status = CTR_spiRW(reg);
	for(c=0;c<uchars;c++)
	{
		CTR_spiRW(*buff++);
	}
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port,NRF_CSN_Pin,GPIO_PIN_SET);	
	return status;
}

void CTR_nrfInit(void)
{
	HAL_Delay(100);
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port,NRF_CSN_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(NRF_SCK_GPIO_Port,NRF_SCK_Pin,GPIO_PIN_RESET);
	CTR_spiWriteBuff(CTR_WRITE_RE + CTR_TX_ADDR, CTR_TX_ADDRESS, CTR_TX_ADR_WIDTH);    //
  CTR_spiWriteBuff(CTR_WRITE_RE + CTR_RX_ADDR_P0, CTR_RX_ADDRESS, CTR_RX_ADR_WIDTH); //
  CTR_spiRWreg(CTR_WRITE_RE + CTR_EN_AA, 0x01);      
  CTR_spiRWreg(CTR_WRITE_RE + CTR_EN_RXADDR, 0x01);  
  CTR_spiRWreg(CTR_WRITE_RE + CTR_RF_CH, 0);        
  CTR_spiRWreg(CTR_WRITE_RE + CTR_RX_PW_P0, CTR_RX_PLOAD_WIDTH); 
  CTR_spiRWreg(CTR_WRITE_RE + CTR_RF_SETUP, 0x07);        
  CTR_spiRWreg(CTR_WRITE_RE + CTR_CONFIG, 0x0e);          
}

void CTR_nrfSetRX(void)
{
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,GPIO_PIN_RESET);
	CTR_spiRWreg(CTR_WRITE_RE + CTR_CONFIG, 0x0f);
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,GPIO_PIN_SET);
	HAL_Delay(130);
}

void CTR_nrfSetTX(void)
{
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,GPIO_PIN_RESET);
	CTR_spiRWreg(CTR_WRITE_RE + CTR_CONFIG, 0x0e);
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,GPIO_PIN_SET);
	HAL_Delay(130);
}

unsigned char CTR_nrfGetPacket(unsigned char* rx_buf)
{
	uint8_t value=0,sta=0;
	sta = CTR_spiRead(CTR_STATUS);
	if((sta&0x40) != 0)
	{
		HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,GPIO_PIN_RESET);
		CTR_spiReadBuff(CTR_RD_RX_PLOAD,rx_buf,CTR_TX_PLOAD_WIDTH);
		value = 1;
	}
	CTR_spiRWreg(CTR_WRITE_RE+CTR_STATUS,sta);
	return value;
}

void CTR_nrfSendPacket(unsigned char * tx_buf)
{
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,GPIO_PIN_RESET);
	CTR_spiWriteBuff(CTR_WRITE_RE + CTR_RX_ADDR_P0, CTR_TX_ADDRESS, CTR_TX_ADR_WIDTH);
	CTR_spiWriteBuff(CTR_WR_TX_PLOAD, tx_buf, CTR_TX_PLOAD_WIDTH);
	CTR_spiRWreg(CTR_WRITE_RE + CTR_CONFIG, 0x0e);
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,GPIO_PIN_SET);
}
