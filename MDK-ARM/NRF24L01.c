#include "NRF24L01.h"

// Global Variables
extern SPI_HandleTypeDef hspi1;
extern uint8_t received_byte;
uint8_t dataWatch8, spiData[5];
uint16_t dataWatch16;

// Functions

// Read register
uint8_t NRF24_read_reg(uint8_t RegAddr) // Alright
{
	uint8_t spiData[2];
	spiData[0] = (NRF24_CMD_R_REGISTER| (RegAddr & NRF24_REGADDR_MASK));
	// Begin SPI communication
	HAL_GPIO_WritePin(CS_PORT, CSN_Pin, GPIO_PIN_RESET); // Set GPIOA, PIN 4 to Low
	HAL_SPI_Transmit(&hspi1, spiData, 1, 100); // Transmit SPI data
	HAL_SPI_Receive(&hspi1, &spiData[1], 1, 100); // Receive SPI data
	HAL_GPIO_WritePin(CS_PORT, CSN_Pin, GPIO_PIN_SET); // Set GPIOA, PIN 4 to High
	// End SPI communication
	return spiData[1];
}

// Write register
void NRF24_write_reg(uint8_t RegAddr,uint8_t data) // Alright
{
	uint8_t spiData[2];
	spiData[0] = (NRF24_CMD_W_REGISTER| (RegAddr & NRF24_REGADDR_MASK));
	// Begin SPI communication
	HAL_GPIO_WritePin(CS_PORT, CSN_Pin, GPIO_PIN_RESET); // Set GPIOA, PIN 4 to Low
	HAL_SPI_Transmit(&hspi1, spiData, 1, 100); // Transmit SPI data
	HAL_SPI_Transmit(&hspi1, &data, 1, 100); // Transmit SPI data
	HAL_GPIO_WritePin(CS_PORT, CSN_Pin, GPIO_PIN_SET); // Set GPIOA, PIN 4 to High
	// End SPI communication
}

// Read register
uint8_t NRF24_read_cmd(uint8_t RegAddr) // Alright
{
	uint8_t spiData[2];
	spiData[0] = (NRF24_CMD_R_RX_PAYLOAD| (RegAddr & NRF24_REGADDR_MASK));
	// Begin SPI communication
	HAL_GPIO_WritePin(CS_PORT, CSN_Pin, GPIO_PIN_RESET); // Set GPIOA, PIN 4 to Low
	HAL_SPI_Transmit(&hspi1, spiData, 1, 100); // Transmit SPI data
	HAL_SPI_Receive(&hspi1, &spiData[1], 1, 100); // Receive SPI data
	HAL_GPIO_WritePin(CS_PORT, CSN_Pin, GPIO_PIN_SET); // Set GPIOA, PIN 4 to High
	// End SPI communication
	return spiData[1];
}

void NRF24_FlushRx()
{
	uint8_t spiData[1];
	spiData[0] = NRF24_CMD_FLUSH_RX;
	// Begin SPI communication
	HAL_GPIO_WritePin(CS_PORT, CSN_Pin, GPIO_PIN_RESET); // Set GPIOA, PIN 4 to Low
	HAL_SPI_Transmit(&hspi1, spiData, 1, 100); // Transmit SPI data
	HAL_GPIO_WritePin(CS_PORT, CSN_Pin, GPIO_PIN_SET); // Set GPIOA, PIN 4 to High
	// End SPI communication
}

void NRF24_FlushTx()
{
	NRF24_write_reg(NRF24_CMD_FLUSH_TX, 0);
}

void check_NRF24L01_payload()
{
	spiData[2] = NRF24_read_reg(NRF24_REG_STATUS);
	spiData[3] = (spiData[2] & 0x40) >> 6;
		
	if(spiData[3] == 0x01)
	{
		HAL_GPIO_WritePin(CE_PORT, CE_Pin, GPIO_PIN_RESET); // Set GPIOA, PIN 4 to Low
		HAL_GPIO_WritePin(CS_PORT, CSN_Pin, GPIO_PIN_RESET); // Set GPIOA, PIN 4 to Low
		HAL_SPI_Transmit(&hspi1, spiData, 1, 100); // Transmit SPI data
		HAL_SPI_Receive(&hspi1, &spiData[1], 1, 100); // Receive SPI data
		HAL_GPIO_WritePin(CS_PORT, CSN_Pin, GPIO_PIN_SET); // Set GPIOA, PIN 4 to High
		NRF24_write_reg(NRF24_REG_STATUS,0x70);
		HAL_GPIO_WritePin(CE_PORT, CE_Pin, GPIO_PIN_SET); 
		received_byte = spiData[1];
	}
	else
	{
		received_byte = 0x00;
		NRF24_FlushRx();
	}
}

 // NRF24 initialization
void NRF24_init()
{
	NRF24_write_reg(NRF24_REG_CONFIG,0x0B);
	NRF24_write_reg(NRF24_REG_EN_AA,0x00); 
	NRF24_write_reg(NRF24_REG_SETUP_RETR,0x00); // Disable Auto Retransmit Ability
	NRF24_write_reg(NRF24_REG_RF_CH,0x0F); // Set Frequency Channel
	NRF24_write_reg(NRF24_REG_RF_SETUP,0x06); // 1Mbps & 0dBm
	NRF24_write_reg(NRF24_REG_RX_PW_P0,0x01); // Use Pipe 0
	NRF24_write_reg(NRF24_REG_RX_PW_P1,0x00); //
	NRF24_write_reg(NRF24_REG_EN_RXADDR,0x01); // Enable data pipe 1
	NRF24_write_reg(NRF24_REG_CONFIG,0x0B);
	
	dataWatch8 = NRF24_read_reg(NRF24_REG_CONFIG);
	//HAL_Delay(2);
	if(dataWatch8 != 0x0B)
	{
		while(1)
		{
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
			HAL_Delay(50);
		}
	}

	HAL_GPIO_WritePin(CE_PORT, CE_Pin, GPIO_PIN_SET); // Set CE Pin to HIGH
	spiData[0] = NRF24_CMD_R_RX_PAYLOAD;			 // Command to read payload of NRF24L01
}
