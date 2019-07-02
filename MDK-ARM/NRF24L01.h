#include "stm32f1xx_hal.h"

// Pin define
#define CS_PORT GPIOA
#define CE_PORT GPIOA

// Masks and some constants
#define NRF24_REGADDR_MASK      0x1F

// NRF24L01 data rates
#define NRF24_DataRate_1MBps    0 // 01 - 1Mbps
#define NRF24_DataRate_2MBps    1 // 10 - 2Mbps
#define NRF24_DataRate_250KBps  2 // 00 - 250kbps

// NRF Output power in TX mode
#define NRF24_OutPower_n18dBm   0 // -18dBm
#define NRF24_OutPower_n12dBm   1 // -12dBm
#define NRF24_OutPower_n6dBm    2 // -6dBm
#define NRF24_OutPower_0dBm     3 // 0dBm

// NRF24L01 SPI commands name
#define NRF24_CMD_R_REGISTER    0x00   // 000A AAAA 
#define NRF24_CMD_W_REGISTER    0x20	 // 001A AAAA
#define NRF24_CMD_R_RX_PAYLOAD  0x61   // 0110 0001
#define NRF24_CMD_W_TX_PAYLOAD  0xA0	 // 1010 0000
#define NRF24_CMD_FLUSH_TX      0xE1	 // 1110 0001
#define NRF24_CMD_FLUSH_RX      0xE2	 // 1110 0010
#define NRF24_CMD_REUSE_TX_PL   0xE3	 // 1110 0011
#define NRF24_CMD_R_RX_PL_WID   0x60	 // 0110 0000
#define NRF24_CMD_W_ACK_PAYLOAD 0xA8	 // 1010 1PPP
#define NRF24_CMD_W_TX				  0xB0	 // 1011 0000
#define NRF24_CMD_NOP           0xFF   // 1111 1111

// Register map table
#define NRF24_REG_CONFIG        0x00
#define NRF24_REG_EN_AA         0x01
#define NRF24_REG_EN_RXADDR     0x02
#define NRF24_REG_SETUP_AW      0x03
#define NRF24_REG_SETUP_RETR    0x04
#define NRF24_REG_RF_CH         0x05
#define NRF24_REG_RF_SETUP      0x06
#define NRF24_REG_STATUS        0x07
#define NRF24_REG_OBSERVE_TX    0x08
#define NRF24_REG_RPD           0x09
#define NRF24_REG_RX_ADDR_P0    0x0A
#define NRF24_REG_RX_ADDR_P1    0x0B
#define NRF24_REG_RX_ADDR_P2    0x0C
#define NRF24_REG_RX_ADDR_P3    0x0D
#define NRF24_REG_RX_ADDR_P4    0x0E
#define NRF24_REG_RX_ADDR_P5    0x0F
#define NRF24_REG_TX_ADDR       0x10
#define NRF24_REG_RX_PW_P0      0x11
#define NRF24_REG_RX_PW_P1      0x12
#define NRF24_REG_RX_PW_P2      0x13
#define NRF24_REG_RX_PW_P3      0x14
#define NRF24_REG_RX_PW_P4      0x15
#define NRF24_REG_RX_PW_P5      0x16
#define NRF24_REG_FIFO_STATUS   0x17
#define NRF24_REG_DYNPD	        0x1C
#define NRF24_REG_FEATURE				0x1D



uint8_t NRF24_read_reg(uint8_t RegAddr);
void NRF24_write_reg(uint8_t RegAddr,uint8_t data);
void check_NRF24L01_payload(void);
void NRF24_init(void);
