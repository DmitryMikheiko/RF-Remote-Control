#include "stdbool.h"
#include <stdint.h>
#include "main.h"
#include "stm32f1xx_hal.h"

#define csn_1 HAL_GPIO_WritePin(CSN_GPIO_Port,CSN_Pin,GPIO_PIN_SET); 
#define csn_0 HAL_GPIO_WritePin(CSN_GPIO_Port,CSN_Pin,GPIO_PIN_RESET);
#define ce_1 HAL_GPIO_WritePin(CE_GPIO_Port,CE_Pin,GPIO_PIN_SET);
#define ce_0 HAL_GPIO_WritePin(CE_GPIO_Port,CE_Pin,GPIO_PIN_RESET);
#define R_REGISTER  0X00      //Read registers directly address bitwise
#define W_REGISTER  0X20      //write registers      Bitwise OR with address
#define R_RX_PAYLOAD  0X61    //read data 1-32 byte  From the beginning of 0 bytes
#define W_TX_PAYLOAD  0XA0    //write data 1-32 byte  From the beginning of 0 bytes
#define FLUSH_TX  0xE1        //clear TX FIFO regsiters
#define FLUSH_RX  0XE2        //clear RX FIFO regsiters  This command should not be used when the transfer acknowledge signal
#define RESUSE_TX_PL 0XE3     //Re-use on a packet of valid data when CE is high, the data packets continually re-launch
#define NOP  0XFF             //Empty command is used to retrieve data
#define R_RX_PL_WID  0x60


extern uint8_t spi(uint8_t address);
extern void soft_delay_ms(uint16_t ms);
extern void delay_us(uint16_t time_);


unsigned char read_irq(void);
void clr_irq(void);
void tx_mode(void);
void rx_mode(void);
uint8_t read_rx(uint8_t *data);
void send_data(uint8_t *data,uint8_t size);
void NRF24L01_init(void);
void SetID(uint8_t *data);
void NRF24L01_hack_mode(bool state);
uint8_t NRF24L01_ReadRigester(uint8_t addr);
void NRF24L01_WriteRigester(uint8_t addr,uint8_t value);
void NRF24L01_PowerDown(void);
void SetTxAddr(uint8_t *addr);
void SetRxAddr(uint8_t *addr);