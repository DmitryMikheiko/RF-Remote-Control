#include "NRF24L01.h"

uint8_t state = 0x00;
uint8_t NRF24L01_ID[5]={0xE7,0xE7,0xE7,0xE7,0xE7};
uint8_t buffer[33];
uint8_t length;
uint8_t nrf_i;

volatile bool NRF24L01_TX_RUN =false;
bool NRF24L01_RX=false;
bool NRF24L01_TX=false;

bool HackModeState=false;

//*****************************************************************************
//(((((((((((((((((((((((((    NRF24L01    ))))))))))))))))))))))))))))))))))))
//*****************************************************************************
unsigned char read_irq()
{unsigned char stat;
 csn_0
 spi(0x07);
 stat=spi(0xFF);
 csn_1
 return stat; 
}
void clr_irq()
{
 unsigned char stat;
 stat=read_irq();
 csn_0
 spi(0x27);
 spi(stat); 
 csn_1
}
void tx_mode(void)
{ 
	__disable_irq();
 ce_0
 csn_0
 spi(0x20);
 spi(0xE); 
 csn_1
 ce_1
	NRF24L01_TX=true;
	NRF24L01_RX=false;
	soft_delay_ms(2);
	__enable_irq();
}

void rx_mode(void)
{ 
	__disable_irq();
 ce_1
 csn_0
 spi(0x20);
 spi(0xF);
 csn_1
	NRF24L01_TX=false;
	NRF24L01_RX=true;
	soft_delay_ms(2);
__enable_irq();	
}


void send_data(uint8_t *data,uint8_t size)
{ uint8_t a=0;
	__disable_irq();
if(!NRF24L01_TX) tx_mode();  
  while(read_irq() & 0x1); 
 csn_0
 spi(0xE1);
 csn_1
 delay_us(10);
 csn_0
 spi(0xA0);
 for (a=0;a<size;a++){spi(data[a]); }
 csn_1 
 clr_irq();               
 NRF24L01_TX_RUN=true; 
 __enable_irq(); 

}
unsigned char read_fifo_status()
{unsigned char stat;
 __disable_irq();
 csn_0
 spi(0x17);
 stat=spi(0xFF);
 csn_1      
 __enable_irq();
 return stat;
}
uint8_t read_rx(uint8_t *data)
{
	uint8_t a,pos=0,length;
	__disable_irq();
 while(!(read_fifo_status()&1))
 {
 csn_0
 spi(R_RX_PL_WID);
 length=spi(0xFF);
 csn_1   
 csn_0
 spi(0x61);
 for (a=0;a<length;a++) data[pos++]=spi(0xFF);
 csn_1
 csn_0
 spi(0xE2);
 csn_1
 data[pos]='\0';
 clr_irq();  
 }
	__enable_irq();
return length;
}
void SetID(uint8_t *data)
{
char i;
__disable_irq();
	soft_delay_ms(30);
	clr_irq();
csn_0
spi(0x2A);   
for(i=0;i<5;i++) spi(data[i]);
csn_1
csn_0
spi(0x30);   
for(i=0;i<5;i++) spi(data[i]);
csn_1
soft_delay_ms(30);
	clr_irq();
__enable_irq();
}
void NRF24L01_init(void)
{
	__disable_irq();
	soft_delay_ms(10);
	clr_irq();
csn_1
ce_0	
	
csn_0
spi(0x50);
spi(0x73);   //Activate
csn_1

csn_0
spi(0x3D);
spi(0x06);   // 0x04-Enables Dynamic Payload Length  | 0x2  - Enables Payload with ACK
csn_1           
csn_0
spi(0x3C);   //Enable dyn. payload length for all data pipes
spi(0x3F);
csn_1
delay_us(10);
csn_0
spi(0x26);    // Disable LNA Gain  
spi(0x0E);
csn_1
csn_0
spi(0x20);
spi(0xE);   //Config(0x0E) - CRC-2bytes | EN_CRC | PWR_UP
csn_1
csn_0
spi(0x24);
spi(0x13);   //Setup_Retr   500us 3-Re-Transmit
csn_1
soft_delay_ms(1);
tx_mode();
soft_delay_ms(50);     
csn_0                 
spi(0xE2);       
csn_1            
csn_0           
spi(0xE1);      
csn_1            
clr_irq();
__enable_irq();
}
void NRF24L01_hack_mode(bool state)
{
__disable_irq();
HackModeState=state;
csn_0
spi(0x21);
if(state)spi(0x00);
else spi(0x3F);
csn_1
__enable_irq();
}
uint8_t NRF24L01_ReadRigester(uint8_t addr)
{
uint8_t value;
__disable_irq();
csn_0
 spi(addr);
 value=spi(0xFF);
 csn_1
__enable_irq();
return value;
}
void NRF24L01_WriteRigester(uint8_t addr,uint8_t value)
{
__disable_irq();
csn_0
 spi(addr+0x20);
 spi(value);
 csn_1   
 soft_delay_ms(1);
__enable_irq();
}
void NRF24L01_PowerDown(void)
{	
	__disable_irq();
 ce_0
 csn_0
 spi(0x20);
 spi(0x8);
 csn_1
	//NRF24L01_POWER=false;
	NRF24L01_TX=false;
	NRF24L01_RX=false;
	__enable_irq();
}
