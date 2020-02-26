/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "string.h"
#include "LPH91572.c"
#include "NRF24L01.c"
#include "Bat1.h"
#include "UWB.c"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t data;
#define SysFreq 72
#define SYS_FCLK SysFreq
#define HEX2BCD(v)	((v) % 10 + (v) / 10 * 16)
#define BCD2HEX(v)	((v) % 16 + (v) / 16 * 10)
RTC_TimeTypeDef Time;

#define I2C1_DEVICE_ADDRESS 0x50
uint8_t eBuffer[1];

uint16_t InUmV=0;
#define InU_Koef 2.0*3.00*1.0206/4095.0
#define BatteryLevel_1 3000
#define BatteryLevel_2 3400
#define BatteryLevel_3 3700
#define BatteryLevel_4 3900

uint16_t FonColor = BLACK;
uint16_t TextColor = WHITE;

#define PowerDownTime 10
uint8_t PowerDownTimeCounter=0;
bool PowerDown=false;
bool PowerDownNow=false;

extern volatile bool NRF24L01_TX_RUN;
volatile bool RF_RX=false;
u8 RF_Buffer[32];
u8 RF_Buffer_Len=0;

char options[8][17]=
{"  Back           ",
 "  Devices        ",
 "  Find devices   ",
 "  Keypad         ",
 "  Settings       ",
 "                 ",
 "                 ",
 "                 "};	

#define MenuCount 5
uint8_t SelectedMenuPos = 1;
#define MenuShiftY 25
 
volatile u8 PressedButton=0;
 
bool SecDotsShow=false;
 
struct DeviceChannelU8
{
	u8 Type;
	u8 CID;
	char *Name;
	u8 Min;
	u8 Max;
	u8 Value;
};
struct DeviceChannelU16
{
	u8 Type;
	u8 CID;
	char *Name;
	u16 Min;
	u16 Max;
	u16 Value;
};
struct DeviceChannelU32
{
	u8 Type;
	u8 CID;
	char *Name;
	u32 Min;
	u32 Max;
	u32 Value;
};
struct DeviceChannelF
{
	u8 Type;
	u8 CID;
	char *Name;
	float Min;
	float Max;
	float Value;
};
struct DeviceChannelStr
{
	u8 Type;
	u8 CID;
	char *Name;
	char *Value;  
};
struct DeviceDesc
{
	int size;
	u32 ID;
	char *Name;
	char *DescStr;
	u8 ChannelCount;
	void **Channels;
};
#define DevicesMaxCount 15
struct DevicesStruct
{
	u8 Count;
	struct DeviceDesc *Device[DevicesMaxCount];
}Devices;

struct TempDevice
{
	u16 pos;
	u16 Size;
	u16 SizeNow;
	u32 ID;
	u8 *Buffer;
};
struct TempDevicesStruct
{
	u8 Count;
	struct TempDevice *Device[DevicesMaxCount];
}TempDevices;
union _U8ToU16
{
	u8 *Bytes;
	u16 Value;
}U8ToU16;
union _U8ToU32
{
	u8 *Bytes;
	u32 Value;
}U8ToU32;
union _U32ToU8
{
	u8 Bytes[4];
	u32 Value;
}U32ToU8;
union _U16ToU8
{
	u8 Bytes[2];
	u32 Value;
}U16ToU8;
union _U8ToF
{
	u8 *Bytes;
	float Value;
}U8ToF;

#define UWB_FirstMarker '$'
#define UWB_DataMarker 0x1
#define UWB_DataContinueMarker 0x2
#define UWB_ChannelMarker 0x3
#define UWB_SendToChannelMarker 0x4

#define UWB_Type_Digital 0
#define UWB_Type_PWM 1
#define UWB_Type_Analog 2
#define UWB_Type_Value 3
#define UWB_Type_OnlyRead 0x80
#define UWB_Type_8bit 0x00
#define UWB_Type_16bit 0x10
#define UWB_Type_32bit 0x20
#define UWB_Type_F 0x30
#define UWB_Type_Str 0x40
#define UWB_Type_DimMask 0x70
#define UWB_Type_Mask 0x0F

struct KeyStruct
{
	u32 DevID;
	u8 CID;
	u8 Key;
};
struct KeyStruct Keys[13];

char *msgs[5]={"Channel 0","Channel 1", "Channel 2","Channel 3", "string"};

uint8_t Addr[5]={0xAA,0xAA,0xAA,0xAA,0x00};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void WriteTime(void);
void EEPROM_Read(uint16_t address,uint8_t *buffer,uint16_t size);
void EEPROM_Write(uint16_t address,uint8_t *buffer,uint16_t size);
void LcdWD(uint8_t  data);
uint8_t spi(uint8_t address);
void delay_us(uint16_t time_);
void soft_delay_ms(uint16_t ms);
uint16_t GetInUmV(void);
void WriteVoltage(void);
void Tick_1s(void);
void Power(void);
void PowerOff(void);
void PowerOn(void);
void InitAll(void);
void ShowSecDots(void);

struct TempDevice* GetTempDeviceByID(u32 ID);
void UnZipPackage(struct TempDevice* device);
void U8ToStr(u8 data,char *str);
void U16ToHEX(u16 data,char *str);
void U32ToHEX(u32 data,char *str);

void KeyPresed(uint8_t key);
int8_t GetKey(u32 dev_ID,u8 CID);
int8_t SetKey(u32 dev_ID,u8 CID,u8 key);
void WriteSetKeyMenu(u32 dev_ID,u8 ch_id,u16 t_color,u16 b_color);
struct DeviceDesc* GetDeviceByID(u32 ID);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void SendPacket(uint8_t *data,uint8_t size)
{
	send_data(data,size);
}
void LcdWD(uint8_t  data)
{
  //LCD_CS_0
	while ((SPI2->SR &SPI_SR_TXE ) == RESET){}
  SPI2->DR=data;
	//while((SPI2->SR &SPI_SR_BSY)>0); 
	//HAL_SPI_Transmit(&hspi2,&data,1,100);
	//LCD_CS_1
}
uint8_t spi(uint8_t address)
{
	HAL_SPI_TransmitReceive(&hspi1, &address, &data, sizeof(data), 0x1000);
	return data;
}
void delay_us(uint16_t time_)
{
	uint32_t time;
	time=time_;
	time*=SYS_FCLK;
	time/=3;
  while(time--);
}
void soft_delay_ms(uint16_t ms)
{
	while(ms--) delay_us(1000);
}
uint16_t GetInUmV(void)
{
	InUmV=(uint16_t)(((float)(hadc1.Instance->DR))*InU_Koef*1000.0);
	return InUmV;
}
void WriteVoltage(void)
{ 
	int volt,volt2;
  unsigned char volt_string[5];
	
	volt=GetInUmV();
	delay_us(300);
	volt+=GetInUmV();
	delay_us(300);
	volt+=GetInUmV();
	volt/=3;
	volt2=volt;
/*	if(volt<bad_volt)
	{  
	 
	  
	}*/
	volt_string[0]=volt/1000+'0';
	volt%=1000;
	volt_string[1]='.';
	volt_string[2]=volt/100+'0';
	volt%=100;
	volt_string[3]=volt/10+'0';
	volt_string[4]=volt%10+'0';
	LCD_Puts((char*)volt_string,4,10,100,RED,FonColor);
	LCD_Write_Image_RLE_ColorBar(Bat1,sizeof(Bat1),Bat1_ColorBar,0,19,0,11);
	if(volt2<BatteryLevel_4)
	{
		LCD_FillRect(14,3,3,6,FonColor);
	}
	if(volt2<BatteryLevel_3)
	{
		LCD_FillRect(10,3,3,6,FonColor);
	}
	if(volt2<BatteryLevel_2)
	{
		LCD_FillRect(6,3,3,6,FonColor);
	}
	if(volt2<BatteryLevel_1)
	{
		LCD_FillRect(2,3,3,6,FonColor);
	}

}
void Tick_1s(void)
{
	//PowerDownTimeCounter++;
	if(PowerDownTimeCounter>=PowerDownTime)
	{
		PowerOff();
	}
	SecDotsShow=!SecDotsShow;
}
void PowerOff(void)
{
	PowerDownTimeCounter=0;
	PowerDown=true;
}
void PowerOn(void)
{
	PowerDownTimeCounter=0;
	PowerDown=false;
}
void Power(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	if(PowerDown!=PowerDownNow)
	{
		PowerDownNow=PowerDown;
		if(PowerDown)
		{
			//LCD_Puts("Power off",9,10,70,TextColor,FonColor);
			LCD_off();
			NRF24L01_PowerDown();
			HAL_ADC_Stop_IT(&hadc1);
			HAL_TIM_Base_Stop_IT(&htim1);
			//HAL_NVIC_DisableIRQ(EXTI0_IRQn);
			
			HAL_GPIO_WritePin(LCD_POWER_GPIO_Port,LCD_POWER_Pin,0);
			HAL_GPIO_WritePin(RF_MODULE_POWER_GPIO_Port,RF_MODULE_POWER_Pin,0);
			

	
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = CE_Pin|CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  //GPIO_InitStruct.Pin = IRQ_Pin;
 // GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
 // HAL_GPIO_Init(IRQ_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

 
  GPIO_InitStruct.Pin = LCD_RS_Pin|LCD_RESET_Pin|LCD_CS_Pin 
                          |LED_RED_Pin|LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

			
			//__HAL_RCC_HSI_DISABLE(); 
			
			HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON,PWR_SLEEPENTRY_WFI);
			//HAL_PWR_EnterSTANDBYMode();
			
		Power();
		}
		else
		{			
			InitAll();			
			LCD_Puts("Power on ",9,10,70,TextColor,FonColor);
		}
		
	}
	
}

void WriteMenu(u16 t_color,u16 b_color)
{
	u16 pos=0;
	for(pos=0;pos<MenuCount;pos++) 
	if (SelectedMenuPos==pos) LCD_Puts(options[pos],17,0,pos*16+MenuShiftY,b_color,t_color);
	 else LCD_Puts(options[pos],17,0,pos*16+MenuShiftY,t_color,b_color);    
	
}
void WriteDevicesMenu(u8 dev_id,u16 t_color,u16 b_color)
{
	u8 pos=0,f_pos=0;
	u8 length=strlen(Devices.Device[pos]->Name);
	u8 c;
	u16 color1,color2;
	if(dev_id<8) pos=f_pos=0;
	else pos=f_pos=dev_id-7;
	for(;(pos<Devices.Count) && (pos < f_pos+8);pos++)
	{
		color1=color2=t_color;
	 if(pos==dev_id) color1=b_color;
	 else color2=b_color;
	     
		    LCD_Putchar(('0'+pos/10),0,(pos%8)*16+MenuShiftY,color1,color2);
		    LCD_Putchar(('0'+pos%10),8,(pos%8)*16+MenuShiftY,color1,color2);
		    LCD_Putchar(' ',16,pos*16+MenuShiftY,color1,color2);
		    LCD_Puts(Devices.Device[pos]->Name,length,24,pos*16+MenuShiftY,color1,color2);
	      for(c=0;c<(17-length);c++)LCD_Putchar(' ',24+8*(length+c),pos*16+MenuShiftY,color1,color2);
	
	}
}
void WriteDeviceInfo(u8 dev_id,u16 t_color,u16 b_color)
{
	struct DeviceDesc *device=Devices.Device[dev_id];
	char *name=device->Name;
	char str[8];
	u8 length=strlen(name);
	u8 name_shift=(u8)((17-length)/2*8);
	if(length>17)length=17;
	
	LCD_Puts(name,length,name_shift,0,t_color,b_color);
	U8ToStr(device->ChannelCount,str);
	LCD_Puts("Channels: ",10,0,24,t_color,b_color);
	LCD_Puts(str,3,80,24,t_color,b_color);
	LCD_Puts("ID: ",4,0,40,t_color,b_color);
	U32ToHEX(device->ID,str);
	LCD_Puts(str,8,32,40,t_color,b_color);
	LCD_Puts("    Channels",12,0,56,t_color,b_color);
	
	
}
void WriteChannelsInfo(u8 dev_id,u8 ch_id,u16 t_color,u16 b_color)
{
	struct DeviceDesc *device=Devices.Device[dev_id];
	u8 ChCount=device->ChannelCount;
	u8 pos,x;
	u16 color1,color2;
	char str[3];
	if(ch_id<5) pos=0;
	else pos=ch_id-4;
	for(;pos<ChCount;pos++)
	{
		color1=color2=t_color;
		if(pos==ch_id)color1=b_color;
		else color2=b_color;
		U8ToStr(pos,str);
		str[0]=str[1];
		str[1]=str[2];
		x=0;
		LCD_Puts(str,2,x,72+(pos%5)*16,t_color,b_color);
		x+=24;
		if((((u8*)(device->Channels[pos]))[0] & UWB_Type_Mask)==UWB_Type_Digital)
		{
			LCD_Puts("Digital ",8,x,72+(pos%5)*16,color1,color2);
			x+=8*8;
		}
		else if((((u8*)(device->Channels[pos]))[0] & UWB_Type_Mask)==UWB_Type_Analog)
		{
			LCD_Puts("Analog ",7,x,72+(pos%5)*16,color1,color2);
			x+=8*7;
		}
		else if((((u8*)(device->Channels[pos]))[0] & UWB_Type_Mask)==UWB_Type_PWM)
		{
			LCD_Puts("PWM ",4,x,72+(pos%5)*16,color1,color2);
			x+=8*4;
		}
		else if((((u8*)(device->Channels[pos]))[0] & UWB_Type_Mask)==UWB_Type_Value)
		{
			LCD_Puts("Value ",6,x,72+(pos%5)*16,color1,color2);
			x+=8*6;
		}
		if((((u8*)(device->Channels[pos]))[0] & UWB_Type_DimMask) == UWB_Type_8bit)
		{
			LCD_Puts(" 8bit",5,x,72+(pos%5)*16,color1,color2);
			x+=8*5;
		}
		else if((((u8*)(device->Channels[pos]))[0] & UWB_Type_DimMask) == UWB_Type_16bit)
		{
			LCD_Puts("16bit",5,x,72+(pos%5)*16,color1,color2);
			x+=8*5;
		}
		else if((((u8*)(device->Channels[pos]))[0] & UWB_Type_DimMask) == UWB_Type_32bit)
		{
			LCD_Puts("32bit",5,x,72+(pos%5)*16,color1,color2);
			x+=8*5;
		}
		else if((((u8*)(device->Channels[pos]))[0] & UWB_Type_DimMask) == UWB_Type_F)
		{
			LCD_Puts("float",5,x,72+(pos%5)*16,color1,color2);
			x+=8*5;
		}
		else if((((u8*)(device->Channels[pos]))[0] & UWB_Type_DimMask) == UWB_Type_Str)
		{
			LCD_Puts(" str.",5,x,72+(pos%5)*16,color1,color2);
			x+=8*5;
		}
		while((x+8)<132)
		{		
			LCD_Putchar(' ',x,72+(pos%5)*16,color1,color2);
			x+=8;
		}			
	}
}
void WriteChannelInfo(u8 dev_id,u8 ch_id,u16 t_color,u16 b_color)
{
	struct DeviceDesc *device=Devices.Device[dev_id];
	struct DeviceChannelU8 *ch_u8;
	struct DeviceChannelU16 *ch_u16;
	struct DeviceChannelU32 *ch_u32;
	struct DeviceChannelF *ch_f;
	struct DeviceChannelStr *ch_str;
	u8 x=0,row=0;
	u8 l,pos=0;
	char str[8];
	int8_t key_id;
	PressedButton=0;
	LCD_FillScreen(b_color);
		
	ch_u8=device->Channels[ch_id];
	l=strlen(ch_u8->Name);
			while(l>0)
			{
				if(ch_u8->Name[pos]!='\n')
				{
					LCD_Putchar(ch_u8->Name[pos++],x,row*16,t_color,b_color);
					x+=8;
				 if(x>127)
			   {
					x=0;
					row++;
					if(row>2) break;
				 }
				}
				else 
				{
					pos++;
					x=0;
					row++;
					if(row>2) break;
				}
				
				l--;
			}
	 x=0;
	if((((u8*)(device->Channels[ch_id]))[0] & UWB_Type_Mask)==UWB_Type_Digital)
		{
			LCD_Puts("Digital ",8,x,48,t_color,b_color);
			x+=8*8;
		}
		else if((((u8*)(device->Channels[ch_id]))[0] & UWB_Type_Mask)==UWB_Type_Analog)
		{
			LCD_Puts("Analog ",7,x,48,t_color,b_color);
			x+=8*7;
		}
		else if((((u8*)(device->Channels[ch_id]))[0] & UWB_Type_Mask)==UWB_Type_PWM)
		{
			LCD_Puts("PWM ",4,x,48,t_color,b_color);
			x+=8*4;
		}
		else if((((u8*)(device->Channels[ch_id]))[0] & UWB_Type_Mask)==UWB_Type_Value)
		{
			LCD_Puts("Value ",6,x,48,t_color,b_color);
			x+=8*6;
		}
		
		if(((u8*)(device->Channels[ch_id]))[0] & UWB_Type_OnlyRead )
		{
			LCD_Puts("ReadOnly: true",14,0,4*16,t_color,b_color);
		}
		else 
		{
			LCD_Puts("ReadOnly: false",15,0,4*16,t_color,b_color);
		}
		
		if((((u8*)(device->Channels[ch_id]))[0] & UWB_Type_DimMask) == UWB_Type_8bit)
		{
			LCD_Puts(" 8bit",5,x,48,t_color,b_color);

			ch_u8=device->Channels[ch_id];
			LCD_Puts("Min:",4,0,5*16,t_color,b_color);
			U8ToStr(ch_u8->Min,str);
			LCD_Puts(str,3,32,5*16,t_color,b_color);
	    LCD_Puts("Max:",4,0,6*16,t_color,b_color);
			U8ToStr(ch_u8->Max,str);
			LCD_Puts(str,3,32,6*16,t_color,b_color);
	    LCD_Puts("Value:",6,0,7*16,t_color,b_color);
			U8ToStr(ch_u8->Value,str);
			LCD_Puts(str,3,48,7*16,t_color,b_color);
			
			
		}
		else if((((u8*)(device->Channels[ch_id]))[0] & UWB_Type_DimMask) == UWB_Type_16bit)
		{
			LCD_Puts("16bit",5,x,48,t_color,b_color);

			ch_u16=device->Channels[ch_id];
			LCD_Puts("Min:",4,0,5*16,t_color,b_color);
			U16ToHEX(ch_u16->Min,str);
			LCD_Puts(str,4,32,5*16,t_color,b_color);
			LCD_Puts("Max:",4,0,6*16,t_color,b_color);
			U16ToHEX(ch_u16->Max,str);
			LCD_Puts(str,4,32,6*16,t_color,b_color);
			LCD_Puts("Value:",6,0,7*16,t_color,b_color);
			U16ToHEX(ch_u16->Value,str);
			LCD_Puts(str,4,48,7*16,t_color,b_color);
		}
		else if((((u8*)(device->Channels[ch_id]))[0] & UWB_Type_DimMask) == UWB_Type_32bit)
		{
			LCD_Puts("32bit",5,x,48,t_color,b_color);

			ch_u32=device->Channels[ch_id];
			LCD_Puts("Min:",4,0,5*16,t_color,b_color);
			U32ToHEX(ch_u32->Min,str);
			LCD_Puts(str,8,40,5*16,t_color,b_color);
			LCD_Puts("Max:",4,0,6*16,t_color,b_color);
			U32ToHEX(ch_u32->Max,str);
			LCD_Puts(str,8,40,6*16,t_color,b_color);
			LCD_Puts("Value:",6,0,7*16,t_color,b_color);
			U32ToHEX(ch_u32->Value,str);
			LCD_Puts(str,8,48,7*16,t_color,b_color);
		}
		else if((((u8*)(device->Channels[ch_id]))[0] & UWB_Type_DimMask) == UWB_Type_F)
		{
			LCD_Puts("float",5,x,48,t_color,b_color);

			ch_f=device->Channels[ch_id];
			LCD_Puts("Min:",4,0,5*16,t_color,b_color);
			U32ToHEX(ch_f->Min,str);
			LCD_Puts(str,4,32,5*16,t_color,b_color);
			LCD_Puts("Max:",4,0,6*16,t_color,b_color);
			U32ToHEX(ch_f->Max,str);
			LCD_Puts(str,4,32,6*16,t_color,b_color);
			LCD_Puts("Value:",6,0,7*16,t_color,b_color);
			U32ToHEX((u32)ch_f->Value,str);
			LCD_Puts(str,8,48,7*16,t_color,b_color);
		}
		else if((((u8*)(device->Channels[ch_id]))[0] & UWB_Type_DimMask) == UWB_Type_Str)
		{
			LCD_Puts(" str.",5,x,48,t_color,b_color);
					
			ch_str=device->Channels[ch_id];
			LCD_Puts("Value:",6,0,5*16,t_color,b_color);
			x=8*6;
			row=5;
			pos=0;
			l=strlen(ch_str->Value);
			while(l>0)
			{
				LCD_Putchar(ch_str->Value[pos++],x,row*16,t_color,b_color);
				x+=8;
				if(x>127)
				{
					x=0;
					row++;
					if(row>9) break;
				}
				l--;
			}
		}
		LCD_DrawRect(153,85,20,44,1,t_color);
		LCD_Puts("NAME",4,92,156,t_color,b_color);
		if(!(((u8*)(device->Channels[ch_id]))[0] & UWB_Type_OnlyRead ))
		{
		LCD_Puts("Key:",4,0,8*16,t_color,b_color);
		key_id=GetKey(device->ID,((u8*)(device->Channels[ch_id]))[1]);
		if(key_id!=-1)
		{
			LCD_Putchar(Keys[key_id].Key,40,8*16,YELLOW,b_color);
		}
		else 
		{
			LCD_Puts("no",2,40,8*16,RED,b_color);
		}
		
		
		LCD_DrawRect(153,2,20,44,1,t_color);
		LCD_Puts("KEY",3,12,156,t_color,b_color);
		
	  }
		else 
		{
		LCD_DrawRect(153,2,20,44,1,t_color);
		LCD_Puts("DISP",4,8,156,t_color,b_color);
		}
		
}
void ChannelInfo(u8 dev_id,u8 ch_id,u16 t_color,u16 b_color)
{
	PressedButton=0;
	struct DeviceDesc *device=Devices.Device[dev_id];
	WriteChannelInfo(dev_id,ch_id,t_color,b_color);
	while(PressedButton!=10)
			{
				if(PressedButton==1)
				{
					if(!(((u8*)(device->Channels[ch_id]))[0] & UWB_Type_OnlyRead ))
					{
					WriteSetKeyMenu(device->ID,((u8*)(device->Channels[ch_id]))[1],t_color,b_color);
				  WriteChannelInfo(dev_id,ch_id,t_color,b_color);
					}
					else 
					{
						
					}
				}
				else if(PressedButton==3)
				{
					
				}
				PressedButton=0;
				while(PressedButton==0);
			}
}
void WriteSetKeyMenu(u32 dev_ID,u8 CID,u16 t_color,u16 b_color)
{
	u8 x;
	PressedButton=0;
	LCD_FillScreen(b_color);
	LCD_DrawRect(62,5,50,122,1,t_color);
	LCD_Puts("Press a key",11,21,72,t_color,b_color);
	for(x=1;x<112;x++)
	{
		LCD_FillRect(10,98,x,4,t_color);
		delay_ms(30);
		if(PressedButton!=0) 
		{
			//if(PressedButton==11) PressedButton=0;
			SetKey(dev_ID,CID,PressedButton);
			PressedButton=0;
			break;
		}
	}
}
void DeviceInfo(u8 dev_id,u16 t_color,u16 b_color)
{
	u8 ch_id=0;
	PressedButton=0;
	LCD_FillScreen(b_color);
		while(PressedButton!=10)
	{
		if(PressedButton==2)
		{
			if(ch_id>0)ch_id--;
			else ch_id=Devices.Device[dev_id]->ChannelCount-1;
		}
		else if(PressedButton==8)
		{
			if(ch_id<(Devices.Device[dev_id]->ChannelCount - 1)) ch_id++;
		  else ch_id=0;
		}
		else if(PressedButton==5)
		{
			ChannelInfo(dev_id,ch_id,t_color,b_color);
			
			LCD_FillScreen(b_color);
		}
		WriteDeviceInfo(dev_id,t_color,b_color);
		WriteChannelsInfo(dev_id,ch_id,t_color,b_color);
		PressedButton=0;
		while(PressedButton==0)
		{
			Power();
		}
	}
}
void DevicesMenu(u16 t_color,u16 b_color)
{
	PressedButton=0;
	LCD_FillScreen(b_color);
	u8 dev_id=0;
	u8 dev_id_max=Devices.Count;
	bool DeviceInfoShow=false;
	
	while(PressedButton!=10)
	{
		if(PressedButton==2)
		{
			if(dev_id>0)dev_id--;
			else dev_id=dev_id_max;
		}
		else if(PressedButton==8)
		{
			if(dev_id<dev_id_max) dev_id++;
			else dev_id=0;
		}
		else if(PressedButton==5)
		{
			DeviceInfo(dev_id,t_color,b_color);	
			LCD_FillScreen(b_color);
		}
		WriteDevicesMenu(dev_id,t_color,b_color);
		PressedButton=0;
		while(PressedButton==0)
		{
			Power();
		}
	}
	PressedButton=0;
	LCD_FillScreen(b_color);
}
void WriteKeyPad(void)
{
	u8 key;
	u8 x,y;
	u16 button_color=GREEN;
	char str[3];
	LCD_FillScreen(BLACK);
	for(key=1;key<13;key++)
	{
		x=10+39*((key-1)%3);
		y=10+35*((key-1)/3);
		if(Keys[key].DevID!=0)button_color=RED;
		else button_color=GREEN;
		LCD_FillRect(x,y,34,25,button_color);
		if(key<10 || key == 11) LCD_Putchar(Keys[key].Key,x+13,y+4,WHITE,button_color);
		else if(key==10)
		{
			LCD_Puts("MENU",4,x+1,y+4,WHITE,button_color);			
		}
		else if(key==12)
		{
			LCD_Puts("PWR",3,x+5,y+4,WHITE,button_color);			
		}
	}
}
void* GetChannelDesc(struct DeviceDesc *device,u8 CID)
{
	void* ch_desc;
	u8 ch_id;
	for(ch_id=0;ch_id<device->ChannelCount;ch_id++)
	{
		if(((struct DeviceChannelU8*)device->Channels[ch_id])->CID==CID)
		{
			return device->Channels[ch_id];
		}
	}
	return NULL;
}
void KeyPadMenu(u16 t_color,u16 b_color)
{
	u8 l;
	struct DeviceDesc *device;
	void *ch_desc;
	WriteKeyPad();
	PressedButton=0;
	while(PressedButton!=10)
		{
			PressedButton=0;
			while(PressedButton==0)Power();
			if(Keys[PressedButton].DevID!=0)
			{
			device=GetDeviceByID(Keys[PressedButton].DevID);
			l=strlen(device->Name);
			if(l>16) l=16;
			LCD_Puts(device->Name,l,0,142,t_color,b_color);
			ch_desc=GetChannelDesc(device,Keys[PressedButton].CID);
				l=strlen(((struct DeviceChannelU8*)ch_desc)->Name);
				if(l>16)l=16;
			  LCD_Puts(((struct DeviceChannelU8*)ch_desc)->Name,l,0,158,t_color,b_color);
			}
		}
	PressedButton=0;
	LCD_FillScreen(b_color);
}

void FindDevicesMenu(u16 t_color,u16 b_color)
{
	LCD_FillScreen(b_color);
	
	SendCmd(UWP_Cmd_DeviceName);
	while(NRF24L01_TX_RUN);
	rx_mode();
  while(!RF_RX)
	{
		HAL_Delay(2000);
	}
	tx_mode();
}
void Menu(u16 t_color,u16 b_color)
{
	PressedButton=0;
   LCD_FillScreen(b_color);
	while(!(SelectedMenuPos==0 && PressedButton==5))
	{
		
		if(PressedButton==2)
		{
			if(SelectedMenuPos>0)SelectedMenuPos--;
		  else SelectedMenuPos=MenuCount-1;
		}
    else if(PressedButton==8)
		{
      if(SelectedMenuPos<(MenuCount-1))SelectedMenuPos++;
			else SelectedMenuPos=0;
		}
		else if(PressedButton==5)
		 {
			switch (SelectedMenuPos)
			                                                                                                                                                                            {
				case 1:
					DevicesMenu(t_color,b_color);
					break;
				case 2:
					FindDevicesMenu(t_color,b_color);
					break;
				case 3:
					KeyPadMenu(t_color,b_color);
					break;
				case 4:
					break;
				case 5:
				  break;
				default:
					break;
			}
		}
		
		WriteMenu(t_color,b_color);
		
		PressedButton=0;
		while(PressedButton==0)
		{
			Power();
		}
		
	}
	PressedButton=0;
	LCD_FillScreen(b_color);
}
void InitAll(void)
{
	 HAL_Init();
   SystemClock_Config();

	 MX_GPIO_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();

	__HAL_SPI_ENABLE(&hspi2);
	 HAL_ADC_Start(&hadc1);
	 HAL_TIM_Base_Start_IT(&htim1);
	
	 //MX_GPIO_Init();
	 HAL_GPIO_WritePin(LCD_POWER_GPIO_Port,LCD_POWER_Pin,GPIO_PIN_SET);
	 HAL_GPIO_WritePin(RF_MODULE_POWER_GPIO_Port,RF_MODULE_POWER_Pin,GPIO_PIN_SET);
	 soft_delay_ms(10);
	 LCD_init();
	 LCD_FillScreen(FonColor);
   NRF24L01_init();
	 SetRxAddr(Addr);
	 Addr[4]=0xF0;
	 SetTxAddr(Addr);
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
   u8 key;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
 // HAL_Init();

  /* Configure the system clock */
 // SystemClock_Config();

  /* Initialize all configured peripherals */
 /* MX_GPIO_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();*/

  /* USER CODE BEGIN 2 */
/*	 __HAL_SPI_ENABLE(&hspi2);
	 HAL_ADC_Start(&hadc1);
	 HAL_TIM_Base_Start_IT(&htim1);*/
  
	
	// HAL_Delay(500);
   InitAll();
	 
	 for(key=1;key<10;key++) Keys[key].Key='0'+key;
	 Keys[11].Key='0';
	 
   Devices.Count=1;
	 Devices.Device[0]=malloc(sizeof(struct DeviceDesc));
	 
	 Devices.Device[0]->Name="Test";
	 Devices.Device[0]->ChannelCount=4;
	 Devices.Device[0]->DescStr=" ";
	 Devices.Device[0]->ID=0xA0B1C2D3;
	 Devices.Device[0]->Channels=(void*) malloc(sizeof(void*)*4);
	 Devices.Device[0]->Channels[0]=malloc (sizeof(struct DeviceChannelU8));
	 Devices.Device[0]->Channels[1]=malloc (sizeof(struct DeviceChannelU32));
	 Devices.Device[0]->Channels[2]=malloc (sizeof(struct DeviceChannelF));
   Devices.Device[0]->Channels[3]=malloc (sizeof(struct DeviceChannelStr));
	  //((struct DeviceChannelStr*)Devices.Device[0]->Channels[3])->Value=msgs[4];
		//((struct DeviceChannelU8*)Devices.Device[0]->Channels[0])->Name=msgs[0];
	/*  ((struct DeviceChannelU8*)Devices.Device[0]->Channels[1])->Name=msgs[1];
	  ((struct DeviceChannelU8*)Devices.Device[0]->Channels[2])->Name=msgs[2];
		((struct DeviceChannelU8*)Devices.Device[0]->Channels[3])->Name=msgs[3];*/
		
	 ((struct DeviceChannelU8*)Devices.Device[0]->Channels[0])->CID=1;
	 ((struct DeviceChannelU8*)Devices.Device[0]->Channels[0])->Name=malloc(8);
	 ((struct DeviceChannelU8*)Devices.Device[0]->Channels[0])->Name[0]='c';
	 ((struct DeviceChannelU8*)Devices.Device[0]->Channels[0])->Name[1]='h';
	 ((struct DeviceChannelU8*)Devices.Device[0]->Channels[0])->Name[2]='a';
	 ((struct DeviceChannelU8*)Devices.Device[0]->Channels[0])->Name[3]='n';
	 ((struct DeviceChannelU8*)Devices.Device[0]->Channels[0])->Name[4]='n';
	 ((struct DeviceChannelU8*)Devices.Device[0]->Channels[0])->Name[5]='e';
	 ((struct DeviceChannelU8*)Devices.Device[0]->Channels[0])->Name[6]='l';
	 ((struct DeviceChannelU8*)Devices.Device[0]->Channels[0])->Name[7]='0';
	 
	 ((struct DeviceChannelU8*)Devices.Device[0]->Channels[0])->Type=UWB_Type_8bit | UWB_Type_Analog;
	 ((struct DeviceChannelU8*)Devices.Device[0]->Channels[1])->Type=UWB_Type_32bit | UWB_Type_Digital;
	 ((struct DeviceChannelU8*)Devices.Device[0]->Channels[2])->Type=UWB_Type_F | UWB_Type_Analog;
	 ((struct DeviceChannelU8*)Devices.Device[0]->Channels[3])->Type=UWB_Type_Str | UWB_Type_OnlyRead | UWB_Type_Value ;
	 
	 ((struct DeviceChannelU8*)Devices.Device[0]->Channels[0])->Max=100;
	 ((struct DeviceChannelU8*)Devices.Device[0]->Channels[0])->Min=1;
	 ((struct DeviceChannelU8*)Devices.Device[0]->Channels[0])->Value=50;
	
	 ((struct DeviceChannelU32*)Devices.Device[0]->Channels[1])->Max=0xAABBCCDD;
	 ((struct DeviceChannelU32*)Devices.Device[0]->Channels[1])->Min=0;
	 ((struct DeviceChannelU32*)Devices.Device[0]->Channels[1])->Value=0xff;
	 
	 //((struct DeviceChannelU8*)Devices.Device[0]->Channels[0])
   key=0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	//eBuffer[0]='A';
	
	// EEPROM_Write(0x00,eBuffer,1);
	//HAL_Delay(100);
//	eBuffer[0]='B';
	// EEPROM_Read(0x00,eBuffer,1);
	
	// LCD_Putchar(eBuffer[0],10,10,RED,BLACK);
	
   
	 HAL_Delay(100);
	 WriteTime();
	 WriteVoltage();
	 Power();
		if(PressedButton==10)
		{
			Menu(WHITE,BLACK);
		}
		if(PressedButton!=0 && key==0)
		{
			KeyPresed(PressedButton);
			key=PressedButton;
		}
		else key=PressedButton;
		PressedButton=0;
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 17999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CE_Pin|CSN_Pin|RF_MODULE_POWER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_POWER_Pin|LCD_RS_Pin|LCD_RESET_Pin|LCD_CS_Pin 
                          |R1_Pin|R2_Pin|R3_Pin|LED_RED_Pin 
                          |LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CE_Pin CSN_Pin RF_MODULE_POWER_Pin */
  GPIO_InitStruct.Pin = CE_Pin|CSN_Pin|RF_MODULE_POWER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : IRQ_Pin */
  GPIO_InitStruct.Pin = IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_POWER_Pin LCD_RS_Pin LCD_RESET_Pin LCD_CS_Pin 
                           LED_RED_Pin LED_BLUE_Pin */
  GPIO_InitStruct.Pin = LCD_POWER_Pin|LCD_RS_Pin|LCD_RESET_Pin|LCD_CS_Pin 
                          |LED_RED_Pin|LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : L1_Pin L2_Pin L3_Pin L4_Pin */
  GPIO_InitStruct.Pin = L1_Pin|L2_Pin|L3_Pin|L4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : R1_Pin R2_Pin R3_Pin */
  GPIO_InitStruct.Pin = R1_Pin|R2_Pin|R3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void KeyPresed(uint8_t key)
{
	uint8_t msg[3];
	msg[0]=('0'+(char)(key/10));
	msg[1]=('0'+(char)(key%10));
	//msg[2]=',';
	send_data(msg,2);
}
uint8_t KeyPadRead(void)
{
	uint8_t key=0;
	PowerDownTimeCounter=0;
	if(PowerDown)
	{
		PowerOn();
	}
	HAL_GPIO_WritePin(R1_GPIO_Port,R1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R2_GPIO_Port,R2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(R3_GPIO_Port,R3_Pin,GPIO_PIN_SET);
	delay_us(10);
	if(HAL_GPIO_ReadPin(L1_GPIO_Port,L1_Pin)==0) key=1;
	if(HAL_GPIO_ReadPin(L2_GPIO_Port,L2_Pin)==0) key=4;
	if(HAL_GPIO_ReadPin(L2_GPIO_Port,L3_Pin)==0) key=7;
	if(HAL_GPIO_ReadPin(L4_GPIO_Port,L4_Pin)==0) key=10;
	HAL_GPIO_WritePin(R1_GPIO_Port,R1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(R2_GPIO_Port,R2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R3_GPIO_Port,R3_Pin,GPIO_PIN_SET);
	delay_us(10);
	if(HAL_GPIO_ReadPin(L1_GPIO_Port,L1_Pin)==0) key=2;
	if(HAL_GPIO_ReadPin(L2_GPIO_Port,L2_Pin)==0) key=5;
	if(HAL_GPIO_ReadPin(L2_GPIO_Port,L3_Pin)==0) key=8;
	if(HAL_GPIO_ReadPin(L4_GPIO_Port,L4_Pin)==0) key=11;
	HAL_GPIO_WritePin(R1_GPIO_Port,R1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(R2_GPIO_Port,R2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(R3_GPIO_Port,R3_Pin,GPIO_PIN_RESET);
	delay_us(10);
	if(HAL_GPIO_ReadPin(L1_GPIO_Port,L1_Pin)==0) key=3;
	if(HAL_GPIO_ReadPin(L2_GPIO_Port,L2_Pin)==0) key=6;
	if(HAL_GPIO_ReadPin(L2_GPIO_Port,L3_Pin)==0) key=9;
	if(HAL_GPIO_ReadPin(L4_GPIO_Port,L4_Pin)==0) key=12;
	HAL_GPIO_WritePin(R1_GPIO_Port,R1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R2_GPIO_Port,R2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R3_GPIO_Port,R3_Pin,GPIO_PIN_RESET);
	delay_us(10);
	if(key!=0)
	{
	//LCD_Putchar(('0'+(char)(key/10)),0,0,RED,GREEN);
	//LCD_Putchar(('0'+(char)(key%10)),8,0,RED,GREEN);
		//KeyPresed(key);
		soft_delay_ms(10);
		PressedButton=key;
	}	
	return key;
}


void NRF24L01_IRQ(void)
{
	state=read_irq();
	if(state&0x40)
	{
		RF_Buffer_Len=read_rx(RF_Buffer);
		RF_RX=true;
	}
	if(state&0x10)
	{
		NRF24L01_TX_RUN=false;
		csn_0
    spi(FLUSH_TX);
    csn_1
	}
	if(state & 0x1)
	{
		csn_0
		spi(FLUSH_TX);
		csn_1
	}
	if(state & 0x20)
	{
		NRF24L01_TX_RUN=false;
	}
  clr_irq();
}
void ReadTime(unsigned char *time)
{
	HAL_RTC_GetTime(&hrtc,&Time,RTC_FORMAT_BCD);
	time[0]=BCD2HEX(Time.Hours);
	time[1]=BCD2HEX(Time.Minutes);
}
void WriteTime(void)
{ unsigned char time[2];
  unsigned char time_string[5];

	  ReadTime(time);
		time_string[0]='0'+((time[0]/10) %10);
	  time_string[1]='0'+(time[0] %10);
	  time_string[2]=':';
		time_string[3]='0'+((time[1]/10) %10);
	  time_string[4]='0'+(time[1] %10);
	
	  LCD_Put24(time_string[0],3+0,20,WHITE,BLACK);
	  LCD_Put24(time_string[1],6+24,20,WHITE,BLACK);
	  
	  LCD_Put24(time_string[3],6+72,20,WHITE,BLACK);
	  LCD_Put24(time_string[4],9+96,20,WHITE,BLACK);

	  ShowSecDots();
}
void ShowSecDots(void)
{
	if(SecDotsShow)
	{
		LCD_Put24(':',6+48,20,WHITE,BLACK);
	}
	else
	{
		LCD_Put24(' ',6+48,20,WHITE,BLACK);
	}
	
}
void EEPROM_Read(uint16_t address,uint8_t *buffer,uint16_t size)
{
	HAL_I2C_Mem_Read (&hi2c1, (uint16_t) I2C1_DEVICE_ADDRESS << 1, address, 1, buffer, size, 10); 
}
void EEPROM_Write(uint16_t address,uint8_t *buffer,uint16_t size)
{
	HAL_I2C_Mem_Write(&hi2c1, (uint16_t) I2C1_DEVICE_ADDRESS << 1, address, 1, buffer, size, 10);
}
void RackageProcc(u8 *buf,u8 size)
{
	u16 Size;
	u8 pos;
	u32 ID;
	struct TempDevice *device;
	if(size<6)return;
	if(buf[0]==UWB_FirstMarker)
	{
		if(buf[1]==UWB_DataMarker)
		{
			if(size>8)
			{
			U8ToU16.Bytes=&buf[2];
			Size=U8ToU16.Value;
			
			U8ToU32.Bytes=&buf[4];
			ID=U8ToU32.Value;
			
				device=GetTempDeviceByID(ID);
				device->pos=0;
				device->ID=ID;
				device->Size=Size;
				for(pos=8;pos<size;pos++)
				{
					device->Buffer[device->pos++]=buf[pos];
				}
				device->SizeNow=size;
			}
		}
		else if(buf[1]==UWB_DataContinueMarker)
		{
			U8ToU32.Bytes=&buf[2];
			ID=U8ToU32.Value;
			device=GetTempDeviceByID(ID);
			if(device->ID!=ID) return;
			for(pos=6;pos<size;pos++)
				{
					device->Buffer[device->pos++]=buf[pos];
				}
				device->SizeNow+=size;
		}
		if(device->Size==device->SizeNow)
		{
			UnZipPackage(device);
		}
	}
}

struct TempDevice* GetTempDeviceByID(u32 ID)
{
	u8 index;
	for(index=0;index<TempDevices.Count;index++)
	{
		if(TempDevices.Device[index]->ID==ID)
		{
			return TempDevices.Device[index];
		}
	}
	if(TempDevices.Count<DevicesMaxCount)TempDevices.Count++;
	return TempDevices.Device[index];
}

struct DeviceDesc* GetDeviceByID(u32 ID)
{
	u8 index;
	for(index=0;index<Devices.Count;index++)
	{
		if(Devices.Device[index]->ID==ID) return Devices.Device[index];
	}
	Devices.Count++;
	return Devices.Device[index];
}
void UnZipPackage(struct TempDevice* device)
{
	u8  NamePos,DescStrPos;
	u16 length;
	u8  ChannelsCount;
	u8  ChannelNumber=0;
	u16 ChannelSize;
	u8  ChannelType;
	u8  ChannelNamePos;
	u16 ChannelStartPos;
	u16 pos;
	u16 ChannelNameSize;
	u8  CID;
	u8  *buffer=device->Buffer;
	struct DeviceDesc* new_device=GetDeviceByID(device->ID);
	
	U8ToU16.Bytes=buffer;
	NamePos=U8ToU16.Value;
	U8ToU16.Bytes=buffer;
	DescStrPos=U8ToU16.Value;
	ChannelsCount=buffer[4];
	
	new_device->ID=device->ID;
	new_device->size=device->Size;
	new_device->ChannelCount=ChannelsCount;
	new_device->Channels=(void*) malloc(ChannelsCount);
	
	  pos=5;
		ChannelStartPos=pos;
	while(ChannelsCount-- > 0)
	{
		
	   ChannelStartPos=pos;
		if(buffer[pos++]==UWB_FirstMarker && buffer[pos++]==UWB_ChannelMarker)
		{
			U8ToU16.Bytes=&buffer[pos++];
			ChannelSize=U8ToU16.Value;
			pos++;
			CID=buffer[pos++];
			ChannelType=buffer[pos++];
			if((ChannelType & UWB_Type_DimMask )==UWB_Type_8bit)
			{
				new_device->Channels[ChannelNumber]=malloc(sizeof(struct DeviceChannelU8));
				((struct DeviceChannelU8*)new_device->Channels[ChannelNumber])->CID=CID;
				((struct DeviceChannelU8*)new_device->Channels[ChannelNumber])->Type=ChannelType;
				((struct DeviceChannelU8*)new_device->Channels[ChannelNumber])->Min=buffer[pos++];
				((struct DeviceChannelU8*)new_device->Channels[ChannelNumber])->Max=buffer[pos++];
				((struct DeviceChannelU8*)new_device->Channels[ChannelNumber])->Value=buffer[pos++];
				
			}
			else if((ChannelType & UWB_Type_DimMask )==UWB_Type_16bit)
			{
				new_device->Channels[ChannelNumber]=malloc(sizeof(struct DeviceChannelU16));
				((struct DeviceChannelU8*)new_device->Channels[ChannelNumber])->CID=CID;
				((struct DeviceChannelU8*)new_device->Channels[ChannelNumber])->Type=ChannelType;
				 U8ToU16.Bytes=&buffer[pos++];
				 pos++;
        ((struct DeviceChannelU8*)new_device->Channels[ChannelNumber])->Min=U8ToU16.Value;
				 U8ToU16.Bytes=&buffer[pos++];
				 pos++;
				((struct DeviceChannelU8*)new_device->Channels[ChannelNumber])->Max=U8ToU16.Value;
				 U8ToU16.Bytes=&buffer[pos++];
				 pos++;
				((struct DeviceChannelU8*)new_device->Channels[ChannelNumber])->Value=U8ToU16.Value;
				
			}
			else if((ChannelType & UWB_Type_DimMask )==UWB_Type_32bit)
			{
				new_device->Channels[ChannelNumber]=malloc(sizeof(struct DeviceChannelU32));
				((struct DeviceChannelU8*)new_device->Channels[ChannelNumber])->CID=CID;
				((struct DeviceChannelU8*)new_device->Channels[ChannelNumber])->Type=ChannelType;
				 U8ToU32.Bytes=&buffer[pos];
				 pos+=4;
        ((struct DeviceChannelU8*)new_device->Channels[ChannelNumber])->Min=U8ToU32.Value;
				 U8ToU32.Bytes=&buffer[pos];
				 pos+=4;
				((struct DeviceChannelU8*)new_device->Channels[ChannelNumber])->Max=U8ToU32.Value;
				 U8ToU32.Bytes=&buffer[pos];
				 pos+=4;
				((struct DeviceChannelU8*)new_device->Channels[ChannelNumber])->Value=U8ToU32.Value;
				
			}
			else if((ChannelType & UWB_Type_DimMask )==UWB_Type_F)
			{
				new_device->Channels[ChannelNumber]=malloc(sizeof(struct DeviceChannelU32));
				((struct DeviceChannelU8*)new_device->Channels[ChannelNumber])->CID=CID;
				((struct DeviceChannelU8*)new_device->Channels[ChannelNumber])->Type=ChannelType;
				 U8ToF.Bytes=&buffer[pos];
				 pos+=4;
        ((struct DeviceChannelU8*)new_device->Channels[ChannelNumber])->Min=U8ToF.Value;
				 U8ToF.Bytes=&buffer[pos];
				 pos+=4;
				((struct DeviceChannelU8*)new_device->Channels[ChannelNumber])->Max=U8ToF.Value;
				 U8ToF.Bytes=&buffer[pos];
				 pos+=4;
				((struct DeviceChannelU8*)new_device->Channels[ChannelNumber])->Value=U8ToF.Value;
				
			}
		 ChannelNameSize=device->Size+2-(pos-ChannelStartPos);	
			
		  ((struct DeviceChannelU8*)new_device->Channels[ChannelNumber])->Name= malloc(ChannelNameSize);
		  ChannelNamePos=0;
     do
		 { 
			 ((struct DeviceChannelU8*)new_device->Channels[ChannelNumber])->Name[ChannelNamePos++]=buffer[pos];
			 
		 }while(buffer[pos++]!=0);
     	
		}
		else return ;
	}
	pos=NamePos;
	NamePos=0;
	length=strlen((const char*)&buffer[pos]);
	new_device->Name=malloc(length);
	do{
		new_device->Name[NamePos++]=buffer[pos];
	}while(buffer[pos++]!=0);
	
	pos=DescStrPos;
	DescStrPos=0;
	length=strlen((const char*)&buffer[pos]);
	new_device->DescStr=malloc(length);
	do{
		new_device->DescStr[DescStrPos++]=buffer[pos];
	}while(buffer[pos++]!=0);
	
}
void U8ToStr(u8 data,char *str)
{
	str[0]='0'+data/100;
	data%=100;
	str[1]='0'+data/10;
	str[2]='0'+data%10;
	if(str[0]=='0')
	{
		str[0]=' ';
		if(str[1]=='0') str[1]=' ';
	}
}
void U8ToHEX(u8 data,char *str)
{
	str[0]=data/16;
	str[1]=data%16;
	if(str[0]<10)str[0]+='0';
	else str[0]='A'-10+str[0];
	if(str[1]<10)str[1]+='0';
	else str[1]='A'-10+str[1];
}
void U16ToHEX(u16 data,char *str)
{
	char temp[2];
	u8 bytes[4];
	U16ToU8.Value=data;
	U8ToHEX(U16ToU8.Bytes[1],temp);
	str[0]=temp[0];
	str[1]=temp[1];
	U8ToHEX(U16ToU8.Bytes[0],temp);
	str[2]=temp[0];
	str[3]=temp[1];
}
void U32ToHEX(u32 data,char *str)
{
	char temp[2];
	u8 bytes[4];
	U32ToU8.Value=data;
  U8ToHEX(U32ToU8.Bytes[3],temp);
	str[0]=temp[0];
	str[1]=temp[1];
	U8ToHEX(U32ToU8.Bytes[2],temp);
	str[2]=temp[0];
	str[3]=temp[1];
	U8ToHEX(U32ToU8.Bytes[1],temp);
	str[4]=temp[0];
	str[5]=temp[1];
	U8ToHEX(U32ToU8.Bytes[0],temp);
	str[6]=temp[0];
	str[7]=temp[1];
}
int8_t GetKey(u32 dev_ID,u8 CID)
{
	u8 key;
	if(dev_ID==0) return -1;
	for(key=1;key<13;key++)
	{
		if(Keys[key].DevID==dev_ID && Keys[key].CID==CID)
		{
			return key;
		}
	}
	return -1;
}
int8_t SetKey(u32 dev_ID,u8 CID,u8 key)
{
	int8_t old_key;
	old_key=GetKey(dev_ID,CID);
	if(old_key!=-1)
	{
		Keys[old_key].CID=0;
		Keys[old_key].DevID=0;
	}
	if((key>0 && key<10) || key==11)
	{
		Keys[key].DevID=dev_ID;
		Keys[key].CID=CID;
		return key;
	}

	else return -1;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
