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
#include "LPH91572.c"
#include "NRF24L01.c"
#include "Bat1.h"

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


char options[8][17]=
{"  Back           ",
 "  Devices        ",
 "  Find devices   ",
 "  Keypad         ",
 "  Power          ",
 "                 ",
 "                 ",
 "                 "};	

#define MenuCount 5
uint8_t SelectedMenuPos = 1;
#define MenuShiftY 25
 
volatile u8 PressedButton=0;
 
bool SecDotsShow=false;
 
struct DeviceStruct
{
	int size;
	char Name[28];
};
struct DevicesStruct
{
	u8 Count;
	struct DeviceStruct Device[15];
}Devices;
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
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
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
	PowerDownTimeCounter++;
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
	 if (SelectedMenuPos==pos) 
		    LCD_Puts(options[pos],17,0,pos*16+MenuShiftY,b_color,t_color);
	 else LCD_Puts(options[pos],17,0,pos*16+MenuShiftY,t_color,b_color);    
	
}
void WriteDevicesMenu(u8 dev_id,u16 t_color,u16 b_color)
{
	u8 pos=0,f_pos=0;
	if(dev_id<8) pos=f_pos=0;
	else pos=f_pos=dev_id-7;
	for(;(pos<Devices.Count) && (pos < f_pos+8);pos++)
	{
	 if(pos==dev_id)
	 {    
		    LCD_Putchar(('0'+dev_id/10),0,pos*16+MenuShiftY,b_color,t_color);
		    LCD_Putchar(('0'+dev_id%10),8,pos*16+MenuShiftY,b_color,t_color);
		    LCD_Puts(Devices.Device[pos].Name,14,24,pos*16+MenuShiftY,b_color,t_color);
	 }
	 else 
	 {
		    LCD_Putchar(('0'+dev_id/10),0,pos*16+MenuShiftY,t_color,b_color);
		    LCD_Putchar(('0'+dev_id%10),8,pos*16+MenuShiftY,t_color,b_color);
		    LCD_Puts(Devices.Device[pos].Name,14,24,pos*16+MenuShiftY,t_color,b_color);	
	 }
	}
}
void DevicesMenu(u16 t_color,u16 b_color)
{
	PressedButton=0;
	LCD_FillScreen(b_color);
	u8 dev_id=0;
	WriteDevicesMenu(dev_id,t_color,b_color);
	while(PressedButton==10)
	{
		
	}
	PressedButton=0;
	LCD_FillScreen(b_color);
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
					break;
				case 3:
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
			//Power();
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
	
	
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
 // HAL_Init();

  /* Configure the system clock */
 // SystemClock_Config();

  /* Initialize all configured peripherals */
/*  MX_GPIO_Init();
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

  //HAL_Delay(3000);
	//PowerOff();
	//Power();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  htim1.Init.Prescaler = 7999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
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

  /*Configure GPIO pins : PD0 PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_PD01_ENABLE();

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
	msg[2]=',';
	send_data(msg,3);
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
	HAL_GPIO_WritePin(R1_GPIO_Port,R1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(R2_GPIO_Port,R2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(R3_GPIO_Port,R3_Pin,GPIO_PIN_SET);
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
{/*
	state=read_irq();
	if(state&0x40)
	{
		i=read_rx(buffer);
		ReadPackage(buffer);
		PackageReceived=true;
		clr_irq();
		TimeOutTimer=0;
	}
	if(state&0x10)
	{
		if(TransmitRepeats==3)
		{
		ConnectError=true;
		TransmitRepeats=0;
		clr_irq();
		NRF24L01_power_down();
		}
		else
		{			
			TransmitRepeats++;
			csn_0
      spi(FLUSH_TX);
      csn_1
		}
	}
	if(state & 0x1)
	{
		csn_0
		spi(FLUSH_TX);
		csn_1
	}
	if(state & 0x20)
	{
		NRF24L01_Transmit=false;
		TransmitRepeats=0;
		clr_irq();
	//	if(NRF24L01_POWER) rx_mode();
	}*/
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
