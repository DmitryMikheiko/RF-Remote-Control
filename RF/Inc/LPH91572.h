//--------Библиотека дисплея Siemens C75, МЕ75---------
//           Зеленый текстолит LPH9157-2
//              132х176 пикселей
//          ==== Кизим Игорь ====
//-----------------------------------------------------

#include "stm32f1xx_hal.h"
#include "Symbols.h"
#include "LS020_font.h"

#include "0S.h"
#include "1S.h"
#include "2S.h"
#include "3S.h"
#include "4S.h"
#include "5S.h"
#include "6S.h"
#include "7S.h"
#include "8S.h"
#include "9S.h"
#include "SS.h"
#include "S_Dot.h"

typedef uint8_t u8 ;
typedef uint16_t u16; 
typedef uint32_t u32; 
typedef int16_t s16;

  #define LS020_Comp
//#define _8_BIT_COLOR  //Если закомментировано - 16-ти битный цвет
#define _GEOMETRICAL  //Использование функций вывода геометрических фигур
//#define ROT_90 //поворот на 90 градусов
//===============================================================
//		            Назначение выводов порта 
//===============================================================
#define LCD_CS 		  LCD_CS_Pin //Выбор чипа
#define LCD_RESET 	LCD_RESET_Pin //Сброс
#define LCD_RS 		  LCD_RS_Pin	//CD - тип передаваемых данных
#define LCD_DATA    LCD_DATA_Pin
#define LCD_CLK     LCD_CLK_Pin
#define LCD_CS_0    HAL_GPIO_WritePin(LCD_CS_GPIO_Port,LCD_CS,GPIO_PIN_RESET);
#define LCD_CS_1    HAL_GPIO_WritePin(LCD_CS_GPIO_Port,LCD_CS,GPIO_PIN_SET);
#define LCD_RS_0    HAL_GPIO_WritePin(LCD_RS_GPIO_Port,LCD_RS,GPIO_PIN_RESET);
#define LCD_RS_1    HAL_GPIO_WritePin(LCD_RS_GPIO_Port,LCD_RS,GPIO_PIN_SET);
#define LCD_RESET_0 HAL_GPIO_WritePin(LCD_RESET_GPIO_Port,LCD_RESET,GPIO_PIN_RESET);
#define LCD_RESET_1 HAL_GPIO_WritePin(LCD_RESET_GPIO_Port,LCD_RESET,GPIO_PIN_SET);
#define LCD_DATA_0  HAL_GPIO_WritePin(LCD_DATA_GPIO_Port,LCD_DATA,GPIO_PIN_RESET);
#define LCD_DATA_1  HAL_GPIO_WritePin(LCD_DATA_GPIO_Port,LCD_DATA,GPIO_PIN_SET);
#define LCD_CLK_0  HAL_GPIO_WritePin(LCD_CLK_GPIO_Port,LCD_CLK,GPIO_PIN_RESET);
#define LCD_CLK_1  HAL_GPIO_WritePin(LCD_CLK_GPIO_Port,LCD_CLK,GPIO_PIN_SET);

#define delay_ms HAL_Delay

extern void delay_us(uint16_t time_);
extern void LcdWD(uint8_t  data);
//#define LCD_CLK 	  PORTC.3 //Синхронизация
//#define LCD_DATA 	  PORTC.4 //Данные

//*************************************************************
//Команда/Данные
#define CMD 0
#define DAT 1
char RS_old;
  
//===============================================================
//     			    Определение цветов
//===============================================================
#ifdef _8_BIT_COLOR
//8-ми битовая цветовая палитра (256 цветов)
#define GREEN       0x1C
#define DARK_GREEN  0x15
#define RED         0xE0
#define BLUE        0x1F    
#define DARK_BLUE   0x03
#define YELLOW      0xFC
#define ORANGE      0xEC
#define VIOLET      0xE3
#define WHITE       0xFF
#define BLACK       0x00
#define GREY        0x6D

#else
//16-ти битовая цветовая палитра (65536 цветов)
#define    BLACK                0x0000 
#define    WHITE                0xFFFF 
#define    GRAY                 0xE79C
#define    GREEN                0x07E0 
#define    BLUE                 0x001F 
#define    RED                  0xF800 
#define    SKY                  0x5d1c 
#define    YELLOW               0xffe0 
#define    MAGENTA              0xf81f
#define    CYAN                 0x07ff
#define    ORANGE               0xfca0 
#define    PINK                 0xF97F
#define    BROWN                0x8200
#define    VIOLET               0x9199
#define    SILVER               0xa510
#define    GOLD                 0xa508
#define    BEGH                 0xf77b
#define    NAVY                 0x000F      
#define    DARK_GREEN           0x03E0      
#define    DARK_CYAN            0x03EF      
#define    MAROON               0x7800      
#define    PURPLE               0x780F      
#define    OLIVE                0x7BE0      
#define    LIGHT_GREY           0xC618      
#define    DARK_GREY            0x7BEF
#endif

//*************************************************************
//ПРОТОТИПЫ ФУНКЦИЙ
void LCD_init (void);
void LCD_off(void);
void Send_to_lcd (unsigned char RS, unsigned char data);
void SetArea (char x1, char x2, char y1, char y2);
void Put_Pixel (char x, char y, unsigned int color);
#ifdef LS020_Comp
void LCD_Putchar (char symbol, char x, char y, int t_color, int b_color);    
void LCD_Puts(char *str,int length, int x, int y,  int t_color, int b_color);

#else
void Send_Symbol (unsigned char symbol, char x, char y, int t_color, int b_color, char zoom_width, char zoom_height, int rot);  
void LCD_Putchar (char symbol, char x, char y, int t_color, int b_color, char zoom_width, char zoom_height, int rot);    
void LCD_Puts(char *str,int length, int x, int y,  int t_color, int b_color, char zoom_width, char zoom_height, int rot);
#endif
void LCD_Put24Digit(uint16_t xs,uint16_t ys,const uint8_t *picture,uint16_t size,int t_color,int b_color);
void LCD_Put24(char symbol,uint8_t x,uint8_t y,int t_color,int b_color);
void Send_Symbol_Shadow (unsigned char symbol, char x, char y, int t_color, char zoom_width, char zoom_height, int rot);
void LCD_Putchar_Shadow (char symbol, char x, char y, int t_color, char zoom_width, char zoom_height, int rot);
void LCD_Puts_Shadow (char *str, int x, int y,  int t_color, char zoom_width, char zoom_height, int rot);
void LCD_Putsf_Shadow ( char *str, int x, int y,  int t_color, char zoom_width, char zoom_height, int rot);
void LCD_FillScreen (unsigned int color);
void LCD_Write_Image_RLE_ColorBar( const unsigned char *data,int size,const unsigned char *ColorBar,char x1, char x2, char y1, char y2);
void LCD_Output_image (char x, char y, char width, char height,  char *img, int rot);
void Send_Image (char x, char y, char width, char height,  char *img, int rot);
#ifdef _GEOMETRICAL
void LCD_DrawLine (char x1, char y1, char x2, char y2, int color);
void LCD_DrawRect (char x1, char y1, char width, char height, char size, int color);
void LCD_FillRect (char x1, char y1, char width, char height, int color);
void LCD_DrawCircle (char xcenter, char ycenter, char rad, int color);
void LCD_FillCircle (char xcenter, char ycenter, char rad, int color);
void LCD_DrawTriangle (char x1, char y1, char x2, char y2, char x3, char y3, int color);
void LCD_FillTriangle (char x1, char y1, char x2, char y2, char x3, char y3, int color);
void LCD_FillTriangleA (char x1, char y1, char x2, char y2, char x3, char y3, int color);
#endif 




    
