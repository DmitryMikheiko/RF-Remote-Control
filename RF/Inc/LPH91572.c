#include "LPH91572.h"


//===============================================================
//                        �������������
//===============================================================
void LCD_init(void)
{
 LCD_RESET_0
 delay_ms(100);
 LCD_RESET_1
 delay_ms(100);
 Send_to_lcd(CMD, 0x01); //����������� �����
 Send_to_lcd(CMD, 0x36); //Memory Access Control (����������� ���������� ������� ������� (������): 0bVHRXXXXX, V - ���������� �� ��������� (0 - ������-����, 1 - �����-�����), 
                          //H - ���������� �� ����������� (0 - �����-�������, 1 - ������-������), R - �������� ������� ������ � ������� (��� ���� ���������� �������� ������-����, �����-�������))
 #ifdef ROT_90
 Send_to_lcd(DAT, 0x20);
 #else   
 Send_to_lcd(DAT, 0x00); 
 #endif
 Send_to_lcd(CMD, 0x11); //����� �� ������� ������
 delay_ms(20);
 Send_to_lcd(CMD, 0x3a); //��������� �������� �������
 #ifdef _8_BIT_COLOR
 Send_to_lcd(DAT, 0x02); //���� �� ������� 256 ������  0x2
 #else
 Send_to_lcd(DAT, 0x05); //��� ����� �� ������� 65536 ������
 #endif
 delay_ms(20);
 Send_to_lcd(CMD, 0x29); //��������� �������
}
void LCD_off(void)
{	
	LCD_FillScreen(BLACK);	
}
//===============================================================
//������� ������ �������/������ � LCD (RS==0 - �������, RS==1 - ������)
//===============================================================
void Send_to_lcd (unsigned char RS, unsigned char data)
{
 

 if ((RS_old != RS) || (!RS_old && !RS)) 
 { 
	 while((SPI2->SR &SPI_SR_BSY)>0); 
  LCD_CS_1  
	 if(RS) LCD_RS_1
		 else LCD_RS_0   
  LCD_CS_0    
 }

 LcdWD(data);
 /*
 if ((data & 128) )  LCD_DATA_1 
 else LCD_DATA_0
 LCD_CLK_1
 LCD_CLK_0
 //LCD_DATA = 0;
 if ((data & 64) )  LCD_DATA_1 
 else LCD_DATA_0;
 LCD_CLK_1
 LCD_CLK_0
 //LCD_DATA = 0;
 if ((data & 32) )  LCD_DATA_1 
 else LCD_DATA_0
 LCD_CLK_1
 LCD_CLK_0
// LCD_DATA = 0;
 if ((data & 16))  LCD_DATA_1 
 else LCD_DATA_0
 LCD_CLK_1
 LCD_CLK_0
// LCD_DATA = 0;
 if ((data & 8))  LCD_DATA_1 
 else LCD_DATA_0
 LCD_CLK_1
 LCD_CLK_0
 //LCD_DATA = 0;
 if ((data & 4) )  LCD_DATA_1  
 else LCD_DATA_0
 LCD_CLK_1
 LCD_CLK_0
// LCD_DATA = 0;
 if ((data & 2))  LCD_DATA_1   
 else LCD_DATA_0
 LCD_CLK_1
 LCD_CLK_0
// LCD_DATA = 0;
 if ((data & 1) )  LCD_DATA_1  
 else LCD_DATA_0
 LCD_CLK_1
 LCD_CLK_0
 */
 RS_old=RS;  
}

//===============================================================
//              ������� ������������� ������� ������
//===============================================================
void SetArea(char x1, char x2, char y1, char y2)
{
 /*x1=131-x2;
 x2=131-x1; 
 y1=175-y2;
 y2=175-y1; */
 Send_to_lcd( CMD, 0x2A );  //������ ������� �� X
 Send_to_lcd( DAT, x1 );    //���������
 Send_to_lcd( DAT, x2 );    //��������

 Send_to_lcd( CMD, 0x2B );  //������ ������� �� Y
 Send_to_lcd( DAT, y1 );    //��������� 
 Send_to_lcd( DAT, y2 );    //��������   

 Send_to_lcd( CMD, 0x2C );  //���������� ������� �� ������ ������ � ������ � �������� �������� ������
}                 
//===============================================================
void LCD_Write_Image_RLE_ColorBar(const unsigned char *data,int size,const unsigned char *ColorBar,char x1, char x2, char y1, char y2)
{
  unsigned char colorL,colorH; 
  int i,c,index; 
  SetArea(x1,x2,y1,y2);  
  LCD_RS_1   

  for(i=0;i<size;i++)
  { 
    index=data[i++]*2;
    colorH=ColorBar[index];  
    colorL=ColorBar[index+1];   
    for(c=0;c<data[i];c++)
    {
      Send_to_lcd( DAT, colorH );
      Send_to_lcd( DAT, colorL );  
    }
  }
}
//===============================================================
//                          ������ ����� 
//=============================================================== 
void Put_Pixel (char x, char y, unsigned int color) 
{
 SetArea( y, y, x, x ); 
 LCD_RS_1    
 
 #ifdef _8_BIT_COLOR    //(8-�� ������� �������� ������� (256 ������))
 Send_to_lcd( DAT, color ); //������ - ����� ���� ������� 
 #else                    //(16-�� ������� �������� ������� (65536 ������))
 Send_to_lcd( DAT, (color >> 8) ); 
 Send_to_lcd( DAT, color );
 #endif
}  
                   
//===============================================================
//           ������� ���������� ������� �� �������
//===============================================================

void Send_Symbol (unsigned char symbol, char x, char y, int t_color, int b_color, char zoom_width, char zoom_height, int rot) 
{
 unsigned char temp_symbol, a, b, zw, zh, mask; 
  
 if (symbol>127) symbol-=64;    //������� ������������� ����� ������� ASCII
 for ( a = 0; a < 5; a++) //��������� 5 ����, ������������ ������
 {
  temp_symbol = font_5x8[symbol-32][a];
  zw = 0; 
  while(zw != zoom_width) //����� ����� ����������� zw ��� 
  {    
   mask=0x01;  
   switch(rot)
   {
    case 0: case 180: SetArea( x+zw, x+zw, y, y+(zoom_height*8)-1 ); break;
    case 90: case 270: SetArea( x, x+(zoom_height*8)-1, y+zw, y+zw ); break;                  
   } 
   LCD_RS_1 //���������� ������          
   for ( b = 0; b < 8; b++ ) //���� ����������� 8 ��� �����
   {         
    zh = zoom_height; //� zoom_height ��� ���������� ������ �������
    while(zh != 0) //����� ������� ����������� z ���
    {
     if (temp_symbol&mask) 
     {
      #ifdef _8_BIT_COLOR    //(8-�� ������� �������� ������� (256 ������))
      Send_to_lcd( DAT, t_color ); //������ - ����� ���� ������� 
      #else                    //(16-�� ������� �������� ������� (65536 ������))
      Send_to_lcd( DAT, (t_color >> 8) ); Send_to_lcd( DAT, t_color );
      #endif
     }
     else 
     {
      #ifdef _8_BIT_COLOR    //(8-�� ������� �������� ������� (256 ������))
      Send_to_lcd( DAT, b_color ); //������ - ����� ���� ������� 
      #else                    //(16-�� ������� �������� ������� (65536 ������))
      Send_to_lcd( DAT, (b_color >> 8) ); Send_to_lcd( DAT, b_color );
      #endif
     }
     zh--;
    }
    mask<<=1; //������ ���������� mask �� 1 ��� �����;    
   }
   zw++;
  }
  switch(rot)
  {
   case 0: case 180: x=x+zoom_width;  break; //�������� ����� ���������� ������� �� ��� x ��� ������ ���������� �����
   case 90: case 270: y=y+zoom_width; break; //�������� ����� ���������� ������� �� ��� y ��� ������ ���������� �����            
  }                  
 }
}  

//===============================================================
// ������� ������ ������ ������� ASCII-���� (�� ����� Symbols.h)
//===============================================================
#ifdef LS020_Comp
void LCD_Putchar(char symbol, char x, char y, int t_color, int b_color)
{ 
	uint8_t i;
	uint16_t ic,char_code;
	SetArea(x,x+7,y,y+15);
	for (i=0;i<16;i++) 
	{ char_code=ascii_8x16[((symbol-0x20)*16)+i];
		for (ic=0;ic<8;ic++)
		 {
			 if ( ((char_code >> (7-ic)) & 0x01) == 0x01)
          { 
						#ifdef _8_BIT_COLOR    //(8-�� ������� �������� ������� (256 ������))
            Send_to_lcd( DAT, t_color ); //������ - ����� ���� ������� 
            #else                    //(16-�� ������� �������� ������� (65536 ������))
            Send_to_lcd( DAT, (t_color >> 8) ); 
						Send_to_lcd( DAT, t_color );
            #endif
					}
          else
          {
            #ifdef _8_BIT_COLOR    //(8-�� ������� �������� ������� (256 ������))
            Send_to_lcd( DAT, b_color ); //������ - ����� ���� ������� 
            #else                    //(16-�� ������� �������� ������� (65536 ������))
            Send_to_lcd( DAT, (b_color >> 8) ); 
						Send_to_lcd( DAT, b_color );
            #endif
          }					 
		 }		
	}
}
void LCD_Puts(char *str,int length, int x, int y,  int t_color, int b_color)
{ 
	uint16_t i;  
  for(i=0;i<length;i++)
  {
    LCD_Putchar(*str++,(x+i*8),y,t_color,b_color);
	
  }	
}
#else
void LCD_Putchar(char symbol, char x, char y, int t_color, int b_color, char zoom_width, char zoom_height, int rot)
{
 unsigned char m;
 if(zoom_width == 0)   zoom_width = 1;
 if(zoom_height == 0)  zoom_height = 1;
 switch (rot)
 {
  case 0:  //��������� ����� ���� � � � - ����� ������� ���� �������   
  Send_Symbol( symbol, x, y, t_color, b_color, zoom_width, zoom_height, rot);
  break;
  //================================
  case 90:
  m=y; y=x; x=m;
  Send_to_lcd(CMD, 0x36); 
  Send_to_lcd(DAT, 0x40); //��������� ����� ���� � � � - ������ ������� ���� ������� 
  Send_Symbol( symbol, x, y, t_color, b_color, zoom_width, zoom_height, rot);
  Send_to_lcd(CMD, 0x36); 
  Send_to_lcd(DAT, 0x00);
  break;
  //================================
  case 180:       
  Send_to_lcd(CMD, 0x36); 
  Send_to_lcd(DAT, 0xC0); //��������� ����� ���� � � � - ������ ������ ���� ������� 
  Send_Symbol( symbol, x, y, t_color, b_color, zoom_width, zoom_height, rot);
  Send_to_lcd(CMD, 0x36); 
  Send_to_lcd(DAT, 0x00);
  break;
  //================================
  case 270:
  m=y; y=x; x=m;
  Send_to_lcd(CMD, 0x36); 
  Send_to_lcd(DAT, 0x80); //��������� ����� ���� � � � - ����� ������ ���� �������  
  Send_Symbol( symbol, x, y, t_color, b_color, zoom_width, zoom_height, rot);
  Send_to_lcd(CMD, 0x36); 
  Send_to_lcd(DAT, 0x00);
  break;
  //================================
  default:
  Send_to_lcd(CMD, 0x36); 
  Send_to_lcd(DAT, 0x00); //��������� ����� ���� � � � - ����� ������� ���� �������  
  Send_Symbol( symbol, x, y, t_color, b_color, zoom_width, zoom_height, rot);
  Send_to_lcd(CMD, 0x36); 
  Send_to_lcd(DAT, 0x00);
  //=================================    
 };  
}

//===============================================================
//          ������� ������ ������, ������������� � ram 
//===============================================================
void LCD_Puts(char *str,int length int x, int y,  int t_color, int b_color, char zoom_width, char zoom_height, int rot)
{
 unsigned char i=0;
              
 if(zoom_width == 0)   zoom_width = 1;
 if(zoom_height == 0)  zoom_height = 1;
   
 for(i=0;i<length;i++) //x � y - ����� ������� ��������� �������; � ����������� ���������� i ����� ������ ���������� ������� ��������� �� i*6 (��� ������������ ������� �������)  
 {      
  LCD_Putchar(str[i], x+(i*6*zoom_width), y, t_color, b_color, zoom_width, zoom_height, rot);
  i++;
 }  
}


//===============================================================
//     ������� ���������� ������� �� ������� ��� ����� ����
//===============================================================
#endif
void Send_Symbol_Shadow (unsigned char symbol, char x, char y, int t_color, char zoom_width, char zoom_height, int rot) 
{
 unsigned char temp_symbol, a, b, zw, zh, mask; 
 char m, n;
 m=x; 
 n=y;
 if (symbol>127) symbol-=64;    //������� ������������� ����� ������� ASCII
 for ( a = 0; a < 5; a++) //��������� 5 ����, ������������ ������
 {  
  temp_symbol = font_5x8[symbol-32][a];
  zw = 0; 
  while(zw != zoom_width) //����� ����� ����������� zw ��� 
  {    
   switch(rot)
   {
    case 0: case 180: n=y; break;
    case 90: case 270: m=x; break;
   } 
   mask=0x01;     
   for ( b = 0; b < 8; b++ ) //���� ����������� 8 ��� �����
   {         
    zh = 0; //� zoom_height ��� ���������� ������ �������
    while(zh != zoom_height) //����� ������� ����������� z ���
    {
     switch(rot)
     {
      case 0: case 180:  
      if (temp_symbol&mask) 
      {
       Put_Pixel (m+zw, n+zh, t_color);
      }
      break; 
      case 90: case 270: 
      if (temp_symbol&mask) 
      {
       Put_Pixel (m+zh, n+zw, t_color);
      }
      break; //�������� ����� ���������� ������� �� ��� y ��� ������ ���������� �����            
     }            
     zh++;
    }
    mask<<=1; //������ ���������� mask �� 1 ��� �����;
    switch(rot)
    {
     case 0: case 180: n=n+zoom_height; break;
     case 90: case 270: m=m+zoom_height; break;
    }          
   }
   zw++;   
  }
  switch(rot)
  {
   case 0: case 180: m=m+zoom_width; break;
   case 90: case 270: n=n+zoom_width; break;
  }           
 }
} 

//===============================================================
// ������� ������ ������ ������� ASCII-���� ��� ����� ����
//===============================================================
void LCD_Putchar_Shadow (char symbol, char x, char y, int t_color, char zoom_width, char zoom_height, int rot)
{
 unsigned char m;
 if(zoom_width == 0)   zoom_width = 1;
 if(zoom_height == 0)  zoom_height = 1;
 switch (rot)
 {
  case 0:  //��������� ����� ���� � � � - ����� ������� ���� �������   
  Send_Symbol_Shadow( symbol, x, y, t_color, zoom_width, zoom_height, rot);
  break;
  //================================
  case 90:
  m=y; y=x; x=m;
  Send_to_lcd(CMD, 0x36); 
  Send_to_lcd(DAT, 0x40); //��������� ����� ���� � � � - ������ ������� ���� ������� 
  Send_Symbol_Shadow( symbol, x, y, t_color, zoom_width, zoom_height, rot);
  Send_to_lcd(CMD, 0x36); 
  Send_to_lcd(DAT, 0x00);
  break;
  //================================
  case 180:       
  Send_to_lcd(CMD, 0x36); 
  Send_to_lcd(DAT, 0xC0); //��������� ����� ���� � � � - ������ ������ ���� ������� 
  Send_Symbol_Shadow( symbol, x, y, t_color, zoom_width, zoom_height, rot);
  Send_to_lcd(CMD, 0x36); 
  Send_to_lcd(DAT, 0x00);
  break;
  //================================
  case 270:
  m=y; y=x; x=m;
  Send_to_lcd(CMD, 0x36); 
  Send_to_lcd(DAT, 0x80); //��������� ����� ���� � � � - ����� ������ ���� �������  
  Send_Symbol_Shadow( symbol, x, y, t_color, zoom_width, zoom_height, rot);
  Send_to_lcd(CMD, 0x36); 
  Send_to_lcd(DAT, 0x00);
  break;
  //================================
  default:
  Send_to_lcd(CMD, 0x36); 
  Send_to_lcd(DAT, 0x00); //��������� ����� ���� � � � - ����� ������� ���� �������  
  Send_Symbol_Shadow( symbol, x, y, t_color, zoom_width, zoom_height, rot);
  Send_to_lcd(CMD, 0x36); 
  Send_to_lcd(DAT, 0x00);
  //=================================    
 };  
}
void LCD_Put24Digit(uint16_t xs,uint16_t ys,const uint8_t *picture,uint16_t size,int t_color,int b_color)
{
	  uint32_t i;
		uint8_t count;
	  uint16_t color;
	  SetArea(xs,xs+23,ys,ys+39);
	  LCD_RS_0
	for(i=0;i<size;i+=2)
		{
       if(picture[i]==0)color=t_color;
			 else color=b_color;
			 count=picture[i+1];
      while(count>0)
			{
				count--;
			      #ifdef _8_BIT_COLOR    //(8-�� ������� �������� ������� (256 ������))
            Send_to_lcd( DAT, color ); //������ - ����� ���� ������� 
            #else                    //(16-�� ������� �������� ������� (65536 ������))
            Send_to_lcd( DAT, (color >> 8) ); 
						Send_to_lcd( DAT, color );				   
            #endif
			}
			 	
		}
	
	LCD_RS_1
	SetArea(0,175,0,131);
}
void LCD_Put24(char symbol,uint8_t x,uint8_t y,int t_color,int b_color)
{
	u16 size;
	const uint8_t *dig;
	switch(symbol)
	{
		case '0': dig=S0;size=sizeof(S0);break;
		case '1': dig=S1;size=sizeof(S1);break;
		case '2': dig=S2;size=sizeof(S2);break;
		case '3': dig=S3;size=sizeof(S3);break;
		case '4': dig=S4;size=sizeof(S4);break;
		case '5': dig=S5;size=sizeof(S5);break;
		case '6': dig=S6;size=sizeof(S6);break;
		case '7': dig=S7;size=sizeof(S7);break;
		case '8': dig=S8;size=sizeof(S8);break;
		case '9': dig=S9;size=sizeof(S9);break;
		case ':': dig=S_Dot;size=sizeof(S_Dot);break;
		default: dig=SS;size=sizeof(SS);break;
	}
	LCD_Put24Digit(x,y,dig,size,t_color,b_color);
}
//===============================================================
//   ������� ������ ������, ������������� � ram ��� ����� ���� 
//===============================================================
void LCD_Puts_Shadow(char *str, int x, int y,  int t_color, char zoom_width, char zoom_height, int rot)
{
 unsigned char i=0;
                    
 if(zoom_width == 0)   zoom_width = 1;
 if(zoom_height == 0)  zoom_height = 1;
   
 while (str[i]) //x � y - ����� ������� ��������� �������; � ����������� ���������� i ����� ������ ���������� ������� ��������� �� i*6 (��� ������������ ������� �������)  
 {      
  LCD_Putchar_Shadow(str[i], x+(i*6*zoom_width), y, t_color, zoom_width, zoom_height, rot);
  i++;
 }  
}

//===============================================================
// ������� ������ ������, ������������� �� flash ��� ����� ����
//===============================================================
void LCD_Putsf_Shadow(char *str, int x, int y,  int t_color, char zoom_width, char zoom_height, int rot)
{
 unsigned char i=0;
           
 if(zoom_width == 0)   zoom_width = 1;
 if(zoom_height == 0)  zoom_height = 1;
   
 while (str[i])   
 {      
  LCD_Putchar_Shadow(str[i], x+(i*6*zoom_width), y, t_color, zoom_width, zoom_height, rot);
  i++;
 }  
} 
   
//===============================================================
//                  ������� ������ ������ 
//===============================================================
void LCD_FillScreen (unsigned int color)
{ 
 unsigned int x; 
 SetArea( 0, 131, 0, 175 );   //������� ����� ������ 
	while((SPI2->SR &SPI_SR_BSY)>0); 
 LCD_RS_1   
 LCD_CS_0
 //������ - ����� ���� �������
 for (x = 0; x < 23232; x++)  // 23232 - ��� 132 * 176
 {   
  #ifdef _8_BIT_COLOR    //(8-�� ������� �������� ������� (256 ������))
 // Send_to_lcd( DAT, color ); //������ - ����� ���� ������� 
	  LcdWD(color);
  #else            //(16-�� ������� �������� ������� (65536 ������))
 // Send_to_lcd( DAT, (color >> 8) ); Send_to_lcd( DAT, color );
	 LcdWD(color>>8);
	 LcdWD(color);
  #endif
 }  
 while((SPI2->SR &SPI_SR_BSY)>0);
 LCD_CS_1 
 LCD_RS_0
} 

//===============================================================
//                 ������� ������ �����������
//===============================================================
void LCD_Output_image (char x, char y, char width, char height,char *img, int rot)
{ 
 unsigned char m;
 switch (rot)
 {
  case 0:      
  Send_Image (x, y, width, height, img, rot);
  break;
  //================================
  case 90:
  m=y; y=x; x=m;
  Send_to_lcd(CMD, 0x36); 
  Send_to_lcd(DAT, 0x40); //��������� ����� ���� � � � - ������ ������� ���� ������� 
  Send_Image (x, y, width, height, img, rot);
  Send_to_lcd(CMD, 0x36); 
  Send_to_lcd(DAT, 0x00);
  break;
  //================================
  case 180:       
  Send_to_lcd(CMD, 0x36); 
  Send_to_lcd(DAT, 0xC0); //��������� ����� ���� � � � - ������ ������ ���� ������� 
  Send_Image (x, y, width, height, img, rot);
  Send_to_lcd(CMD, 0x36); 
  Send_to_lcd(DAT, 0x00);
  break;
  //================================
  case 270:
  m=y; y=x; x=m;
  Send_to_lcd(CMD, 0x36); 
  Send_to_lcd(DAT, 0x80); //��������� ����� ���� � � � - ����� ������ ���� �������  
  Send_Image (x, y, width, height, img, rot);
  Send_to_lcd(CMD, 0x36); 
  Send_to_lcd(DAT, 0x00);
  break;
  //================================
  default:
  Send_to_lcd(CMD, 0x36); 
  Send_to_lcd(DAT, 0x00); //��������� ����� ���� � � � - ����� ������� ���� �������  
  Send_Image (x, y, width, height, img, rot);
  Send_to_lcd(CMD, 0x36); 
  Send_to_lcd(DAT, 0x00);
  //=================================    
 };
}   

//===============================================================
//          ������� ��� ����������� ������ �����������   
//===============================================================
//����� �������� � Image2Lcd � NokiaImageCreator ������ ����������� �����-������� ������-����.
//x, y - ������ ������� ������ �����������; width � height - ������ � ������ �����������   
void Send_Image (char x, char y, char width, char height, char *img, int rot)  
{  
 char x1, y1; 
 
 switch (rot)
 {
  case 0: case 180:  
  for(y1=y; y1<(y+height); y1++)
  {
   SetArea( x, x+(width-1), y1, y1 );   
   for(x1=x; x1<x+width; x1++)
   {   
    #ifdef _8_BIT_COLOR    //(8-�� ������� �������� ������� (256 ������))
    Send_to_lcd( DAT, *img++ ); //������ - ����� ���� ������� 
    #else            //(16-�� ������� �������� ������� (65536 ������))
    Send_to_lcd( DAT, *img++ ); Send_to_lcd( DAT, *img++ );  
    #endif
   }
  }
  break;
 
  case 90: case 270:
  for(x1=x; x1<x+height; x1++)
  {
   SetArea( x1, x1, y, y+(width-1) );       
   for(y1=y; y1<y+width; y1++)
   {
    #ifdef _8_BIT_COLOR    //(8-�� ������� �������� ������� (256 ������))
    Send_to_lcd( DAT, *img++ ); //������ - ����� ���� ������� 
    #else            //(16-�� ������� �������� ������� (65536 ������))
    Send_to_lcd( DAT, *img++ ); Send_to_lcd( DAT, *img++ );      
    #endif  
   } 
  }
  break; 
 }; 
} 

#ifdef _GEOMETRICAL
//===============================================================
//                      ���������� �����
//===============================================================
void LCD_DrawLine (char x1, char y1, char x2, char y2, int color)
{ 
 short  x, y, d, dx, dy, i, i1, i2, kx, ky;
 signed char flag;

 dx = x2 - x1;
 dy = y2 - y1;
 if (dx == 0 && dy == 0) Put_Pixel(x1, y1, color);  //�����
 else      //�����
 {
  kx = 1;
  ky = 1;
  if( dx < 0 )
  { 
   dx = -dx; 
   kx = -1; 
  }
  else
  if(dx == 0) kx = 0;
  if(dy < 0)
  { 
   dy = -dy; 
   ky = -1; 
  }
  if(dx < dy)
  { 
   flag = 0; 
   d = dx; 
   dx = dy; 
   dy = d; 
  }
  else flag = 1;
  i1 = dy + dy; 
  d = i1 - dx; 
  i2 = d - dx;
  x = x1; 
  y = y1;

  for(i=0; i < dx; i++)
  {
   Put_Pixel(x, y, color);
   if(flag) x += kx;
   else y += ky;
   if( d < 0 ) d += i1;
   else
   {
    d += i2;
    if(flag) y += ky;
    else x += kx;
   }
  }
  Put_Pixel(x, y, color);
 }
}   
  
//===============================================================
//                        ���������� �����                
//===============================================================
void LCD_DrawRect (char x1, char y1, char width, char height, char size, int color)
{
 unsigned int i;
 char x2=x1+(width-1), y2=y1+(height-1); //�������� ������� ����� �� ���� � � �
 for( i=1; i<=size; i++)   // size - ������� �����
 {
  LCD_DrawLine(x1, y1, x1, y2, color);
  LCD_DrawLine(x2, y1, x2, y2, color);
  LCD_DrawLine(x1, y1, x2, y1, color);
  LCD_DrawLine(x1, y2, x2, y2, color);
  x1++; // ���������� ������� �����, ���� ��� ������
  y1++;
  x2--;
  y2--;
 }
}

//===============================================================
//              ��������� ������������� ������ COLOR
//===============================================================
void LCD_FillRect (char x1, char y1, char width, char height, int color)
{
 unsigned int x, y;
       
// SetArea(y1,y1+height-1,175-(x1+width-1),175-x1 );
	SetArea(x1,x1+width-1,y1,y1+height-1);
	while((SPI2->SR &SPI_SR_BSY)>0); 
 LCD_RS_1
 
 y = width * height;            //���������� �������� � ��������������
 for (x = 0; x < y; x++) 
 {   
  #ifdef _8_BIT_COLOR    //(8-�� ������� �������� ������� (256 ������))
  Send_to_lcd( DAT, color ); //������ - ����� ���� ������� 
  #else            //(16-�� ������� �������� ������� (65536 ������))
  Send_to_lcd( DAT, (color >> 8) ); Send_to_lcd( DAT, color );
  #endif
 }   
}  
 
//===============================================================
//                  ���������� ����������
//===============================================================
void LCD_DrawCircle (char xcenter, char ycenter, char rad, int color)
{
 signed char tswitch, x1=0, y1; 
 char d;

 d = ycenter - xcenter;
 y1 = rad;
 tswitch = 3 - 2 * rad;
 while (x1 <= y1) 
 {
  Put_Pixel(xcenter + x1, ycenter + y1, color); 
  Put_Pixel(xcenter + x1, ycenter - y1, color);
  Put_Pixel(xcenter - x1, ycenter + y1, color);   
  Put_Pixel(xcenter - x1, ycenter - y1, color);
  Put_Pixel(ycenter + y1 - d, ycenter + x1, color); 
  Put_Pixel(ycenter + y1 - d, ycenter - x1, color);
  Put_Pixel(ycenter - y1 - d, ycenter + x1, color);
  Put_Pixel(ycenter - y1 - d, ycenter - x1, color);

  if (tswitch < 0) tswitch += (4 * x1 + 6);
  else 
  {
   tswitch += (4 * (x1 - y1) + 10);
   y1--;
  }
  x1++;
 }
}    

//===============================================================
//                 ��������� ���� ������ COLOR
//===============================================================
void LCD_FillCircle (char xcenter, char ycenter, char rad, int color)
{
 signed int x1=0, y1, tswitch; 
 y1 = rad;
 tswitch = 1 - rad;

 do
 {
  LCD_DrawLine(xcenter-x1, ycenter+y1, xcenter+x1, ycenter+y1, color);
  LCD_DrawLine(xcenter-x1, ycenter-y1, xcenter+x1, ycenter-y1, color);
  LCD_DrawLine(xcenter-y1, ycenter+x1, xcenter+y1, ycenter+x1, color);
  LCD_DrawLine(xcenter-y1, ycenter-x1, xcenter+y1, ycenter-x1, color);

  if(tswitch < 0)
  tswitch+= 3 + 2*x1++;
  else
  tswitch+= 5 + 2*(x1++ - y1--);
 } while(x1 <= y1);
}

//===============================================================
//                     ���������� �����������
//===============================================================
void LCD_DrawTriangle(char x1, char y1, char x2, char y2, char x3, char y3, int color)
{
 LCD_DrawLine(x1, y1, x2, y2, color);
 LCD_DrawLine(x3, y3, x1, y1, color);
 LCD_DrawLine(x3, y3, x2, y2, color);  
}

//===============================================================
//              ��������� ����������� ������ COLOR
//===============================================================
void LCD_FillTriangle(char x1, char y1, char x2, char y2, char x3, char y3, int color)
{
 LCD_FillTriangleA(x1, y1, x2, y2, x3, y3, color);
 LCD_FillTriangleA(x3, y3, x1, y1, x2, y2, color);
 LCD_FillTriangleA(x3, y3, x2, y2, x1, y1, color);
}    

//===============================================================
void LCD_FillTriangleA(char x1, char y1, char x2, char y2, char x3, char y3, int color)
{
 signed long x, y, addx, dx, dy;
 signed long P;
 int i;
 long a1,a2,b1,b2;
 if(y1>y2)  {b1=y2; b2=y1; a1=x2; a2=x1;}
 else       {b1=y1; b2=y2; a1=x1; a2=x2;}
 dx = a2 -a1;
 dy = b2 - b1;
 if(dx<0)dx=-dx;
 if(dy<0)dy=-dy;
 x = a1;
 y = b1;
   
 if(a1 > a2)    addx = -1;
 else           addx = 1;
   
 if(dx >= dy)
 {
  P = 2*dy - dx;
  for(i=0; i<=dx; ++i)
  {
   LCD_DrawLine((int)x, (int)y, x3, y3, color);
   if(P < 0)
   {
    P += 2*dy;
    x += addx;
   }
   else
   {
    P += 2*dy - 2*dx;
    x += addx;
    y ++;
   }
  }
 }
 else
 {
  P = 2*dx - dy;
  for(i=0; i<=dy; ++i)
  {
   LCD_DrawLine((int)x, (int)y, x3, y3, color);
   if(P < 0)
   {
    P += 2*dx;
    y ++;
   }
   else
   {
    P += 2*dx - 2*dy;
    x += addx;
    y ++;
   }
  }
 }
}

//===============================================================
#endif
