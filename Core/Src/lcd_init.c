#include "lcd_init.h"
#include "main.h"
#include "spi.h"

void LCD_GPIO_Init(void)
{

}

//void delay(int t)
//{
//	while(t--);
//}

/******************************************************************************
      函数说明：LCD串行数据写入函数
      入口数据：dat  要写入的串行数据
      返回值：  无
******************************************************************************/
void LCD_Writ_Bus(uint8_t dat) 
{	
	LCD_CS_Clr();
	SPI1_ReadWriteByte(dat);
	LCD_CS_Set();
}

/******************************************************************************
      函数说明：LCD写入数据
      入口数据：dat 写入的数据
      返回值：  无
******************************************************************************/
void LCD_WR_DATA8(uint8_t dat)
{
	LCD_Writ_Bus(dat);
}


/******************************************************************************
      函数说明：LCD写入数据
      入口数据：dat 写入的数据
      返回值：  无
******************************************************************************/
void LCD_WR_DATA(uint16_t dat)
{
	LCD_Writ_Bus(dat>>8);
	LCD_Writ_Bus(dat);
}


/******************************************************************************
      函数说明：LCD写入命令
      入口数据：dat 写入的命令
      返回值：  无
******************************************************************************/
void LCD_WR_REG(uint8_t dat)
{
	LCD_DC_Clr();//写命令
	LCD_Writ_Bus(dat);
	LCD_DC_Set();//写数据
}


/******************************************************************************
      函数说明：设置起始和结束地址
      入口数据：x1,x2 设置列的起始和结束地址
                y1,y2 设置行的起始和结束地址
      返回值：  无
******************************************************************************/
void LCD_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c);//写储存器
}

void LCD_Init(void)
{
	//SPI1_Init();
	//LCD_GPIO_Init();//初始化GPIO
	
	LCD_RES_Clr();//复位
	HAL_Delay(100);
	LCD_RES_Set();
	HAL_Delay(100);
	
	LCD_BLK_Set();//打开背光
  HAL_Delay(100);
	
	//************* Start Initial Sequence **********//
	LCD_WR_REG(0x11); //Sleep out 
	HAL_Delay(120);              //Delay 120ms 
	//************* Start Initial Sequence **********// 
	LCD_WR_REG(0Xf0);
	LCD_WR_DATA8(0xc3);
	LCD_WR_REG(0Xf0);
	LCD_WR_DATA8(0x96);
	LCD_WR_REG(0x36);    // Memory Access Control 
	if(USE_HORIZONTAL==0)LCD_WR_DATA8(0x48);
	else if(USE_HORIZONTAL==1)LCD_WR_DATA8(0x88);
	else if(USE_HORIZONTAL==2)LCD_WR_DATA8(0x28);
	else LCD_WR_DATA8(0xE8);
	LCD_WR_REG(0X3a);
	LCD_WR_DATA8(0x05);
	LCD_WR_REG(0Xe6);
	LCD_WR_DATA8(0x0f);
	LCD_WR_DATA8(0xf2);
	LCD_WR_DATA8(0x3f);
	LCD_WR_DATA8(0x4f);
	LCD_WR_DATA8(0x4f);
	LCD_WR_DATA8(0x28);
	LCD_WR_DATA8(0x0e);
	LCD_WR_DATA8(0x00);
	LCD_WR_REG(0Xc5);
	LCD_WR_DATA8(0x2a);
	LCD_WR_REG(0Xe0);
	LCD_WR_DATA8(0xf0);
	LCD_WR_DATA8(0x03);
	LCD_WR_DATA8(0x0a);
	LCD_WR_DATA8(0x11);
	LCD_WR_DATA8(0x14);
	LCD_WR_DATA8(0x1c);
	LCD_WR_DATA8(0x3b);
	LCD_WR_DATA8(0x55);
	LCD_WR_DATA8(0x4a);
	LCD_WR_DATA8(0x0a);
	LCD_WR_DATA8(0x13);
	LCD_WR_DATA8(0x14);
	LCD_WR_DATA8(0x1c);
	LCD_WR_DATA8(0x1f);
	LCD_WR_REG(0Xe1);
	LCD_WR_DATA8(0xf0);
	LCD_WR_DATA8(0x03);
	LCD_WR_DATA8(0x0a);
	LCD_WR_DATA8(0x0c);
	LCD_WR_DATA8(0x0c);
	LCD_WR_DATA8(0x09);
	LCD_WR_DATA8(0x36);
	LCD_WR_DATA8(0x54);
	LCD_WR_DATA8(0x49);
	LCD_WR_DATA8(0x0f);
	LCD_WR_DATA8(0x1b);
	LCD_WR_DATA8(0x18);
	LCD_WR_DATA8(0x1b);
	LCD_WR_DATA8(0x1f);
	LCD_WR_REG(0Xf0);
	LCD_WR_DATA8(0x3c);
	LCD_WR_REG(0Xf0);
	LCD_WR_DATA8(0x69);
	LCD_WR_REG(0X29);
	
	LCD_Fill(0,0,LCD_W,LCD_H,BLACK);
} 









