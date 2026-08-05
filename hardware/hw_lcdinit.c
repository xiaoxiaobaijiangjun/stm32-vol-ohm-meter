#include "hw_lcdinit.h"
#include "spi.h"

#define ENABLE_1_8_LCD 1

#ifdef ENABLE_0_96_LCD

void lcd_0_96_write_data8(lcd_interface *lcd_t,uint8_t data)
{
	//select lcd
	HAL_GPIO_WritePin(lcd_t->lcd_p->lcd_cs_gpio,lcd_t->lcd_p->lcd_cs_pin,GPIO_PIN_RESET);

	HAL_SPI_Transmit(lcd_t->lcd_p->hspi,&data,1,HAL_MAX_DELAY);

	//deselect lcd
	HAL_GPIO_WritePin(lcd_t->lcd_p->lcd_cs_gpio,lcd_t->lcd_p->lcd_cs_pin,GPIO_PIN_SET);
}

void lcd_0_96_write_data16(lcd_interface *lcd_t,uint16_t data)
{
	uint8_t sendData[2] = {0};
	sendData[0] = (data>>8);
	sendData[1] = (data);

	//select lcd
	HAL_GPIO_WritePin(lcd_t->lcd_p->lcd_cs_gpio,lcd_t->lcd_p->lcd_cs_pin,GPIO_PIN_RESET);

	HAL_SPI_Transmit(lcd_t->lcd_p->hspi,sendData,2,HAL_MAX_DELAY);

	//deselect lcd
	HAL_GPIO_WritePin(lcd_t->lcd_p->lcd_cs_gpio,lcd_t->lcd_p->lcd_cs_pin,GPIO_PIN_SET);
}

void lcd_0_96_write_reg(lcd_interface *lcd_t,uint8_t reg)
{
	HAL_GPIO_WritePin(lcd_t->lcd_p->lcd_dc_gpio,lcd_t->lcd_p->lcd_dc_pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(lcd_t->lcd_p->lcd_cs_gpio,lcd_t->lcd_p->lcd_cs_pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(lcd_t->lcd_p->hspi,&reg,1,HAL_MAX_DELAY);
	HAL_GPIO_WritePin(lcd_t->lcd_p->lcd_dc_gpio,lcd_t->lcd_p->lcd_dc_pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(lcd_t->lcd_p->lcd_cs_gpio,lcd_t->lcd_p->lcd_cs_pin,GPIO_PIN_SET);
}

void lcd_0_96_address_set(lcd_interface *lcd_t,uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{
    if(lcd_t->lcd_p->lcd_direction == 0 || lcd_t->lcd_p->lcd_direction == 1){
        lcd_0_96_write_reg(lcd_t,0x2a);
	    lcd_0_96_write_data16(lcd_t,x1+24);
	    lcd_0_96_write_data16(lcd_t,x2+24);
	    lcd_0_96_write_reg(lcd_t,0x2b);
	    lcd_0_96_write_data16(lcd_t,y1);
	    lcd_0_96_write_data16(lcd_t,y2);
	    lcd_0_96_write_reg(lcd_t,0x2c);
    }
	if(lcd_t->lcd_p->lcd_direction == 2 || lcd_t->lcd_p->lcd_direction == 3){
        lcd_0_96_write_reg(lcd_t,0x2a);
	    lcd_0_96_write_data16(lcd_t,x1);
	    lcd_0_96_write_data16(lcd_t,x2);
	    lcd_0_96_write_reg(lcd_t,0x2b);
	    lcd_0_96_write_data16(lcd_t,y1+24);
	    lcd_0_96_write_data16(lcd_t,y2+24);
	    lcd_0_96_write_reg(lcd_t,0x2c);
    }
}

void lcd_0_96_enter_sleep(lcd_interface *lcd_t)
{
	lcd_0_96_write_reg(lcd_t,0x28);     	//Display off
	HAL_Delay(10);
	lcd_0_96_write_reg(lcd_t,0x10);     	//Sleep enter
	HAL_Delay(120);
}

void lcd_0_96_exit_sleep(lcd_interface *lcd_t)
{
	lcd_0_96_write_reg(lcd_t,0x11);     	//Sleep out
	HAL_Delay(120);
	lcd_0_96_write_reg(lcd_t,0x29);     	//Display on
	HAL_Delay(10);
	lcd_0_96_write_reg(lcd_t,0x2C);
}

void lcd_0_96_init(lcd_interface *lcd_t)
{
	HAL_GPIO_WritePin(lcd_t->lcd_p->lcd_res_gpio, lcd_t->lcd_p->lcd_res_pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(lcd_t->lcd_p->lcd_res_gpio, lcd_t->lcd_p->lcd_res_pin, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(lcd_t->lcd_p->lcd_blk_gpio, lcd_t->lcd_p->lcd_blk_pin, GPIO_PIN_SET);
    HAL_Delay(100);

	//************* Start Initial Sequence **********//
	lcd_0_96_write_reg(lcd_t,0x11); //Sleep out
	HAL_Delay(120);	//Delay 120ms
	//------------------------------------ST7735S Frame Rate-----------------------------------------//
	lcd_0_96_write_reg(lcd_t,0xB1);     	//Normal mode
	lcd_0_96_write_data8(lcd_t,0x05);
    lcd_0_96_write_data8(lcd_t,0x3c);
    lcd_0_96_write_data8(lcd_t,0x3c);

	lcd_0_96_write_reg(lcd_t,0xB2);     	//Idle mode
	lcd_0_96_write_data8(lcd_t,0x05);
    lcd_0_96_write_data8(lcd_t,0x3c);
    lcd_0_96_write_data8(lcd_t,0x3c);

	lcd_0_96_write_reg(lcd_t,0xB3);     	//Partial mode
    lcd_0_96_write_data8(lcd_t,0x05);
    lcd_0_96_write_data8(lcd_t,0x3c);
    lcd_0_96_write_data8(lcd_t,0x3c);
    lcd_0_96_write_data8(lcd_t,0x05);
    lcd_0_96_write_data8(lcd_t,0x3c);
    lcd_0_96_write_data8(lcd_t,0x3c);

	lcd_0_96_write_reg(lcd_t,0xB4);     //Dot inversion
    lcd_0_96_write_data8(lcd_t,0x03);

	lcd_0_96_write_reg(lcd_t,0xC0);     //AVDD GVDD
    lcd_0_96_write_data8(lcd_t,0xAB);
    lcd_0_96_write_data8(lcd_t,0x0B);
    lcd_0_96_write_data8(lcd_t,0x04);

	lcd_0_96_write_reg(lcd_t,0xC1);     //VGH VGL
	lcd_0_96_write_data8(lcd_t,0xC5);   //C0

	lcd_0_96_write_reg(lcd_t,0xC2);     //Normal Mode
    lcd_0_96_write_data8(lcd_t,0x0D);
    lcd_0_96_write_data8(lcd_t,0x00);

	lcd_0_96_write_reg(lcd_t,0xC3);     //Idle
    lcd_0_96_write_data8(lcd_t,0x8D);
    lcd_0_96_write_data8(lcd_t,0x6A);

	lcd_0_96_write_reg(lcd_t,0xC4);     //Partial+Full
    lcd_0_96_write_data8(lcd_t,0x8D);
    lcd_0_96_write_data8(lcd_t,0xEE);

	lcd_0_96_write_reg(lcd_t,0xC5);     //VCOM
    lcd_0_96_write_data8(lcd_t,0x0F);

	lcd_0_96_write_reg(lcd_t,0xE0);     //positive gamma
    lcd_0_96_write_data8(lcd_t,0x07);
    lcd_0_96_write_data8(lcd_t,0x0E);
    lcd_0_96_write_data8(lcd_t,0x08);
    lcd_0_96_write_data8(lcd_t,0x07);
    lcd_0_96_write_data8(lcd_t,0x10);
    lcd_0_96_write_data8(lcd_t,0x07);
    lcd_0_96_write_data8(lcd_t,0x02);
    lcd_0_96_write_data8(lcd_t,0x07);
    lcd_0_96_write_data8(lcd_t,0x09);
    lcd_0_96_write_data8(lcd_t,0x0F);
    lcd_0_96_write_data8(lcd_t,0x25);
    lcd_0_96_write_data8(lcd_t,0x36);
    lcd_0_96_write_data8(lcd_t,0x00);
    lcd_0_96_write_data8(lcd_t,0x08);
    lcd_0_96_write_data8(lcd_t,0x04);
    lcd_0_96_write_data8(lcd_t,0x10);

	lcd_0_96_write_reg(lcd_t,0xE1);     //negative gamma
    lcd_0_96_write_data8(lcd_t,0x0A);
    lcd_0_96_write_data8(lcd_t,0x0D);
    lcd_0_96_write_data8(lcd_t,0x08);
    lcd_0_96_write_data8(lcd_t,0x07);
    lcd_0_96_write_data8(lcd_t,0x0F);
    lcd_0_96_write_data8(lcd_t,0x07);
    lcd_0_96_write_data8(lcd_t,0x02);
    lcd_0_96_write_data8(lcd_t,0x07);
    lcd_0_96_write_data8(lcd_t,0x09);
    lcd_0_96_write_data8(lcd_t,0x0F);
    lcd_0_96_write_data8(lcd_t,0x25);
    lcd_0_96_write_data8(lcd_t,0x35);
    lcd_0_96_write_data8(lcd_t,0x00);
    lcd_0_96_write_data8(lcd_t,0x09);
    lcd_0_96_write_data8(lcd_t,0x04);
    lcd_0_96_write_data8(lcd_t,0x10);

	lcd_0_96_write_reg(lcd_t,0xFC);
    lcd_0_96_write_data8(lcd_t,0x80);

	lcd_0_96_write_reg(lcd_t,0x3A);
    lcd_0_96_write_data8(lcd_t,0x05);

	lcd_0_96_write_reg(lcd_t,0x36);
	if(lcd_t->lcd_p->lcd_direction == 0)lcd_0_96_write_data8(lcd_t,0x08);
	else if(lcd_t->lcd_p->lcd_direction == 1)lcd_0_96_write_data8(lcd_t,0xC8);
	else if(lcd_t->lcd_p->lcd_direction == 2)lcd_0_96_write_data8(lcd_t,0x78);
	else lcd_0_96_write_data8(lcd_t,0xA8);

	lcd_0_96_write_reg(lcd_t,0x20);     //Display inversion
	lcd_0_96_write_reg(lcd_t,0x29);     //Display on
	lcd_0_96_write_reg(lcd_t,0x2A);     //Set Column Address
    lcd_0_96_write_data8(lcd_t,0x00);
    lcd_0_96_write_data8(lcd_t,0x1A);
    lcd_0_96_write_data8(lcd_t,0x00);
    lcd_0_96_write_data8(lcd_t,0x69);

	lcd_0_96_write_reg(lcd_t,0x2B);     //Set Page Address
    lcd_0_96_write_data8(lcd_t,0x00);
    lcd_0_96_write_data8(lcd_t,0x01);
    lcd_0_96_write_data8(lcd_t,0x00);
    lcd_0_96_write_data8(lcd_t,0xA0);

	lcd_0_96_write_reg(lcd_t,0x2C);
}

void register_0_96_lcd(lcd_parameter *lcd_p,lcd_interface * lcd_t)
{
	lcd_p->hspi = &hspi1;
	lcd_p->lcd_direction = 3;
	if(lcd_p->lcd_direction == 0 || lcd_p->lcd_direction == 1)
	{
		lcd_p->lcd_width = 160;
		lcd_p->lcd_height = 80;
	}
	else
	{
		lcd_p->lcd_width = 80;
		lcd_p->lcd_height = 160;
	}
	lcd_p->lcd_cs_gpio = SPI_CS_GPIO_Port;
	lcd_p->lcd_cs_pin = SPI_CS_Pin;
	lcd_p->lcd_blk_gpio = LCD_BLK_GPIO_Port;
	lcd_p->lcd_blk_pin = LCD_BLK_Pin;
	lcd_p->lcd_dc_gpio = LCD_DC_GPIO_Port;
	lcd_p->lcd_dc_pin = LCD_DC_Pin;
	lcd_p->lcd_res_gpio = LCD_RESET_GPIO_Port;
	lcd_p->lcd_res_pin = LCD_RESET_Pin;
	lcd_t->lcd_p = lcd_p;
	lcd_t->lcd_init = lcd_0_96_init;
	lcd_t->lcd_address_set = lcd_0_96_address_set;
	lcd_t->lcd_write_data16 = lcd_0_96_write_data16;
	lcd_t->lcd_write_data8 = lcd_0_96_write_data8;
	lcd_t->lcd_write_reg = lcd_0_96_write_reg;
	lcd_t->lcd_enter_sleep = lcd_0_96_enter_sleep;
	lcd_t->lcd_exit_sleep = lcd_0_96_exit_sleep;
}

#elif defined(ENABLE_1_8_LCD)

void lcd_1_80_write_data8(lcd_interface *lcd_t,uint8_t data)
{
	//select lcd
	HAL_GPIO_WritePin(lcd_t->lcd_p->lcd_cs_gpio,lcd_t->lcd_p->lcd_cs_pin,GPIO_PIN_RESET);

	HAL_SPI_Transmit(lcd_t->lcd_p->hspi,&data,1,1000);

	//deselect lcd
	HAL_GPIO_WritePin(lcd_t->lcd_p->lcd_cs_gpio,lcd_t->lcd_p->lcd_cs_pin,GPIO_PIN_SET);
}

void lcd_1_80_write_data16(lcd_interface *lcd_t,uint16_t data)
{
	uint8_t sendData[2] = {0};
	sendData[0] = (data>>8);
	sendData[1] = (data);

	//select lcd
	HAL_GPIO_WritePin(lcd_t->lcd_p->lcd_cs_gpio,lcd_t->lcd_p->lcd_cs_pin,GPIO_PIN_RESET);

	HAL_SPI_Transmit(lcd_t->lcd_p->hspi,sendData,2,1000);

	//deselect lcd
	HAL_GPIO_WritePin(lcd_t->lcd_p->lcd_cs_gpio,lcd_t->lcd_p->lcd_cs_pin,GPIO_PIN_SET);
}

void lcd_1_80_write_reg(lcd_interface *lcd_t,uint8_t reg)
{
	HAL_GPIO_WritePin(lcd_t->lcd_p->lcd_dc_gpio,lcd_t->lcd_p->lcd_dc_pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(lcd_t->lcd_p->lcd_cs_gpio,lcd_t->lcd_p->lcd_cs_pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(lcd_t->lcd_p->hspi,&reg,1,1000);
	HAL_GPIO_WritePin(lcd_t->lcd_p->lcd_dc_gpio,lcd_t->lcd_p->lcd_dc_pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(lcd_t->lcd_p->lcd_cs_gpio,lcd_t->lcd_p->lcd_cs_pin,GPIO_PIN_SET);
}

void lcd_1_80_address_set(lcd_interface *lcd_t,uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{
    if(lcd_t->lcd_p->lcd_direction == 0 || lcd_t->lcd_p->lcd_direction == 1){
        lcd_1_80_write_reg(lcd_t,0x2a);
	    lcd_1_80_write_data16(lcd_t,x1);
	    lcd_1_80_write_data16(lcd_t,x2);
	    lcd_1_80_write_reg(lcd_t,0x2b);
	    lcd_1_80_write_data16(lcd_t,y1);
	    lcd_1_80_write_data16(lcd_t,y2);
	    lcd_1_80_write_reg(lcd_t,0x2c);
    }
	if(lcd_t->lcd_p->lcd_direction == 2 || lcd_t->lcd_p->lcd_direction == 3){
		lcd_1_80_write_reg(lcd_t,0x2a);
	    lcd_1_80_write_data16(lcd_t,x1);
	    lcd_1_80_write_data16(lcd_t,x2);
	    lcd_1_80_write_reg(lcd_t,0x2b);
	    lcd_1_80_write_data16(lcd_t,y1);
	    lcd_1_80_write_data16(lcd_t,y2);
	    lcd_1_80_write_reg(lcd_t,0x2c);
    }
}

void lcd_1_80_enter_sleep(lcd_interface *lcd_t)
{
	lcd_1_80_write_reg(lcd_t,0x28);     	//Display off
	HAL_Delay(10);
	lcd_1_80_write_reg(lcd_t,0x10);     	//Sleep enter
	HAL_Delay(120);
}

void lcd_1_80_exit_sleep(lcd_interface *lcd_t)
{
	lcd_1_80_write_reg(lcd_t,0x11);     	//Sleep out
	HAL_Delay(120);
	lcd_1_80_write_reg(lcd_t,0x29);     	//Display on
	HAL_Delay(10);
	lcd_1_80_write_reg(lcd_t,0x2C);
}

void lcd_1_80_init(lcd_interface *lcd_t)
{
	HAL_GPIO_WritePin(lcd_t->lcd_p->lcd_res_gpio, lcd_t->lcd_p->lcd_res_pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(lcd_t->lcd_p->lcd_res_gpio, lcd_t->lcd_p->lcd_res_pin, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(lcd_t->lcd_p->lcd_blk_gpio, lcd_t->lcd_p->lcd_blk_pin, GPIO_PIN_SET);
    HAL_Delay(100);

	//************* Start Initial Sequence **********//
	lcd_1_80_write_reg(lcd_t,0x11); //Sleep out
	HAL_Delay(120);	//Delay 120ms
	//------------------------------------ST7735S Frame Rate-----------------------------------------//
	lcd_1_80_write_reg(lcd_t,0xB1);     	//Normal mode
	lcd_1_80_write_data8(lcd_t,0x05);
    lcd_1_80_write_data8(lcd_t,0x3c);
    lcd_1_80_write_data8(lcd_t,0x3c);

	lcd_1_80_write_reg(lcd_t,0xB2);     	//Idle mode
	lcd_1_80_write_data8(lcd_t,0x05);
    lcd_1_80_write_data8(lcd_t,0x3c);
    lcd_1_80_write_data8(lcd_t,0x3c);

    lcd_1_80_write_reg(lcd_t,0xB3);     	//Partial mode
    lcd_1_80_write_data8(lcd_t,0x05);
    lcd_1_80_write_data8(lcd_t,0x3c);
    lcd_1_80_write_data8(lcd_t,0x3c);
    lcd_1_80_write_data8(lcd_t,0x05);
    lcd_1_80_write_data8(lcd_t,0x3c);
    lcd_1_80_write_data8(lcd_t,0x3c);

    lcd_1_80_write_reg(lcd_t,0xB4);     //Dot inversion
    lcd_1_80_write_data8(lcd_t,0x03);

    lcd_1_80_write_reg(lcd_t,0xC0);     //AVDD GVDD
	lcd_1_80_write_data8(lcd_t,0x28);
	lcd_1_80_write_data8(lcd_t,0x08);
	lcd_1_80_write_data8(lcd_t,0x04);

	lcd_1_80_write_reg(lcd_t,0xC1);     //VGH VGL
	lcd_1_80_write_data8(lcd_t,0xC0);   //C0

	lcd_1_80_write_reg(lcd_t,0xC2);     //Normal Mode
	lcd_1_80_write_data8(lcd_t,0x0D);
	lcd_1_80_write_data8(lcd_t,0x00);

	lcd_1_80_write_reg(lcd_t,0xC3);     //Idle
	lcd_1_80_write_data8(lcd_t,0x8D);
	lcd_1_80_write_data8(lcd_t,0x2A);

	lcd_1_80_write_reg(lcd_t,0xC4);     //Partial+Full
	lcd_1_80_write_data8(lcd_t,0x8D);
	lcd_1_80_write_data8(lcd_t,0xEE);

	lcd_1_80_write_reg(lcd_t,0xC5);     //VCOM
	lcd_1_80_write_data8(lcd_t,0x1A);

	lcd_1_80_write_reg(lcd_t,0x36);     //RGB mode
	if(lcd_t->lcd_p->lcd_direction == 0)
	{
		lcd_1_80_write_data8(lcd_t,0x00);
	}
	else if(lcd_t->lcd_p->lcd_direction == 1)
	{
		lcd_1_80_write_data8(lcd_t,0xC0);
	}
	else if(lcd_t->lcd_p->lcd_direction == 2)
	{
		lcd_1_80_write_data8(lcd_t,0x70);
	}
	else
	{
		lcd_1_80_write_data8(lcd_t,0xA0);
	}

	lcd_1_80_write_reg(lcd_t,0xE0);     //Gamma Sequence
	lcd_1_80_write_data8(lcd_t,0x04);
	lcd_1_80_write_data8(lcd_t,0x22);
	lcd_1_80_write_data8(lcd_t,0x07);
	lcd_1_80_write_data8(lcd_t,0x0A);
	lcd_1_80_write_data8(lcd_t,0x2E);
	lcd_1_80_write_data8(lcd_t,0x30);
	lcd_1_80_write_data8(lcd_t,0x25);
	lcd_1_80_write_data8(lcd_t,0x2A);
	lcd_1_80_write_data8(lcd_t,0x28);
	lcd_1_80_write_data8(lcd_t,0x26);
	lcd_1_80_write_data8(lcd_t,0x2E);
	lcd_1_80_write_data8(lcd_t,0x3A);
	lcd_1_80_write_data8(lcd_t,0x00);
	lcd_1_80_write_data8(lcd_t,0x01);
	lcd_1_80_write_data8(lcd_t,0x03);
	lcd_1_80_write_data8(lcd_t,0x13);

	lcd_1_80_write_reg(lcd_t,0xE1);     //Gamma Sequence
	lcd_1_80_write_data8(lcd_t,0x04);
	lcd_1_80_write_data8(lcd_t,0x16);
	lcd_1_80_write_data8(lcd_t,0x06);
	lcd_1_80_write_data8(lcd_t,0x0D);
	lcd_1_80_write_data8(lcd_t,0x2D);
	lcd_1_80_write_data8(lcd_t,0x26);
	lcd_1_80_write_data8(lcd_t,0x23);
	lcd_1_80_write_data8(lcd_t,0x27);
	lcd_1_80_write_data8(lcd_t,0x27);
	lcd_1_80_write_data8(lcd_t,0x25);
	lcd_1_80_write_data8(lcd_t,0x2D);
	lcd_1_80_write_data8(lcd_t,0x3B);
	lcd_1_80_write_data8(lcd_t,0x00);
	lcd_1_80_write_data8(lcd_t,0x01);
	lcd_1_80_write_data8(lcd_t,0x04);
	lcd_1_80_write_data8(lcd_t,0x13);

	lcd_1_80_write_reg(lcd_t,0x3A);     //65k mode
	lcd_1_80_write_data8(lcd_t,0x05);
	lcd_1_80_write_reg(lcd_t,0x29);	//Display on
}

void register_1_80_lcd(lcd_parameter *lcd_p,lcd_interface * lcd_t)
{
	lcd_p->hspi = &hspi1;
	lcd_p->lcd_direction = 3;
	if(lcd_p->lcd_direction == 0 || lcd_p->lcd_direction == 1)
	{
		lcd_p->lcd_width = 160;
		lcd_p->lcd_height = 128;
	}
	else
	{
		lcd_p->lcd_width = 128;
		lcd_p->lcd_height = 160;
	}
	lcd_p->lcd_cs_gpio = LCD_CS_GPIO_Port;
	lcd_p->lcd_cs_pin = LCD_CS_Pin;
	lcd_p->lcd_blk_gpio = LCD_BL_GPIO_Port;
	lcd_p->lcd_blk_pin = LCD_BL_Pin;
	lcd_p->lcd_dc_gpio = LCD_DC_GPIO_Port;
	lcd_p->lcd_dc_pin = LCD_DC_Pin;
	lcd_p->lcd_res_gpio = LCD_RES_GPIO_Port;
	lcd_p->lcd_res_pin = LCD_RES_Pin;
	lcd_t->lcd_p = lcd_p;
	lcd_t->lcd_init = lcd_1_80_init;
	lcd_t->lcd_address_set = lcd_1_80_address_set;
	lcd_t->lcd_write_data16 = lcd_1_80_write_data16;
	lcd_t->lcd_write_data8 = lcd_1_80_write_data8;
	lcd_t->lcd_write_reg = lcd_1_80_write_reg;
	lcd_t->lcd_enter_sleep = lcd_1_80_enter_sleep;
	lcd_t->lcd_exit_sleep = lcd_1_80_exit_sleep;
}

#endif
