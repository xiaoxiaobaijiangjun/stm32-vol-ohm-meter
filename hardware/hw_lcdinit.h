#ifndef __HW_LCDINIT_H
#define __HW_LCDINIT_H

#include "main.h"

typedef struct LCD_PARAMETER
{
	uint8_t lcd_direction;
	uint16_t lcd_width;
	uint16_t lcd_height;
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef* lcd_cs_gpio;
	uint16_t lcd_cs_pin;
	GPIO_TypeDef* lcd_dc_gpio;
	uint16_t lcd_dc_pin;
	GPIO_TypeDef* lcd_res_gpio;
	uint16_t lcd_res_pin;
	GPIO_TypeDef* lcd_blk_gpio;
	uint16_t lcd_blk_pin;
}lcd_parameter;

typedef struct LCD_INTERFACE
{
	lcd_parameter *lcd_p;
	void (* lcd_write_data8)(struct LCD_INTERFACE *,uint8_t);
	void (* lcd_write_data16)(struct LCD_INTERFACE *,uint16_t);
	void (* lcd_write_reg)(struct LCD_INTERFACE *,uint8_t);
	void (* lcd_address_set)(struct LCD_INTERFACE *,uint16_t,uint16_t,uint16_t,uint16_t);
	void (* lcd_enter_sleep)(struct LCD_INTERFACE *);
	void (* lcd_exit_sleep)(struct LCD_INTERFACE *);
	void (* lcd_init)(struct LCD_INTERFACE *);
}lcd_interface;

void register_0_96_lcd(lcd_parameter *lcd_p,lcd_interface * lcd_t);
void register_1_80_lcd(lcd_parameter *lcd_p,lcd_interface * lcd_t);

#endif
