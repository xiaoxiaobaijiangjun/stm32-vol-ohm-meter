/*
 * app_task.c
 *
 *  Created on: 2026年1月26日
 *      Author: jlc
 */

#include "app_task.h"
#include "../hardware/hw_key.h"
#include "../hardware/hw_lcdinit.h"
#include "../middle/mid_kalman.h"
#include "../middle/mid_lcd.h"
#include "../middle/mid_filter.h"
#include "stdio.h"
#include "adc.h"

#define ADC_CH_NUM	7	//adc channel number
#define VOL_RANGE_0_5 	0x01	// vol range 0v-5v flag
#define VOL_RANGE_5_15	0x02	// vol range 5v-15v flag
#define RES_RANGE_0_1	0x03	// res range 0r-1k flag
#define RES_RANGE_1_100	0x04	// res range 1k-100k flag
#define RES_MODE		0x05	// res mode
#define VOL_MODE		0x06	// vol mode
#define BUZ_MODE		0x07	// buz mode

static lcd_parameter lcd_p;
static lcd_interface lcd_t;
static key_handle_t mode_key;

static uint16_t adc_buf[ADC_CH_NUM];	// adc dma buffer
static volatile uint8_t key_trigger_bit = 0;	// key interrupt flag
static volatile uint8_t adc_dma_done = 0;		// adc transition complete flag
static volatile uint8_t sample_cnt = 0;			//cumsum num
static uint8_t mode_sel = VOL_MODE;	//mode select,default vol mode
static uint8_t range_sel = VOL_RANGE_5_15;	// range select, default range is 5v-15v
static uint32_t ref_165_sum;	//ref_165_vol cumsum
static uint32_t vol1_sum;		//vol vin1 cumsum
static uint32_t vol2_sum;		//vol vin2 cumsum
static uint32_t res_in1_sum;	//resistance vin2 vol cumsum
static uint32_t res_in2_sum;	//resistance vin1 vol cumsum
static uint32_t ref_230_sum;	//ref_230_vol cumsum
static uint32_t vref_sum;		//interior vref sum
static float old_vol1_val;
static float old_vol2_val;

static void show_resistance_page(lcd_interface *lcd_t,uint16_t *adc_buf,uint8_t *range);
static void show_volatge_page(lcd_interface *lcd_t,uint16_t *adc_buf,uint8_t *range);
static void show_beep_page(lcd_interface *lcd_t,uint16_t *adc_buf,uint8_t *range);
static void key_handle(key_handle_t *key_t,lcd_interface *lcd_t,uint8_t *mode_sel,uint8_t *range_sel);
static void reset_adc_gather(uint16_t * adc_buf);
static void show_hint(lcd_interface *lcd_t);

void init_task(void)
{
	register_1_80_lcd(&lcd_p,&lcd_t);
	mode_key = Key_Init(MODE_GPIO_Port,MODE_Pin,GPIO_PIN_RESET);
	lcd_t.lcd_init(&lcd_t);
	lcd_t.lcd_exit_sleep(&lcd_t);
	lcd_fill(&lcd_t,0,0,lcd_t.lcd_p->lcd_height,lcd_t.lcd_p->lcd_width,BLACK);
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_CH_NUM);
}

void cycle_task(void)
{
	if(key_trigger_bit > 0)
	{
		if(HAL_GPIO_ReadPin(mode_key.key_gpio,mode_key.key_pin) == mode_key.key_press_level)
		{
			for(uint16_t i=0;i<500;i++)
			{
				Key_Scanf(&mode_key);
				HAL_Delay(10);
				if(mode_key.key_state != KEY_NoPress){
					break;
				}
			}
		}
		else
		{
			if(key_trigger_bit == 1){
				mode_key.key_state = KeyPress;
			}
		}
		key_handle(&mode_key,&lcd_t,&mode_sel,&range_sel);
		mode_key.key_state = KEY_NoPress;
		key_trigger_bit = 0;
	}
	if(mode_sel == VOL_MODE){
		show_volatge_page(&lcd_t,&adc_buf[0],&range_sel);
	}
	else if(mode_sel == RES_MODE){
		show_resistance_page(&lcd_t,&adc_buf[0],&range_sel);
	}
	else if(mode_sel == BUZ_MODE){
		show_beep_page(&lcd_t,&adc_buf[0],&range_sel);
	}
}

static void reset_adc_gather(uint16_t * adc_buf)
{
	ref_165_sum = 0;
	vol1_sum = 0;
	vol2_sum = 0;
	res_in1_sum = 0;
	res_in2_sum = 0;
	ref_230_sum = 0;
	vref_sum = 0;
	adc_dma_done = 0;
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_CH_NUM);
}

static inline float my_fabs(float x)
{
    return (x < 0.0f) ? -x : x;
}

/*
 * Function Content: show volatge value;
 * 					notic offset_vol need manual measurement,
 * 					The output voltage value of the follower when unloaded
 * Function Parameter: lcd_interface *lcd_t,uint16_t *adc_buf,uint8_t range
 * Return Value: null
 * notice: offset_val is vol input connect to GND offset val,
 * 			No remove of levitate noise!!!
 */
static void show_volatge_page(lcd_interface *lcd_t,uint16_t *adc_buf,uint8_t *range)
{
    char show_data[32];
    uint16_t vol_color;
    float vol1_val = 0.0f,vol2_val = 0.0f,ref165_vol = 0.0f,offset_val=0.0f,filter_vol = 0.0f;
    uint16_t vin_vol = 0;
    uint8_t sign = 0;

    if(adc_dma_done != 1){
    	return;
    }
    vol1_sum = vol1_sum >> 6;
    vol2_sum = vol2_sum >> 6;
    ref_165_sum = ref_165_sum >> 6;
    vref_sum = vref_sum >> 6;

    vol1_val = vol1_sum * 1.0f / vref_sum * 1.21f;
    vol2_val = vol2_sum * 1.0f / vref_sum * 1.21f;
    ref165_vol = ref_165_sum * 1.0f / vref_sum * 1.21f;

    reset_adc_gather(adc_buf);

    lcd_showString(lcd_t,12,2,(uint8_t *)"Mode:Vol",
    				WHITE, BLACK, 16, 0);

    if(((old_vol1_val + 0.1f) <= vol1_val) || ((old_vol1_val - 0.1f) >= vol1_val)){
    	old_vol1_val = vol1_val;
    	*range = VOL_RANGE_0_5;
    }else if(((old_vol2_val + 0.1f) <= vol2_val) || ((old_vol2_val - 0.1f) >= vol2_val)){
    	old_vol2_val = vol2_val;
    	*range = VOL_RANGE_5_15;
    }

    if(*range == VOL_RANGE_0_5){
    	offset_val = 0.005f;
		lcd_showString(lcd_t,100,2,(uint8_t *)"0--5V",
						LGRAY, BLACK, 16, 0);
		if(vol1_val >= ref165_vol){
			filter_vol = (vol1_val - ref165_vol - offset_val) * 13.3f / 3.3f * 1000;
			sign = 1;
		}else{
			filter_vol = (ref165_vol - vol1_val + offset_val) * 13.3f / 3.3f * 1000;
			sign = 0;
		}
    }
    else{
    	offset_val = 0.005f;
    	lcd_showString(lcd_t,100,2,(uint8_t *)"0-15V",
    			  				LGRAY, BLACK, 16, 0);
    	if(vol2_val >= ref165_vol){
    		filter_vol = (vol2_val - ref165_vol - offset_val) * 11.0f * 1000;
    		sign = 1;
		}else{
			filter_vol = (ref165_vol - vol2_val + offset_val) * 11.0f * 1000;
			sign = 0;
		}
    }
    vin_vol = Moving_Average(filter_vol);
    if (vin_vol < 5000) {
        vol_color = GREEN;
    } else if (vin_vol < 10000) {
        vol_color = YELLOW;
    } else {
        vol_color = RED;
    }

    if(sign == 0){
    	lcd_showString(lcd_t,8,48,(uint8_t *)"-", LGRAY, BLACK, 32, 0);
    }else{
    	lcd_showString(lcd_t,8,48,(uint8_t *)"+", LGRAY, BLACK, 32, 0);
    }
    snprintf(show_data,sizeof(show_data),"%2d.%03d",
    				vin_vol/1000, vin_vol%1000);

    lcd_showString(lcd_t,24,48,(uint8_t *)show_data,
                   vol_color, BLACK, 32, 0);

    lcd_showString(lcd_t,120,48,(uint8_t *)"V",
    				LGRAY, BLACK, 32, 0);

    lcd_showString(lcd_t,0,110,
                   (uint8_t *)"0-5V<---sw--->5V-15V",
				   DARKGRAY, BLACK, 16, 0);
}

static void show_resistance_page(lcd_interface *lcd_t,uint16_t *adc_buf,uint8_t *range)
{
    char show_data[32];
    uint16_t res_color = 0;
    uint32_t resistance = 0;
    float ref165_vol = 0,res1_in = 0,res2_in,ref230_vol = 0;
    float current_val = 0;

    if(adc_dma_done != 1){
    	return;
    }
    ref_165_sum = ref_165_sum >> 6;
    ref_230_sum = ref_230_sum >> 6;
    res_in1_sum = res_in1_sum >> 6;
    res_in2_sum = res_in2_sum >> 6;
    vref_sum = vref_sum >> 6;

    ref165_vol = ref_165_sum * 1.0f / vref_sum * 1.21f;
    res1_in = res_in1_sum * 1.0f / vref_sum * 1.21f;
    res2_in = res_in2_sum * 1.0f / vref_sum * 1.21f;
    ref230_vol = ref_230_sum * 1.0f / vref_sum * 1.21f;

    reset_adc_gather(adc_buf);

    current_val = (ref165_vol * 2.0f - ref230_vol) / 1000.0f;


    lcd_showString(lcd_t,12,2,(uint8_t *)"Mode:Res",
    				WHITE, BLACK, 16, 0);

    if(res1_in < 3.30f){
    	*range = RES_RANGE_0_1;
    }else{
    	*range = RES_RANGE_1_100;
    }

    if(*range == RES_RANGE_0_1){
    	lcd_showString(lcd_t,100,2,(uint8_t *)"0K---1K",
    	    				LGRAY, BLACK, 16, 0);
    	resistance = (res1_in / current_val) + 0.5f;
    	res_color = WHITE;
    	if(resistance > 1000 && resistance<= 3000){
    		HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
    		HAL_Delay(50);
    		HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
		}
    	else if(resistance > 3000)
    	{
    		resistance = 0;
    		HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
    	}
    }else{
    	lcd_showString(lcd_t,100,2,(uint8_t *)"1K-100K",
							LGRAY, BLACK, 16, 0);
    	resistance = (10000.0f) * (res2_in / ((ref165_vol * 2.0f) - res2_in))+0.5f;
    	if (resistance < 47000) {
			res_color = ORANGE;
		} else {
			res_color = YELLOW;
		}
    	if((resistance > 100000) && (resistance <= 150000)){
    		HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
			HAL_Delay(50);
			HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
		}
    	else if(resistance > 150000){
    		resistance = 0;
			HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
    	}
    }
    resistance = Moving_Average_res(resistance);
    if (resistance < 1000) {
        snprintf(show_data, sizeof(show_data),
                 "%6ld", resistance);
    } else {
        snprintf(show_data, sizeof(show_data),
                 "%3ld.%02ld",
                 resistance / 1000,
                 (resistance % 1000) / 10);
    }
    lcd_showString(lcd_t,24,48,(uint8_t *)show_data,
                   res_color, BLACK, 32, 0);

    if (resistance < 1000) {
    	lcd_showString(lcd_t,120,48,(uint8_t *)"R",
        				LGRAY, BLACK, 32, 0);
    }else {
    	lcd_showString(lcd_t,120,48,(uint8_t *)"K",
    	        				LGRAY, BLACK, 32, 0);
    }

    lcd_showString(lcd_t,0,110,
                   (uint8_t *)"1k-100K<--sw-->0R-1K",
				   DARKGRAY, BLACK, 16, 0);
}


static void show_beep_page(lcd_interface *lcd_t,uint16_t *adc_buf,uint8_t *range)
{
    char show_data[32];
    uint16_t res_color = 0;
    uint32_t resistance = 0;
    float ref165_vol = 0,res1_in = 0,ref230_vol = 0;
    float current_val = 0;

    if(adc_dma_done != 1){
    	return;
    }
    ref_165_sum = ref_165_sum >> 6;
    ref_230_sum = ref_230_sum >> 6;
    res_in1_sum = res_in1_sum >> 6;
    res_in2_sum = res_in2_sum >> 6;
    vref_sum = vref_sum >> 6;

    ref165_vol = ref_165_sum * 1.0f / vref_sum * 1.21f;
    res1_in = res_in1_sum * 1.0f / vref_sum * 1.21f;
    //res2_in = res_in2_sum * 1.0f / vref_sum * 1.21f;
    ref230_vol = ref_230_sum * 1.0f / vref_sum * 1.21f;

    reset_adc_gather(adc_buf);

    current_val = (ref165_vol * 2.0f - ref230_vol) / 1000.0f;


    lcd_showString(lcd_t,12,2,(uint8_t *)"Mode:Buz",
    				WHITE, BLACK, 16, 0);

    if(res1_in < 3.30f){
    	*range = RES_RANGE_0_1;
    }else{
    	*range = RES_RANGE_1_100;
    }

    if(*range == RES_RANGE_0_1){
    	resistance = res1_in / current_val;
    	res_color = WHITE;
    	if(resistance > 1000){
			resistance = 9999;
		}
    }else{
    	resistance = 9999;
    }
    resistance = Moving_Average_res(resistance);
    if(resistance <= 15){
    	snprintf(show_data, sizeof(show_data),
				 "%6ld", resistance);
		lcd_showString(lcd_t,24,48,(uint8_t *)show_data,
						   res_color, BLACK, 32, 0);
		lcd_showString(lcd_t,120,48,(uint8_t *)"R",
								LGRAY, BLACK, 32, 0);
		HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
    }
    else if (resistance <= 1000) {
		snprintf(show_data, sizeof(show_data),
				 "%6ld", resistance);
		lcd_showString(lcd_t,24,48,(uint8_t *)show_data,
						   res_color, BLACK, 32, 0);
		lcd_showString(lcd_t,120,48,(uint8_t *)"R",
								LGRAY, BLACK, 32, 0);
		HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
    }
    else{
    	resistance = 0;
    	snprintf(show_data, sizeof(show_data),
    					 "%6ld", resistance);
		lcd_showString(lcd_t,24,48,(uint8_t *)show_data,
						   res_color, BLACK, 32, 0);
    	lcd_showString(lcd_t,88,48,(uint8_t *)"OL.",
    									LGRAY, BLACK, 32, 0);
    	HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
    }

    lcd_showString(lcd_t,0,110,
                   (uint8_t *)"1k-100K<--sw-->0R-1K",
				   DARKGRAY, BLACK, 16, 0);
}

static void show_hint(lcd_interface *lcd_t)
{
	lcd_showString(lcd_t,0,48,(uint8_t *)"notice sw!",
			ORANGE, BLACK, 32, 0);
}

static void key_handle(key_handle_t *key_t,lcd_interface *lcd_t,uint8_t *mode_sel,uint8_t *range_sel)
{
	if(key_t->key_state != KEY_NoPress){
		switch(key_t->key_state)
		{
		case KeyPress:
			lcd_fill(lcd_t,0,0,lcd_t->lcd_p->lcd_height,lcd_t->lcd_p->lcd_width,BLACK);
			show_hint(lcd_t);
			HAL_Delay(2000);
			lcd_fill(lcd_t,0,0,lcd_t->lcd_p->lcd_height,lcd_t->lcd_p->lcd_width,BLACK);
			if(*mode_sel == VOL_MODE){
				*mode_sel = RES_MODE;
			}else if(*mode_sel == RES_MODE){
				*mode_sel = BUZ_MODE;
			}
			else if(*mode_sel == BUZ_MODE){
				*mode_sel = VOL_MODE;
			}
			break;
		case KeyDoublePress:
			break;
		case KeyLongPress:
			break;
		default:
			break;
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_9){
		key_trigger_bit = 1;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1)
	{
		ref_165_sum += adc_buf[0];
		vol1_sum += adc_buf[1];
		vol2_sum += adc_buf[2];
		res_in1_sum += adc_buf[3];
		res_in2_sum += adc_buf[4];
		ref_230_sum += adc_buf[5];
		vref_sum += adc_buf[6];
		sample_cnt++;
		if(sample_cnt>=64){
			HAL_ADC_Stop_DMA(&hadc1);
			sample_cnt = 0;
			adc_dma_done = 1;
		}

	}
}
