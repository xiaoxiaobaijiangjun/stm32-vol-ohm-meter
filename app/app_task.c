/*
 * app_task.c
 *
 * Application layer: measurement modes, UI pages and key handling.
 *
 *  Created on: 2026-01-26
 *      Author: xiaoxiaobaijiangjun
 */

#include "app_task.h"
#include "../hardware/hw_key.h"
#include "../hardware/hw_lcdinit.h"
#include "../middle/mid_lcd.h"
#include "../middle/mid_filter.h"
#include "../middle/mid_calib.h"
#include "stdio.h"
#include "stdlib.h"
#include "adc.h"

#define ADC_CH_NUM		7		/* number of ADC channels scanned by DMA   */
#define VOL_RANGE_0_5	0x01	/* voltage range 0V - 5V                   */
#define VOL_RANGE_5_15	0x02	/* voltage range 5V - 15V                  */
#define RES_RANGE_0_1	0x03	/* resistance range 0R - 1K                */
#define RES_RANGE_1_100	0x04	/* resistance range 1K - 100K              */
#define RES_MODE		0x05	/* resistance measurement mode             */
#define VOL_MODE		0x06	/* voltage measurement mode                */
#define BUZ_MODE		0x07	/* continuity (buzzer) mode                */
/* Open-circuit / overrange detection thresholds for the resistance
 * page. Initial values from design analysis: refine on the bench. */
#define CAL_OPEN_I_MIN   0.0001f   /* A, constant-current floor: below = open */
#define CAL_RES1_SAT_V   3.25f     /* V, 0-1K front-end saturation threshold  */
#define CAL_RES2_FLOOR_V 0.05f     /* V, 1K-100K divider midpoint floor       */


static lcd_parameter lcd_p;
static lcd_interface lcd_t;
static key_handle_t mode_key;

static uint16_t adc_buf[ADC_CH_NUM];			/* ADC DMA conversion buffer     */
static volatile uint8_t key_trigger_bit = 0;	/* set by the key EXTI handler  */
static volatile uint8_t adc_dma_done = 0;		/* one 64-round ADC batch ready */
static volatile uint8_t sample_cnt = 0;			/* rounds accumulated so far    */
static uint8_t mode_sel = VOL_MODE;				/* current mode, default: volt  */
static uint8_t range_sel = VOL_RANGE_5_15;		/* current range, default 5-15V */
static uint32_t ref_165_sum;	/* 1.65V reference, cumulative ADC raw      */
static uint32_t vol1_sum;		/* voltage channel 1 (0-5V divider), raw    */
static uint32_t vol2_sum;		/* voltage channel 2 (5-15V divider), raw   */
static uint32_t res_in1_sum;	/* resistance node 1 (0-1K path), raw       */
static uint32_t res_in2_sum;	/* resistance node 2 (1K-100K path), raw    */
static uint32_t ref_230_sum;	/* 2.30V-ish reference node, raw            */
static uint32_t vref_sum;		/* STM32 internal VREFINT, raw              */
static float old_vol1_val;
static float old_vol2_val;
static uint32_t key_last_scan_tick = 0;		/* tick of the last Key_Scanf() call */
static uint32_t key_event_start_tick = 0;	/* tick when the current key event began */

/* Reference fixture true values: replace with the values measured by the
 * reference multimeter BEFORE the bench session (units mV / ohm, indexed
 * like the CAL_IDX_* ranges). MUST-EDIT before benching. */
static const float CAL_FIXTURE_Y[CAL_RANGE_COUNT] = {
    3312.0f,    /* [0] 0-5V range: 3.3V rail, placeholder */
    9000.0f,    /* [1] 5-15V range: 9V battery, placeholder */
    330.0f,     /* [2] 0-1K range: 330 ohm, placeholder */
    10000.0f,   /* [3] 1K-100K range: 10k ohm, placeholder */
};
/* minimum |x2 - x1| per range (mV / ohm): two points too close make k explode */
static const float CAL_MIN_SPAN[CAL_RANGE_COUNT] = {500.0f, 1000.0f, 100.0f, 2000.0f};

/* onboard calibration UI state machine (implementation before key_handle) */
typedef enum {
    CAL_UI_IDLE = 0,    /* not running                                  */
    CAL_UI_SHORT,       /* step 1: short the probes, click to capture x1 */
    CAL_UI_REF,         /* step 2: apply the fixture, click to capture x2 */
    CAL_UI_COLLECT,     /* averaging samples for the current step        */
    CAL_UI_OK_ERR       /* final banner, auto exit                       */
} cal_ui_state_t;

#define CAL_UI_SAMPLE_MS   30U     /* min spacing between capture samples */
#define CAL_UI_SAMPLE_N    8U      /* samples averaged per capture point  */
#define CAL_UI_TIMEOUT_MS  3000U   /* whole-capture timeout               */
#define CAL_UI_BANNER_MS   1500U   /* CAL OK / CAL ERR display time       */

static cal_ui_state_t cal_ui_state = CAL_UI_IDLE;
static uint8_t  cal_ui_cidx = 0U;           /* range snapshot at entry     */
static uint8_t  cal_ui_collecting_x2 = 0U;  /* 1 = COLLECT gathers x2      */
static float    cal_ui_x1 = 0.0f;
static uint32_t cal_ui_step_tick = 0U;
static uint32_t cal_ui_last_sample_tick = 0U;
static uint8_t  cal_ui_samples = 0U;
static float    cal_ui_sum = 0.0f;
static uint8_t  cal_ui_ok = 0U;
static uint32_t cal_ui_banner_tick = 0U;
static uint32_t cal_ui_draw_tick = 0U;


static void show_resistance_page(lcd_interface *lcd_t,uint16_t *adc_buf,uint8_t *range);
static void show_volatge_page(lcd_interface *lcd_t,uint16_t *adc_buf,uint8_t *range);
static void show_beep_page(lcd_interface *lcd_t,uint16_t *adc_buf,uint8_t *range);
static void key_handle(key_handle_t *key_t,lcd_interface *lcd_t,uint8_t *mode_sel,uint8_t *range_sel);
static void process_key_event(void);
static void reset_adc_gather(uint16_t * adc_buf);
static void cal_ui_service(void);

void init_task(void)
{
	register_1_80_lcd(&lcd_p,&lcd_t);
	mode_key = Key_Init(MODE_GPIO_Port,MODE_Pin,GPIO_PIN_RESET);
	key_last_scan_tick = HAL_GetTick();
	lcd_t.lcd_init(&lcd_t);
	lcd_t.lcd_exit_sleep(&lcd_t);
	lcd_fill(&lcd_t,0,0,lcd_t.lcd_p->lcd_height,lcd_t.lcd_p->lcd_width,BLACK);
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_CH_NUM);
	CAL_Init();		/* calibration storage + UART console / BLE channel */
}

/*
 * CAL session banner: redrawn every 250 ms while the console session is
 * alive (page switches clear the panel, so a draw-once flag would go stale).
 */
static void cal_banner(void)
{
	static uint8_t shown = 0U;
	static uint32_t last_draw = 0U;
	uint32_t now = HAL_GetTick();

	if(CAL_Is_Active() || (cal_ui_state != CAL_UI_IDLE))
	{
		shown = 1U;
		if((now - last_draw) >= 250U)
		{
			last_draw = now;
			lcd_showString(&lcd_t,120,20,(uint8_t *)"CAL", YELLOW, BLACK, 16, 0);
		}
	}
	else if(shown)
	{
		shown = 0U;
		lcd_showString(&lcd_t,120,20,(uint8_t *)"   ", BLACK, BLACK, 16, 0);
	}
}

void cycle_task(void)
{
	process_key_event();
	CAL_Service();
	cal_banner();
	if(mode_sel == VOL_MODE){
		show_volatge_page(&lcd_t,&adc_buf[0],&range_sel);
	}
	else if(mode_sel == RES_MODE){
		show_resistance_page(&lcd_t,&adc_buf[0],&range_sel);
	}
	else if(mode_sel == BUZ_MODE){
		show_beep_page(&lcd_t,&adc_buf[0],&range_sel);
	}
	cal_ui_service();
}

/*
 * Non-blocking key event processor.
 *
 * The MODE key triggers EXTI on the falling edge, which only sets
 * key_trigger_bit. The debounce state machine is then driven here at a
 * 10 ms cadence, so the measurement pages keep refreshing while a key
 * event is being resolved (the old implementation blocked in HAL_Delay
 * for up to 5 s).
 *
 * Result mapping:
 *   KeyPress        -> cycle Voltage / Resistance / Continuity
 *   KeyDoublePress  -> reserved (no-op)
 *   KeyLongPress    -> reserved for future features, e.g. calibration
 */
static void process_key_event(void)
{
	uint32_t now;

	if(key_trigger_bit == 0){
		return;
	}
	now = HAL_GetTick();

	if(mode_key.key_state != KEY_NoPress)
	{
		/* Debounce finished: dispatch the event and close the handling. */
		key_handle(&mode_key,&lcd_t,&mode_sel,&range_sel);
		mode_key.key_state = KEY_NoPress;
		key_trigger_bit = 0;
		key_event_start_tick = 0;
		return;
	}

	if(key_event_start_tick == 0)
	{
		/* New event: record when it started. */
		key_event_start_tick = now;
		key_last_scan_tick = now;

		/* The EXTI edge already proves the key went down; if it is
		 * already up again (very short tap), count it as a completed
		 * short press right away. */
		if(HAL_GPIO_ReadPin(mode_key.key_gpio,mode_key.key_pin) != mode_key.key_press_level){
			mode_key.key_state = KeyPress;
		}
		return;
	}

	/* Feed the debounce state machine at a 10 ms cadence. */
	if((now - key_last_scan_tick) >= 10){
		key_last_scan_tick = now;
		Key_Scanf(&mode_key);
	}

	/* Safety net: the key is already released but the debouncer did not
	 * report an event (tap shorter than the 20 ms debounce window).
	 * Complete the press so the button can never get stuck. */
	if((HAL_GPIO_ReadPin(mode_key.key_gpio,mode_key.key_pin) != mode_key.key_press_level)
		&& ((now - key_event_start_tick) >= 50)){
		mode_key.key_state = KeyPress;
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

/*
 * Voltage page: shows the measured input voltage.
 *
 * The analog front-end divides the input and level-shifts it around the
 * 1.65 V reference, so the software subtracts the measured reference
 * (ref165_vol) and scales back by the divider ratio:
 *   0-5V  range: V_in = (vol1 - ref165) * 13.3 / 3.3
 *   5-15V range: V_in = (vol2 - ref165) * 11
 *
 * offset_val is the follower output offset measured with the input
 * shorted to GND (hard-coded 5 mV; a future calibration feature will
 * make it user-adjustable and store it in Flash).
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
		lcd_showString(lcd_t,100,2,(uint8_t *)"0-5V",
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
    	lcd_showString(lcd_t,100,2,(uint8_t *)"5-15V",
    			  				LGRAY, BLACK, 16, 0);
    	if(vol2_val >= ref165_vol){
    		filter_vol = (vol2_val - ref165_vol - offset_val) * 11.0f * 1000;
    		sign = 1;
		}else{
			filter_vol = (ref165_vol - vol2_val + offset_val) * 11.0f * 1000;
			sign = 0;
		}
    }
    /* two-point calibration: publish the raw value, then apply k*x+b */
    {
        uint8_t cidx = (*range == VOL_RANGE_0_5) ? CAL_IDX_VOL_0_5 : CAL_IDX_VOL_5_15;
        float cal_v = Moving_Average(filter_vol);
        CAL_Publish_Raw(cidx, cal_v);
        cal_v = CAL_Apply(cidx, cal_v);
        vin_vol = (uint16_t)(cal_v + (cal_v >= 0.0f ? 0.5f : -0.5f));
    }
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

    if(cal_ui_state == CAL_UI_IDLE){
    lcd_showString(lcd_t,0,110,
                   (uint8_t *)"0-5V<---sw--->5V-15V",
				   DARKGRAY, BLACK, 16, 0);
    }
}

static void show_resistance_page(lcd_interface *lcd_t,uint16_t *adc_buf,uint8_t *range)
{
    /*
     * Resistance page.
     *
     * A constant current is derived from the two reference nodes:
     *   I = (2 * ref165 - ref230) / 1000  [A], about 1.2 mA.
     *
     *   0R-1K    range: R = res1_in / I
     *   1K-100K  range: 10k series resistor, voltage divider formula
     *                    R = 10k * res2_in / (2 * ref165 - res2_in)
     *
     * The range is auto-detected from res1_in and the beeper gives an
     * audible cue near the range limits.
     *
     * Open circuit / overrange shows "OL." instead of a bogus value: with
     * the leads open the constant-current source collapses and the raw
     * division result is garbage (an open would look like a short). OL
     * readings stay out of the moving-average filter and out of the
     * calibration raw cache on purpose.
     */
    char show_data[32];
    uint16_t res_color = 0;
    uint32_t resistance = 0;
    float ref165_vol = 0,res1_in = 0,res2_in = 0,ref230_vol = 0;
    float current_val = 0;
    uint8_t ol_flag = 0;

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

    /* OL guard #0: constant-current source collapsed (leads open),
     * checked before any division uses current_val. */
    if(current_val < CAL_OPEN_I_MIN){
        ol_flag = 1;
    }

    if(*range == RES_RANGE_0_1){
        lcd_showString(lcd_t,100,2,(uint8_t *)"0K---1K",
                        LGRAY, BLACK, 16, 0);
        res_color = WHITE;
        if(!ol_flag && (res1_in >= CAL_RES1_SAT_V)){
            ol_flag = 1;    /* front-end saturated */
        }
        if(!ol_flag){
            resistance = (res1_in / current_val) + 0.5f;
            if(resistance > 3000){
                ol_flag = 1;    /* overrange */
            }
            else if(resistance > 1000){
                /* near the range limit: audible cue (blocking, known gap) */
                HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
                HAL_Delay(50);
                HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
            }
        }
    }else{
        lcd_showString(lcd_t,100,2,(uint8_t *)"1K-100K",
                            LGRAY, BLACK, 16, 0);
        if(!ol_flag && ((res2_in < CAL_RES2_FLOOR_V)
                        || (res2_in >= (ref165_vol * 2.0f)))){
            ol_flag = 1;    /* open circuit / divider midpoint collapsed */
        }
        if(!ol_flag){
            resistance = (10000.0f) * (res2_in / ((ref165_vol * 2.0f) - res2_in))+0.5f;
            if(resistance > 150000){
                ol_flag = 1;    /* overrange */
            }
        }
        if(!ol_flag){
            if (resistance < 47000) {
                res_color = ORANGE;
            } else {
                res_color = YELLOW;
            }
            if(resistance > 100000){
                /* near the range limit: audible cue (blocking, known gap) */
                HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
                HAL_Delay(50);
                HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
            }
        }
    }

    if(ol_flag){
        /* OL: keep the garbage reading out of the filter and of the
         * calibration raw cache, clear the value slot and flag OL.,
         * and make sure the beeper stays off. */
        HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
        lcd_showString(lcd_t,24,48,(uint8_t *)"      ",
                        LGRAY, BLACK, 32, 0);
        lcd_showString(lcd_t,64,48,(uint8_t *)"OL.",
                        LGRAY, BLACK, 32, 0);
        lcd_showString(lcd_t,120,48,(uint8_t *)"R",
                        LGRAY, BLACK, 32, 0);
    }else{
        resistance = Moving_Average_res(resistance);
        /* two-point calibration: publish the raw value, then apply k*x+b */
        {
            uint8_t cidx = (*range == RES_RANGE_0_1) ? CAL_IDX_RES_0_1 : CAL_IDX_RES_1_100;
            float cal_r;
            CAL_Publish_Raw(cidx, (float)resistance);
            cal_r = CAL_Apply(cidx, (float)resistance);
            if(cal_r < 0.0f)
            {
                cal_r = 0.0f;
            }
            resistance = (uint32_t)(cal_r + 0.5f);
        }
        if (resistance < 1000) {
            snprintf(show_data, sizeof(show_data),
                     "%6lu", (unsigned long)resistance);
        } else {
            snprintf(show_data, sizeof(show_data),
                     "%3lu.%02lu",
                     (unsigned long)(resistance / 1000),
                     (unsigned long)((resistance % 1000) / 10));
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
    }

    if(cal_ui_state == CAL_UI_IDLE){
    lcd_showString(lcd_t,0,110,
                   (uint8_t *)"1k-100K<--sw-->0R-1K",
                   DARKGRAY, BLACK, 16, 0);
    }
}



static void show_beep_page(lcd_interface *lcd_t,uint16_t *adc_buf,uint8_t *range)
{
    /*
     * Continuity page: same front-end as the resistance mode, but the
     * buzzer stays on while the measured resistance is below 15 ohms.
     */
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
    /* continuity uses the calibrated 0-1K pair as well */
    {
        float cal_r;
        CAL_Publish_Raw(CAL_IDX_RES_0_1, (float)resistance);
        cal_r = CAL_Apply(CAL_IDX_RES_0_1, (float)resistance);
        if(cal_r < 0.0f)
        {
            cal_r = 0.0f;
        }
        resistance = (uint32_t)(cal_r + 0.5f);
    }
    if(resistance <= 15){
    	snprintf(show_data, sizeof(show_data),
				 "%6lu", (unsigned long)resistance);
		lcd_showString(lcd_t,24,48,(uint8_t *)show_data,
						   res_color, BLACK, 32, 0);
		lcd_showString(lcd_t,120,48,(uint8_t *)"R",
								LGRAY, BLACK, 32, 0);
		HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
    }
    else if (resistance <= 1000) {
		snprintf(show_data, sizeof(show_data),
				 "%6lu", (unsigned long)resistance);
		lcd_showString(lcd_t,24,48,(uint8_t *)show_data,
						   res_color, BLACK, 32, 0);
		lcd_showString(lcd_t,120,48,(uint8_t *)"R",
								LGRAY, BLACK, 32, 0);
		HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
    }
    else{
    	resistance = 0;
    	snprintf(show_data, sizeof(show_data),
    					 "%6lu", (unsigned long)resistance);
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

/* ---------------- onboard calibration UI (long-press entry) ----------------
 * Two-point calibration without a host PC: point 1 is free (shorted probes
 * read 0 V / 0 ohm), point 2 is the reference fixture (CAL_FIXTURE_Y).
 * The measurement page keeps running: it is what publishes the raw value
 * the capture averages. Key mapping while active: click = capture point,
 * long press = abort without saving, double press ignored, mode switching
 * suppressed.
 */
static void cal_ui_show_line(uint16_t y, const char *s, uint16_t color)
{
    char line[17];
    uint8_t i = 0U;

    while((i < 16U) && (s[i] != '\0'))
    {
        line[i] = s[i];
        i++;
    }
    while(i < 16U)
    {
        line[i] = ' ';   /* pad with spaces to wipe what was there */
        i++;
    }
    line[16] = '\0';
    lcd_showString(&lcd_t,0,y,(uint8_t *)line, color, BLACK, 16, 0);
}

static void cal_ui_enter(uint8_t cidx)
{
    /* NOTE: keep the (height, width) argument order, see key_handle. */
    lcd_fill(&lcd_t,0,0,lcd_t.lcd_p->lcd_height,lcd_t.lcd_p->lcd_width,BLACK);
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
    cal_ui_cidx = cidx;
    cal_ui_x1 = 0.0f;
    cal_ui_collecting_x2 = 0U;
    cal_ui_state = CAL_UI_SHORT;
    cal_ui_draw_tick = 0U;   /* force an immediate prompt draw */
}

static void cal_ui_exit(void)
{
    cal_ui_state = CAL_UI_IDLE;
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
    /* NOTE: keep the (height, width) argument order, see key_handle. */
    lcd_fill(&lcd_t,0,0,lcd_t.lcd_p->lcd_height,lcd_t.lcd_p->lcd_width,BLACK);
}

/* short confirmation beep (blocking 30 ms, acceptable for a one-shot cue) */
static void cal_ui_beep(void)
{
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
    HAL_Delay(30);
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
}

static void cal_ui_fail(void)
{
    cal_ui_ok = 0U;
    cal_ui_state = CAL_UI_OK_ERR;
    cal_ui_banner_tick = HAL_GetTick();
}

static void cal_ui_capture_done(float x)
{
    if(!cal_ui_collecting_x2)
    {
        cal_ui_x1 = x;
        cal_ui_state = CAL_UI_REF;   /* step 2: fixture point */
        return;
    }
    /* solve y = k*x + b from (x1, 0) and (x2, CAL_FIXTURE_Y[cidx]) */
    {
        float y2 = CAL_FIXTURE_Y[cal_ui_cidx];
        float dx = x - cal_ui_x1;
        float adx = (dx >= 0.0f) ? dx : -dx;
        float k, b;

        if(adx < CAL_MIN_SPAN[cal_ui_cidx])
        {
            cal_ui_fail();   /* points too close: k would explode */
            return;
        }
        k = y2 / dx;
        b = -k * cal_ui_x1;
        if(CAL_Onboard_Set(cal_ui_cidx, k, b))
        {
            cal_ui_ok = 1U;
            cal_ui_state = CAL_UI_OK_ERR;
            cal_ui_banner_tick = HAL_GetTick();
            cal_ui_beep();
        }
        else
        {
            cal_ui_fail();   /* k/b sanity window or flash write failed */
        }
    }
}

static void cal_ui_key(uint8_t key_state)
{
    if(key_state == KeyPress)
    {
        if((cal_ui_state == CAL_UI_SHORT) || (cal_ui_state == CAL_UI_REF))
        {
            /* start the capture for the current step */
            cal_ui_beep();
            cal_ui_collecting_x2 = (cal_ui_state == CAL_UI_REF) ? 1U : 0U;
            cal_ui_samples = 0U;
            cal_ui_sum = 0.0f;
            cal_ui_step_tick = HAL_GetTick();
            cal_ui_last_sample_tick = cal_ui_step_tick;
            cal_ui_state = CAL_UI_COLLECT;
        }
        /* clicks during COLLECT / OK_ERR are ignored */
    }
    else if(key_state == KeyLongPress)
    {
        cal_ui_exit();   /* abort: leave without saving */
    }
    /* KeyDoublePress: ignored */
}

static void cal_ui_service(void)
{
    float raw = 0.0f;
    uint8_t have_raw;
    uint32_t now = HAL_GetTick();

    if(cal_ui_state == CAL_UI_IDLE)
    {
        return;
    }

    have_raw = CAL_Get_Raw(cal_ui_cidx, &raw);

    if(cal_ui_state == CAL_UI_COLLECT)
    {
        if((now - cal_ui_step_tick) > CAL_UI_TIMEOUT_MS)
        {
            cal_ui_fail();   /* capture timed out (page stopped feeding) */
        }
        else if(have_raw && ((now - cal_ui_last_sample_tick) >= CAL_UI_SAMPLE_MS))
        {
            cal_ui_last_sample_tick = now;
            cal_ui_sum += raw;
            cal_ui_samples++;
            if(cal_ui_samples >= CAL_UI_SAMPLE_N)
            {
                cal_ui_capture_done(cal_ui_sum / (float)CAL_UI_SAMPLE_N);
            }
        }
    }
    else if((cal_ui_state == CAL_UI_OK_ERR)
            && ((now - cal_ui_banner_tick) >= CAL_UI_BANNER_MS))
    {
        cal_ui_exit();
        return;
    }

    if((now - cal_ui_draw_tick) >= 200U)
    {
        const char *prompt;
        uint16_t prompt_color = YELLOW;
        char buf[24];

        cal_ui_draw_tick = now;
        switch(cal_ui_state)
        {
        case CAL_UI_SHORT:
        case CAL_UI_COLLECT:
            prompt = (cal_ui_collecting_x2) ? "CAL2:REF KEY" : "CAL1:SHORT KEY";
            break;
        case CAL_UI_REF:
            prompt = "CAL2:REF KEY";
            break;
        default:
            prompt = cal_ui_ok ? "CAL OK" : "CAL ERR";
            prompt_color = cal_ui_ok ? GREEN : RED;
            break;
        }
        cal_ui_show_line(110, prompt, prompt_color);

        /* live pre-calibration value so the user can see it settle */
        if(have_raw && (cal_ui_state != CAL_UI_OK_ERR))
        {
            if((cal_ui_cidx == CAL_IDX_VOL_0_5) || (cal_ui_cidx == CAL_IDX_VOL_5_15))
            {
                long mv = (long)(raw + ((raw >= 0.0f) ? 0.5f : -0.5f));
                (void)snprintf(buf, sizeof(buf), "x=%s%ld.%03ldV",
                               (mv < 0) ? "-" : "",
                               labs(mv) / 1000L, labs(mv) % 1000L);
            }
            else
            {
                long ohm = (long)(raw + 0.5f);
                (void)snprintf(buf, sizeof(buf), "x=%ldR", ohm);
            }
            cal_ui_show_line(80, buf, WHITE);
        }
    }
}

/*
 * Dispatch a resolved key event.
 * KeyPress cycles the measurement mode; KeyLongPress enters the onboard
 * calibration UI from the voltage/resistance page; double press is a no-op.
 * While the calibration UI is active it owns the key completely: click =
 * capture point, long press = abort, mode switching suppressed.
 */
static void key_handle(key_handle_t *key_t,lcd_interface *lcd_t,uint8_t *mode_sel,uint8_t *range_sel)
{
	if(cal_ui_state != CAL_UI_IDLE)
	{
		cal_ui_key(key_t->key_state);
		return;
	}

	switch(key_t->key_state)
	{
	case KeyPress:
		/* Clear the panel so the next page does not keep leftover pixels.
		 * NOTE: keep the (height, width) argument order as-is: with the
		 * 1.8" LCD in portrait mode (MADCTL with MV set) this transposed
		 * window is what actually covers the whole panel. */
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
	case KeyLongPress:
		/* enter onboard calibration: snapshots the range shown right now
		 * (the buzzer page has no calibration range, ignore it there) */
		if((*mode_sel == VOL_MODE) || (*mode_sel == RES_MODE))
		{
			uint8_t cidx;
			if(*mode_sel == VOL_MODE)
			{
				cidx = (*range_sel == VOL_RANGE_0_5) ? CAL_IDX_VOL_0_5 : CAL_IDX_VOL_5_15;
			}
			else
			{
				cidx = (*range_sel == RES_RANGE_0_1) ? CAL_IDX_RES_0_1 : CAL_IDX_RES_1_100;
			}
			cal_ui_enter(cidx);
		}
		break;
	case KeyDoublePress:
	default:
		break;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == MODE_Pin){
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
