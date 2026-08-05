/*
 * mid_filter.c
 *
 * Moving-average filters used by the measurement pages.
 *
 *  Created on: 2026-02-09
 *      Author: xiaoxiaobaijiangjun
 */
#include "mid_filter.h"

#define FILTER_N     16
#define FILTER_RES_N 16

/* Voltage filter: float samples (mV), keeps sub-millivolt resolution. */
static float filter_buf[FILTER_N] = {0.0f};
static float filter_sum = 0.0f;
static uint8_t filter_index = 0;

/* Resistance filter: integer samples (ohms). */
static uint32_t filter_buf_res[FILTER_RES_N] = {0};
static uint32_t filter_sum_res = 0;
static uint8_t filter_index_res = 0;

float Moving_Average(float new_value)
{
    filter_sum -= filter_buf[filter_index];   /* drop the oldest sample */

    filter_buf[filter_index] = new_value;     /* store the new sample   */

    filter_sum += new_value;

    filter_index++;
    if(filter_index >= FILTER_N)
        filter_index = 0;

    return filter_sum / FILTER_N;
}

uint32_t Moving_Average_res(uint32_t new_value)
{
    filter_sum_res -= filter_buf_res[filter_index_res];

    filter_buf_res[filter_index_res] = new_value;

    filter_sum_res += new_value;

    filter_index_res++;
    if(filter_index_res >= FILTER_RES_N)
    	filter_index_res = 0;

    return filter_sum_res / FILTER_RES_N;
}

