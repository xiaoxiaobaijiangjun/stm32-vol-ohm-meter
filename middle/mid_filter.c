/*
 * my_filter.c
 *
 *  Created on: 2026年2月9日
 *      Author: jlc
 */
#include "mid_filter.h"

#define FILTER_N 16
#define FILTER_RES_N 16

static uint16_t filter_buf[FILTER_N] = {0};
static uint32_t filter_sum = 0;
static uint8_t filter_index = 0;

static uint32_t filter_buf_res[FILTER_RES_N] = {0};
static uint32_t filter_sum_res = 0;
static uint8_t filter_index_res = 0;

uint16_t Moving_Average(uint16_t new_value)
{
    filter_sum -= filter_buf[filter_index];  // Subtract the old value

    filter_buf[filter_index] = new_value;    // Store new value

    filter_sum += new_value;                 // Add new values

    filter_index++;
    if(filter_index >= FILTER_N)
        filter_index = 0;

    return filter_sum / FILTER_N;
}

uint32_t Moving_Average_res(uint32_t new_value)
{
    filter_sum_res -= filter_buf_res[filter_index_res];  // Subtract the old value

    filter_buf_res[filter_index_res] = new_value;    // Store new value

    filter_sum_res += new_value;                 // Add new values

    filter_index_res++;
    if(filter_index_res >= FILTER_RES_N)
    	filter_index_res = 0;

    return filter_sum_res / FILTER_RES_N;
}

