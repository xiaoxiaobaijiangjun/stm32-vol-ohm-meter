/*
 * mid_calib.h - two-point calibration per measurement range + UART console
 * + BLE report, all on USART1 (see hw_uart1.h).
 *
 * Model: every range displays  y = k * x + b  where x is the pre-calibration
 * value the measurement page already computes (node voltage formula result,
 * mV for voltage ranges; computed ohms for resistance ranges) and k/b default
 * to the identity (k=1, b=0). A first-order fit absorbs the systematic gain
 * and offset errors of the whole analog chain (divider ratio, VREF, reference
 * node, op-amp offset, sense resistor tolerance).
 *
 * The two-point SOLVE runs on the host (tools/cal_host.py): the wizard takes
 * two CAP captures per range plus the true values read from the reference
 * multimeter, computes k=(y2-y1)/(x2-x1) and b=y1-k*x1, then SETs and SAVEs
 * the pair. The firmware only stores and applies.
 */

#ifndef MID_CALIB_H_
#define MID_CALIB_H_

#include <stdint.h>

#define CAL_IDX_VOL_0_5     0U   /* voltage, 0-5V   range, x in mV   */
#define CAL_IDX_VOL_5_15    1U   /* voltage, 5-15V  range, x in mV   */
#define CAL_IDX_RES_0_1     2U   /* resistance 0-1K range, x in ohm  */
#define CAL_IDX_RES_1_100   3U   /* resistance 1-100K range, x in ohm*/
#define CAL_RANGE_COUNT     4U

void    CAL_Init(void);                        /* uart init + flash load    */
void    CAL_Service(void);                     /* console + 1 Hz report     */
void    CAL_Publish_Raw(uint8_t idx, float x); /* pages feed pre-cal values */
float   CAL_Apply(uint8_t idx, float x);       /* apply k*x+b (identity-safe)*/
uint8_t CAL_Is_Active(void);                   /* console session running   */
uint8_t CAL_Report_Enabled(void);              /* BLE 1 Hz report on        */

#endif /* MID_CALIB_H_ */
