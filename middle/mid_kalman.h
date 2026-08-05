/*
 * mid_kalman.h
 *
 *  Created on: 2026年1月16日
 *      Author: jlc
 */

#ifndef MID_KALMAN_H_
#define MID_KALMAN_H_

typedef struct
{
    float X;   // Estimated state value
    float P;   // Estimated error covariance
    float Q;   // Process noise
    float R;   // measurement noise
    float K;   // kalman gain
} Kalman_t;

void Kalman_Init(Kalman_t *kfp, float q, float r);
float Kalman_Filter(Kalman_t *kfp, float measurement);
void Kalman_Reset(Kalman_t *kfp, float value);

#endif /* MID_KALMAN_H_ */
