/*
 * mid_kalman.c
 *
 *  Created on: 2026年1月16日
 *      Author: jlc
 */
#include "mid_kalman.h"

void Kalman_Init(Kalman_t *kfp, float q, float r)
{
    kfp->X = 0.0f;
    kfp->P = 1.0f;
    kfp->Q = q;
    kfp->R = r;
    kfp->K = 0.0f;
}

float Kalman_Filter(Kalman_t *kfp, float measurement)
{
    /* 1. forecast */
    kfp->P = kfp->P + kfp->Q;

    /* 2. Calculate the Kalman gain */
    kfp->K = kfp->P / (kfp->P + kfp->R);

    /* 3. Update the estimated value */
    kfp->X = kfp->X + kfp->K * (measurement - kfp->X);

    /* 4. Update error covariance */
    kfp->P = (1 - kfp->K) * kfp->P;

    return kfp->X;
}

void Kalman_Reset(Kalman_t *kfp, float value)
{
	kfp->X = value;
	kfp->P = 1.0f;
}


