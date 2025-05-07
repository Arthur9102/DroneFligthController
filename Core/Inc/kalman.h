/*
 * kalman.h
 *
 *  Created on: May 7, 2025
 *      Author: Admin
 */

// kalman_filter.h
#ifndef KALMAN_H
#define KALMAN_H

typedef struct {
    float x[4];      // State: [phi, phi_dot, theta, theta_dot]
    float P[4][4];   // Error covariance matrix
    float Q[4][4];   // Process noise covariance
    float R[2][2];   // Measurement noise covariance
    float K[4][2];   // Kalman gain
    float gyro_bias[3]; // Gyroscope bias estimates [p_bias, q_bias, r_bias]
} KalmanEKFFilter;

void KalmanEKF_Init(KalmanEKFFilter *kf);
void KalmanEKF_Update(KalmanEKFFilter *kf, float phi_acc, float theta_acc,
                      float p, float q, float r, float dt, float acc_magnitude);
void KalmanEKF_GetAttitude(KalmanEKFFilter *kf, float *roll, float *pitch,
                           float *roll_rate, float *pitch_rate);
void KalmanEKF_GetGyroBias(KalmanEKFFilter *kf, float *p_bias, float *q_bias, float *r_bias);
void KalmanEKF_Reset(KalmanEKFFilter *kf, float init_roll, float init_pitch);

#endif

