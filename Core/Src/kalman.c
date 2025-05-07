#include "kalman.h"
#include <math.h>
#include <string.h> // For memset

#define DT 0.01f     // Default sampling time (10ms = 100Hz)
#define EPSILON 1e-7f // Small value to avoid division by zero


// Safe inverse for 2x2 matrix
static void matrix_inverse_2x2(float in[2][2], float out[2][2]) {
    float det = in[0][0] * in[1][1] - in[0][1] * in[1][0];

    // Protection against singular matrix
    if (fabsf(det) < EPSILON) {
        det = (det < 0) ? -EPSILON : EPSILON;
    }

    float inv_det = 1.0f / det;
    out[0][0] =  inv_det * in[1][1];
    out[0][1] = -inv_det * in[0][1];
    out[1][0] = -inv_det * in[1][0];
    out[1][1] =  inv_det * in[0][0];
}

// Clear small values to avoid numerical instability
static void clean_small_values(float *value) {
    if (fabsf(*value) < EPSILON) {
        *value = 0.0f;
    }
}

// Initialize Kalman EKF
void KalmanEKF_Init(KalmanEKFFilter *kf) {
    // Zero out all matrices
    memset(kf, 0, sizeof(KalmanEKFFilter));

    // Initialize error covariance matrix P with high initial uncertainty
    for (int i = 0; i < 4; i++) {
        kf->P[i][i] = 10.0f;
    }

    // Initialize process noise Q
    for (int i = 0; i < 4; i++) {
        kf->Q[i][i] = (i % 2 == 0) ? 0.0001f : 0.01f; // Smaller for angles, larger for rates
    }

    // Initialize measurement noise covariance R
    kf->R[0][0] = 0.03f; // Roll measurement noise
    kf->R[1][1] = 0.03f; // Pitch measurement noise

    // Initialize gyro bias estimates to zero
    kf->gyro_bias[0] = 0.0f;
    kf->gyro_bias[1] = 0.0f;
    kf->gyro_bias[2] = 0.0f;

}

// Constrain pitch to avoid gimbal lock
static float constrainPitch(float theta) {
    if (theta > M_PI/2.0f - EPSILON) {
        theta = M_PI/2.0f - EPSILON;
    } else if (theta < -M_PI/2.0f + EPSILON) {
        theta = -M_PI/2.0f + EPSILON;
    }
    return theta;
}

// Update the filter with new measurements
void KalmanEKF_Update(KalmanEKFFilter *kf, float phi_acc, float theta_acc,
                      float p, float q, float r, float dt, float acc_magnitude) {
    // Input validation
    if (dt <= 0.0f || isnan(phi_acc) || isnan(theta_acc) || isnan(p) ||
        isnan(q) || isnan(r) || isnan(acc_magnitude)) {
        return; // Skip update
    }

    // Apply gyro bias correction
    float p_corrected = p - kf->gyro_bias[0];
    float q_corrected = q - kf->gyro_bias[1];
    float r_corrected = r - kf->gyro_bias[2];

    // Current state estimates
    float phi_hat = kf->x[0];
    float phi_dot_hat = kf->x[1];
    float theta_hat = kf->x[2];
    float theta_dot_hat = kf->x[3];

    // Constrain pitch to avoid gimbal lock
    // theta_hat = constrainPitch(theta_hat);

    // Calculate Euler angle derivatives
    float phi_dot = p_corrected + sinf(phi_hat) * tanf(theta_hat) * q_corrected +
                    cosf(phi_hat) * tanf(theta_hat) * r_corrected;
    float theta_dot = cosf(phi_hat) * q_corrected - sinf(phi_hat) * r_corrected;

    // Adaptive process noise based on acceleration magnitude
    float original_Q[4][4];
    memcpy(original_Q, kf->Q, sizeof(original_Q));
    float accel_scale = 1.0f;
    if (acc_magnitude > 9.81f * 1.2f) { // More than 1.2G
        accel_scale = fminf(5.0f, acc_magnitude / 9.81f);
        for (int i = 0; i < 4; i++) {
            kf->Q[i][i] *= (i % 2 == 0) ? accel_scale : 1.0f;
        }
    }

    // Predict state
    float x_pred[4];
    x_pred[0] = phi_hat + dt * phi_dot;
    x_pred[1] = phi_dot_hat + dt * phi_dot;
    x_pred[2] = theta_hat + dt * theta_dot;
    x_pred[3] = theta_dot_hat + dt * theta_dot;
    x_pred[2] = constrainPitch(x_pred[2]);

    // State transition matrix A
    float A[4][4] = {{1.0f, dt, 0.0f, 0.0f},
                     {0.0f, 1.0f, 0.0f, 0.0f},
                     {0.0f, 0.0f, 1.0f, dt},
                     {0.0f, 0.0f, 0.0f, 1.0f}};

    // Predict error covariance: P = A*P*A' + Q
    float P_temp[4][4] = {0};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                P_temp[i][j] += A[i][k] * kf->P[k][j];
            }
        }
    }

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            kf->P[i][j] = kf->Q[i][j];
            for (int k = 0; k < 4; k++) {
                kf->P[i][j] += P_temp[i][k] * A[j][k];
            }
            clean_small_values(&kf->P[i][j]);
        }
    }

    // Restore original Q
    memcpy(kf->Q, original_Q, sizeof(original_Q));

    // Measurement matrix C
    float C[2][4] = {{1.0f, 0.0f, 0.0f, 0.0f},
                     {0.0f, 0.0f, 1.0f, 0.0f}};

    // Measurement vector z
    float z[2] = {phi_acc, theta_acc};

    // Compute C*P*C' + R
    float temp[2][4] = {0};
    float CPCt[2][2] = {0};
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                temp[i][j] += C[i][k] * kf->P[k][j];
            }
        }
    }
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            for (int k = 0; k < 4; k++) {
                CPCt[i][j] += temp[i][k] * C[j][k];
            }
            CPCt[i][j] += kf->R[i][j];
        }
    }

    // Compute Kalman gain: K = P*C'*(C*P*C' + R)^(-1)
    float inv_CPCt[2][2] = {0};
    matrix_inverse_2x2(CPCt, inv_CPCt);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 2; j++) {
            kf->K[i][j] = 0.0f;
            for (int k = 0; k < 2; k++) {
                for (int m = 0; m < 4; m++) {
                    kf->K[i][j] += kf->P[i][m] * C[k][m] * inv_CPCt[k][j];
                }
            }
        }
    }

    // Update state: x = x_pred + K*(z - C*x_pred)
    float residual[2] = {z[0] - x_pred[0], z[1] - x_pred[2]};
    for (int i = 0; i < 4; i++) {
        kf->x[i] = x_pred[i];
        for (int j = 0; j < 2; j++) {
            kf->x[i] += kf->K[i][j] * residual[j];
        }
        clean_small_values(&kf->x[i]);
    }

    // Update covariance: P = (I - K*C)*P
    float KC[4][4] = {0};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 2; k++) {
                KC[i][j] += kf->K[i][k] * C[k][j];
            }
        }
    }
    float new_P[4][4] = {0};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            new_P[i][j] = kf->P[i][j];
            for (int k = 0; k < 4; k++) {
                new_P[i][j] -= KC[i][k] * kf->P[k][j];
            }
            clean_small_values(&new_P[i][j]);
        }
    }
    memcpy(kf->P, new_P, sizeof(new_P));
}

// Get current attitude estimates and rates
void KalmanEKF_GetAttitude(KalmanEKFFilter *kf, float *roll, float *pitch,
                           float *roll_rate, float *pitch_rate) {
    *roll = kf->x[0];
    *pitch = kf->x[2];
    *roll_rate = kf->x[1];
    *pitch_rate = kf->x[3];
}

// Get current gyro bias estimates
void KalmanEKF_GetGyroBias(KalmanEKFFilter *kf, float *p_bias, float *q_bias, float *r_bias) {
    *p_bias = kf->gyro_bias[0];
    *q_bias = kf->gyro_bias[1];
    *r_bias = kf->gyro_bias[2];
}

// Reset the filter
void KalmanEKF_Reset(KalmanEKFFilter *kf, float init_roll, float init_pitch) {
    // Set initial attitude
    kf->x[0] = init_roll;
    kf->x[2] = init_pitch;

    // Reset rates to zero
    kf->x[1] = 0.0f;
    kf->x[3] = 0.0f;

    // Keep existing bias estimates

    // Reset covariance to high uncertainty
    memset(kf->P, 0, sizeof(kf->P));
    for (int i = 0; i < 4; i++) {
        kf->P[i][i] = 10.0f;
    }

}
