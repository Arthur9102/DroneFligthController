/**
 * @file kalman.c
 * @brief Standalone Kalman filter implementation for sensor fusion
 * @author Ho Quang DUng
 * @date May 7, 2025
 */

 #include "kalman.h"
 #include <math.h>
 
 /**
  * @brief Initialize a Kalman filter structure with default parameters
  * 
  * @param kalman Pointer to Kalman filter structure
  */
 void Kalman_Init(Kalman_t *kalman) {
     /* Set process and measurement noise parameters */
     kalman->Q_angle = 0.001f;    // Process noise variance for angle
     kalman->Q_bias = 0.003f;     // Process noise variance for bias
     kalman->R_measure = 0.03f;   // Measurement noise variance
     
     /* Initialize state variables */
     kalman->angle = 0.0f;        // Initial angle estimate
     kalman->bias = 0.0f;         // Initial bias estimate
     
     /* Initialize error covariance matrix */
     kalman->P[0][0] = 1.0f;
     kalman->P[0][1] = 0.0f;
     kalman->P[1][0] = 0.0f;
     kalman->P[1][1] = 1.0f;
 }
 
 /**
  * @brief Set specific Kalman filter parameters
  * 
  * @param kalman Pointer to Kalman filter structure
  * @param Q_angle Process noise variance for angle
  * @param Q_bias Process noise variance for bias
  * @param R_measure Measurement noise variance
  */
 void Kalman_SetParameters(Kalman_t *kalman, float Q_angle, float Q_bias, float R_measure) {
     kalman->Q_angle = Q_angle;
     kalman->Q_bias = Q_bias;
     kalman->R_measure = R_measure;
 }
 
 /**
  * @brief Set the initial angle for the Kalman filter
  * 
  * @param kalman Pointer to Kalman filter structure
  * @param initialAngle Initial angle value in degrees
  */
 void Kalman_SetAngle(Kalman_t *kalman, float initialAngle) {
     kalman->angle = initialAngle;
 }
 
 /**
  * @brief Kalman filter update function - performs prediction and correction steps
  * 
  * @param kalman Pointer to Kalman filter structure
  * @param newAngle Angle measured from accelerometer (degrees)
  * @param newRate Angular rate from gyroscope (degrees/second)
  * @param dt Time step (seconds)
  * @return float Updated angle estimate (degrees)
  */
 float Kalman_Update(Kalman_t *kalman, float newAngle, float newRate, float dt) {
     /* Step 1: Predict */
     /* Project the state ahead using gyro data */
     float rate = newRate - kalman->bias;
     kalman->angle += dt * rate;
     
     /* Project the error covariance matrix ahead */
     kalman->P[0][0] += dt * (dt * kalman->P[1][1] - kalman->P[0][1] - kalman->P[1][0] + kalman->Q_angle);
     kalman->P[0][1] -= dt * kalman->P[1][1];
     kalman->P[1][0] -= dt * kalman->P[1][1];
     kalman->P[1][1] += kalman->Q_bias * dt;
     
     /* Step 2: Measurement Update (Correction) */
     /* Compute the Kalman gain */
     float S = kalman->P[0][0] + kalman->R_measure;  /* Innovation covariance */
     float K[2];  /* Kalman gain */
     K[0] = kalman->P[0][0] / S;
     K[1] = kalman->P[1][0] / S;
     
     /* Update estimate with measurement */
     float y = newAngle - kalman->angle;  /* Innovation/measurement residual */
     kalman->angle += K[0] * y;
     kalman->bias += K[1] * y;
     
     /* Update the error covariance matrix */
     float P00_temp = kalman->P[0][0];
     float P01_temp = kalman->P[0][1];
     
     kalman->P[0][0] -= K[0] * P00_temp;
     kalman->P[0][1] -= K[0] * P01_temp;
     kalman->P[1][0] -= K[1] * P00_temp;
     kalman->P[1][1] -= K[1] * P01_temp;
     
     return kalman->angle;
 }
 
 /**
  * @brief Calculate roll and pitch angles from accelerometer data
  * 
  * @param accel_data Pointer to structure containing accelerometer data (g)
  * @param angles Pointer to structure to store calculated angles
  */
 void Kalman_CalculateAccelAngles(const AccelData_t *accel_data, Angles_t *angles) {
     /* Calculate roll angle (rotation around X-axis) */
     angles->roll_accel = atan2f(accel_data->y, accel_data->z) * RAD_TO_DEG;
     
     /* Calculate pitch angle (rotation around Y-axis) */
     angles->pitch_accel = atan2f(-accel_data->x, sqrtf(accel_data->y * accel_data->y + 
                          accel_data->z * accel_data->z)) * RAD_TO_DEG;
 }
 
 /**
  * @brief Update both roll and pitch Kalman filters with new sensor data
  * 
  * @param accel_data Pointer to structure containing accelerometer data (g)
  * @param gyro_data Pointer to structure containing gyroscope data (degrees/s)
  * @param roll_kalman Pointer to roll Kalman filter structure
  * @param pitch_kalman Pointer to pitch Kalman filter structure
  * @param angles Pointer to structure to store updated angles
  * @param dt Time step (seconds)
  */
 void Kalman_UpdateAttitude(const AccelData_t *accel_data, const GyroData_t *gyro_data,
                           Kalman_t *roll_kalman, Kalman_t *pitch_kalman,
                           Angles_t *angles, float dt) {
     /* Calculate angles from accelerometer */
     Kalman_CalculateAccelAngles(accel_data, angles);
     
     /* Update roll Kalman filter */
     angles->roll = Kalman_Update(roll_kalman, angles->roll_accel, gyro_data->x, dt);
     
     /* Update pitch Kalman filter */
     angles->pitch = Kalman_Update(pitch_kalman, angles->pitch_accel, gyro_data->y, dt);
 }
 
 /**
  * @brief Complete sensor fusion processing pipeline
  * 
  * @param imu_data Pointer to structure containing IMU sensor data
  * @param roll_kalman Pointer to roll Kalman filter structure
  * @param pitch_kalman Pointer to pitch Kalman filter structure
  * @param dt Time step (seconds)
  */
 void Kalman_ProcessSensorFusion(const IMUData_t *imu_data, 
                                Kalman_t *roll_kalman, Kalman_t *pitch_kalman,
                                Angles_t *angles, float dt) {
     /* Process accelerometer data to get roll and pitch angles */
     Kalman_CalculateAccelAngles(&imu_data->accel, angles);
     
     /* Apply Kalman filter for roll angle */
     angles->roll = Kalman_Update(roll_kalman, angles->roll_accel, imu_data->gyro.x, dt);
     
     /* Apply Kalman filter for pitch angle */
     angles->pitch = Kalman_Update(pitch_kalman, angles->pitch_accel, imu_data->gyro.y, dt);
 }