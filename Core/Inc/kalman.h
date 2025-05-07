/**
 * @file kalman.h
 * @brief Header file for standalone Kalman filter implementation for sensor fusion
 * @author Claude
 * @date May 7, 2025
 */

 #ifndef KALMAN_FUSION_H
 #define KALMAN_FUSION_H
 
 #include <stdint.h>
 
 /* Conversion constant */
 #define RAD_TO_DEG 57.295779513082320876798154814105f
 
 /**
  * @brief Kalman filter structure for sensor fusion
  */
 typedef struct {
     float Q_angle;    /* Process noise variance for angle */
     float Q_bias;     /* Process noise variance for bias */
     float R_measure;  /* Measurement noise variance */
     
     float angle;      /* Estimated angle */
     float bias;       /* Estimated gyro bias */
     
     float P[2][2];    /* Error covariance matrix */
 } Kalman_t;
 
 /**
  * @brief Accelerometer data structure
  */
 typedef struct {
     float x;  /* X-axis acceleration in g */
     float y;  /* Y-axis acceleration in g */
     float z;  /* Z-axis acceleration in g */
 } AccelData_t;
 
 /**
  * @brief Gyroscope data structure
  */
 typedef struct {
     float x;  /* X-axis angular rate in degrees/second */
     float y;  /* Y-axis angular rate in degrees/second */
     float z;  /* Z-axis angular rate in degrees/second */
 } GyroData_t;
 
 /**
  * @brief Combined IMU data structure
  */
 typedef struct {
     AccelData_t accel;  /* Accelerometer data */
     GyroData_t gyro;    /* Gyroscope data */
 } IMUData_t;
 
 /**
  * @brief Angle output structure
  */
 typedef struct {
     float roll;        /* Roll angle (filtered) in degrees */
     float pitch;       /* Pitch angle (filtered) in degrees */
     float roll_accel;  /* Roll angle from accelerometer only in degrees */
     float pitch_accel; /* Pitch angle from accelerometer only in degrees */
 } Angles_t;
 
 /* Function prototypes */
 void Kalman_Init(Kalman_t *kalman);
 void Kalman_SetParameters(Kalman_t *kalman, float Q_angle, float Q_bias, float R_measure);
 void Kalman_SetAngle(Kalman_t *kalman, float initialAngle);
 float Kalman_Update(Kalman_t *kalman, float newAngle, float newRate, float dt);
 void Kalman_CalculateAccelAngles(const AccelData_t *accel_data, Angles_t *angles);
 void Kalman_UpdateAttitude(const AccelData_t *accel_data, const GyroData_t *gyro_data,
                           Kalman_t *roll_kalman, Kalman_t *pitch_kalman,
                           Angles_t *angles, float dt);
 void Kalman_ProcessSensorFusion(const IMUData_t *imu_data, 
                                Kalman_t *roll_kalman, Kalman_t *pitch_kalman,
                                Angles_t *angles, float dt);
 
 #endif /* KALMAN_FUSION_H */