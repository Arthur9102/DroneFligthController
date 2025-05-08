#ifndef CONFIG_H_
#define CONFIG_H_

#include "bmp280.h"
#include "mpu6050.h"

/**
 * @brief Sensor data structure for BMP280 and MPU6050
 */
typedef struct {
    // BMP280 data
    float bmp_temperature; // Temperature (°C)
    float bmp_pressure;    // Pressure (hPa)
    float bmp_altitude;    // Altitude (meters)
    uint8_t bmp_status;    // 1: Success, 0: Failed

    // MPU6050 data
    float mpu_accel_x;     // Acceleration X (g)
    float mpu_accel_y;     // Acceleration Y (g)
    float mpu_accel_z;     // Acceleration Z (g)
    float mpu_gyro_x;      // Gyroscope X (°/s)
    float mpu_gyro_y;      // Gyroscope Y (°/s)
    float mpu_gyro_z;      // Gyroscope Z (°/s)
    float mpu_temperature; // Temperature (°C)
    uint8_t mpu_status;    // 1: Success, 0: Failed
} SensorData;

extern BMP280_HandleTypedef bmp280;
extern MPU6050_HandleTypeDef mpu6050;
extern uint8_t _flag;

/**
 * @brief Initialize BMP280 and MPU6050 sensors
 */
void InitSensor(void);

/**
 * @brief Read data from BMP280 and MPU6050
 * @param data Pointer to SensorData structure to store results
 * @retval 1: At least one sensor read successfully, 0: Both failed
 */
uint8_t ReadSensor(SensorData *data);

/**
 * @brief Calib bias of gryo sensor
 * @param void
 * @retval 
 */
void Calib_gyro();

#endif /* CONFIG_H_ */
