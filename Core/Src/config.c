/**
 * @file    config.c
 * @brief   Sensor initialization and reading for BMP280 and MPU6050
 * @author  Ho Quang Dung
 * @date    2025-05-07
 * @version v1.0
 * @par Copyright
 *          (C) 2025 Ho Quang Dung. All rights reserved.
 * @par History
 *          1: Created
 * @par Reference
 *          BMP280 and MPU6050 datasheets, STM32 HAL Library
 */

#include "config.h"
#include "cmsis_os.h"

#define M_PI 3.14159265358979323846
#define MAG_AVAILABLE 0

BMP280_HandleTypedef bmp280;
MPU6050_HandleTypeDef mpu6050;

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c3;
uint8_t _flag = 0;

extern float gyro_x_bias,gyro_y_bias,gyro_z_bias;

/**
 * @brief Initialize BMP280 and MPU6050 sensors
 * @note  Sets _flag bits:
 *        - Bit 0: BMP280 success
 *        - Bit 1: MPU6050 success
 *        - Bit 4: BMP280 error
 *        - Bit 5: MPU6050 error
 */
void InitSensor(void) {
    // --- BMP280 ---
   bmp280.hi2c = &hi2c3;
   bmp280.dev_addr = 0xEC; // 0x76 << 1 (SDO = GND)
   bmp280.timeout = 100; // 100ms timeout
   if (BMP280_Init(&bmp280)) {
       _flag |= 0x01; // Bit 0: BMP280 OK
   } else {
       _flag |= 0x10; // Bit 4: BMP280 error
   }
//
//    // Short delay to stabilize I2C bus
//    HAL_Delay(10);

    // --- MPU6050 ---
    mpu6050.hi2c = &hi2c1;
    mpu6050.dev_addr = MPU6050_ADDRESS; // 0x68 (AD0 = GND)
    mpu6050.timeout = 100; // 100ms timeout
    if (MPU6050_Init(&mpu6050)) {
        _flag |= 0x02; // Bit 1: MPU6050 OK
    } else {
        _flag |= 0x20; // Bit 5: MPU6050 error
    }
}

/**
 * @brief Read data from BMP280 and MPU6050
 * @param data Pointer to SensorData structure to store results
 * @retval 1: At least one sensor read successfully, 0: Both failed
 */
uint8_t ReadSensor(SensorData *data) {
    uint8_t success = 0;

    // Initialize status
    data->bmp_status = 0;
    data->mpu_status = 0;

    // Read BMP280 if initialized successfully
    if (_flag & 0x01) {
        float temperature, pressure;
        if (BMP280_ReadTemperaturePressure(&bmp280, &temperature, &pressure)) {
            data->bmp_temperature = temperature;
            data->bmp_pressure = pressure;
            data->bmp_altitude = BMP280_CalculateAltitude(pressure, bmp280.P0);
            data->bmp_status = 1;
            success = 1;
        }
    }

    // Read MPU6050 if initialized successfully
    if (_flag & 0x02) {
        MPU6050_RawData raw_data;
        MPU6050_ConvertedData conv_data;
        if (MPU6050_ReadRawData(&mpu6050, &raw_data)) {
            MPU6050_ConvertData(&mpu6050, &raw_data, &conv_data);
            data->mpu_accel_x = conv_data.accel_x;
            data->mpu_accel_y = conv_data.accel_y;
            data->mpu_accel_z = conv_data.accel_z;
            data->mpu_gyro_x = conv_data.gyro_x;
            data->mpu_gyro_y = conv_data.gyro_y;
            data->mpu_gyro_z = conv_data.gyro_z;
            data->mpu_temperature = conv_data.temp;
            data->mpu_status = 1;
            success = 1;
        }
    }

    return success;
}

void Calib_gyro(){
    MPU6050_RawData raw_data;
      int32_t gyro_x_sum = 0, gyro_y_sum = 0, gyro_z_sum = 0;
      const int samples = 200; // Thu thập 200 mẫu
      for (int i = 0; i < samples; i++) {
        if (MPU6050_ReadRawData(&mpu6050, &raw_data)) {
          gyro_x_sum += raw_data.gyro_x;
          gyro_y_sum += raw_data.gyro_y;
          gyro_z_sum += raw_data.gyro_z;
        }
        osDelay(10);
      }
      gyro_x_bias = (float)(gyro_x_sum / samples) * (1.0f / 16.4f) * M_PI / 180.0f; // Chuyển LSB sang rad/s
      gyro_y_bias = (float)(gyro_y_sum / samples) * (1.0f / 16.4f) * M_PI / 180.0f;
      gyro_z_bias = (float)(gyro_z_sum / samples) * (1.0f / 16.4f) * M_PI / 180.0f;
}

void do_motor1(int16_t t) { TIM2->CCR1 = 1000 + 0.85*CONSTRAIN(t, 0, 1000); } //limit 85% motor 
void do_motor2(int16_t t) { TIM5->CCR2 = 6250 + 0.85*CONSTRAIN(t, 0, 1000) * 6.25; } //limit 85% motor 
void do_motor3(int16_t t) { TIM5->CCR3 = 6250 + 0.85*CONSTRAIN(t, 0, 1000) * 6.25; } //limit 85% motor 
void do_motor4(int16_t t) { TIM5->CCR4 = 6250 + 0.85*CONSTRAIN(t, 0, 1000) * 6.25; } //limit 85% motor 
// TIM5 -> CCR2 = 6250 + (ibus.left_horizontal - 1000) * 6.25;
  	  // TIM5 -> CCR3 = 6250 + (ibus.left_horizontal - 1000) * 6.25;
  	  // TIM5 -> CCR4 = 6250 + (ibus.left_horizontal - 1000) * 6.25;
/* Function to estimate yaw using complementary filter */
float estimate_yaw_complementary(float gyro_yaw_rate, float mag_yaw, float dt, float *yaw_state) {
    // 1. Integrate gyro rate to get change in angle
    float gyro_delta = gyro_yaw_rate * dt;
    
    // 2. Apply complementary filter
    if (!MAG_AVAILABLE) {
        // No magnetometer data available - only use gyro
        *yaw_state += gyro_delta;
    } else {
        // Fuse gyro and magnetometer data
        // High-pass filter on gyro (short-term accuracy)
        // Low-pass filter on magnetometer (long-term stability)
        *yaw_state = YAW_CF_ALPHA * (*yaw_state + gyro_delta) + 
                     (1.0f - YAW_CF_ALPHA) * mag_yaw;
    }
    
    // 3. Normalize yaw to range [-π, π]
    while (*yaw_state > M_PI) *yaw_state -= 2.0f * M_PI;
    while (*yaw_state < -M_PI) *yaw_state += 2.0f * M_PI;
    
    return *yaw_state;
}
