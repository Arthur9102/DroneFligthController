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
#include "main.h"
#include <math.h>
//#include <stdio.h>
//#include <string.h>

#define M_PI 3.14159265358979323846
#define MAG_AVAILABLE 0
#define ABS(x) ((x) > 0 ? (x) : -(x))
BMP280_HandleTypedef bmp280;
MPU6050_HandleTypeDef mpu6050;

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c3;
uint8_t _flag = 0;

extern float gyro_x_bias,gyro_y_bias,gyro_z_bias;

// Các cấu hình cho expo mapping
#define EXPO_ROLL 1       // Hệ số expo cho roll
#define EXPO_PITCH 1       // Hệ số expo cho pitch
#define EXPO_YAW 0.6             // Hệ số expo cho yaw
#define EXPO_ALTITUDE 1.0        // Giữ tuyến tính cho độ cao

// Các cấu hình slew rate
#define SLEW_RATE_MAX 100.0      // Tốc độ thay đổi tối đa của đầu ra (đơn vị/chu kỳ)

// Các biến lưu giá trị đầu ra trước đó cho slew rate limiting
static double prev_motor1_output = 0.0;
static double prev_motor2_output = 0.0;
static double prev_motor3_output = 0.0;
static double prev_motor4_output = 0.0;

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
extern uint16_t _throttle_1 ;
extern uint16_t _throttle_2 ;
extern uint16_t _throttle_3 ;
extern uint16_t _throttle_4 ;
void do_motor3(int16_t t) { 
    TIM5->CCR2 = 1100 + CONSTRAIN(t,0,1000) * 2.02;
    _throttle_3 = TIM5->CCR2;
} //limit 85% motor
void do_motor1(int16_t t) { 
    TIM5->CCR1 = 1100 + CONSTRAIN(t, 0, 1000) * 2.02;
    _throttle_1 = TIM5->CCR1;
} //limit 85% motor
void do_motor2(int16_t t) { 
    TIM5->CCR3 = 1100 + CONSTRAIN(t, 0, 1000) * 2.02; 
    _throttle_2 = TIM5->CCR3;
} //limit 85% motor
void do_motor4(int16_t t) { 
    TIM5->CCR4 = 1100 + CONSTRAIN(t, 0, 1000) * 2.02; 
    _throttle_4 = TIM5->CCR4;
} //limit 85% motor

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

#define MAX_INPUT_RANGE 500.0
#define NORM_FACTOR (1.0/MAX_INPUT_RANGE)

// Hàm áp dụng expo mapping
double apply_expo(float input, float expo_factor) {
    if (expo_factor == 1.0) return input;  // Linear case
    if (input == 0.0) return 0.0;  // Optimization

    double sign = (input >= 0.0) ? 1.0 : -1.0;
    double abs_input = fabs(input);
    double normalized = abs_input * NORM_FACTOR;
    
    // Prevent pow() domain error
    if (normalized > 1.0) normalized = 1.0;
    
    return sign * pow(normalized, expo_factor) * MAX_INPUT_RANGE;
}

// Hàm áp dụng deadband
float apply_deadband(float input, float deadband) {
    if (ABS(input) < deadband) {
        return 0.0;
    } else {
        double sign = (input > 0.0) ? 1.0 : -1.0;
        return sign * (ABS(input) - deadband);
    }
}

// Hàm áp dụng slew rate limiting
float apply_slew_rate(float target, float current, float max_rate) {
    if (max_rate <= 0.0f) return current;  // Invalid rate protection

    float delta = target - current;
    if (fabs(delta) <= max_rate) return target;
    
    return current + (delta > 0.0 ? max_rate : -max_rate);
}

#define MOTOR_MIN_OUTPUT 0.0
#define MOTOR_MAX_OUTPUT 1000.0
#define MIXING_SCALE 1.0f    // Scale factor for roll/pitch/yaw mixing

/**
 * @brief Calculate motor outputs based on control inputs
 * @param z_output Throttle input (0 to 1000)
 * @param roll_output Roll control input (-250 to 250)
 * @param pitch_output Pitch control input (-250 to 250)
 * @param yaw_output Yaw control input (-250 to 250)
 * @param motor1 Pointer to motor1 output (Front Right)
 * @param motor2 Pointer to motor2 output (Front Left)
 * @param motor3 Pointer to motor3 output (Back Right)
 * @param motor4 Pointer to motor4 output (Back Left)
 */
void calculate_motor_outputs(float z_output, float roll_output, float pitch_output, float yaw_output,
                           float *motor1, float *motor2, float *motor3, float *motor4) {
    // Input validation
    if (!motor1 || !motor2 || !motor3 || !motor4) {
        return;
    }

    // // Apply expo curves with mixing scale
    // float expo_roll = apply_expo(roll_output, EXPO_ROLL) * MIXING_SCALE;
    // float expo_pitch = apply_expo(pitch_output, EXPO_PITCH) * MIXING_SCALE;
    // float expo_yaw = 0;

    // Calculate base motor outputs
    float throttle_1 = z_output - roll_output + pitch_output + yaw_output;  // Front Right
    float throttle_2 = z_output + roll_output - pitch_output + yaw_output;  // Front Left
    float throttle_3 = z_output + roll_output + pitch_output - yaw_output;  // Back Right
    float throttle_4 = z_output - roll_output - pitch_output - yaw_output;  // Back Left

//    // Find maximum output for scaling
//    float max_output = throttle_1;
//    max_output = (throttle_2 > max_output) ? throttle_2 : max_output;
//    max_output = (throttle_3 > max_output) ? throttle_3 : max_output;
//    max_output = (throttle_4 > max_output) ? throttle_4 : max_output;

//    // Scale outputs if necessary
//    if (max_output > MOTOR_MAX_OUTPUT) {
//        float scale = MOTOR_MAX_OUTPUT / max_output;
//        throttle_1 *= scale;
//        throttle_2 *= scale;
//        throttle_3 *= scale;
//        throttle_4 *= scale;
//     }

    // Apply slew rate limiting
    *motor1 = throttle_1;
    *motor2 = throttle_2;
    *motor3 = throttle_3;
    *motor4 = throttle_4;

    // Update previous values
    // prev_motor1_output = *motor1;
    // prev_motor2_output = *motor2;
    // prev_motor3_output = *motor3;
    // prev_motor4_output = *motor4;
}




