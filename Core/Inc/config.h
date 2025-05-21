#ifndef CONFIG_H_
#define CONFIG_H_

#include "bmp280.h"
#include "mpu6050.h"

#define CONSTRAIN(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

#define YAW_CF_ALPHA 0.98f  // Complementary filter coefficient (gyro trust factor)
#define SAMPLING_PERIOD 0.01f  // 10ms (100Hz) sampling rate

#define FLAGS_PID							0x00000001U
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
}SensorData;

typedef struct {
    float roll;
    float pitch;
}Data_uart;

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

void do_motor1(int16_t t);
void do_motor2(int16_t t);
void do_motor3(int16_t t);
void do_motor4(int16_t t);


float estimate_yaw_complementary(float gyro_yaw_rate, float mag_yaw, float dt, float *yaw_state);


void calculate_motor_outputs(double z_output, double roll_output, double pitch_output, double yaw_output,
                           double *motor1, double *motor2, double *motor3, double *motor4);



#endif /* CONFIG_H_ */
