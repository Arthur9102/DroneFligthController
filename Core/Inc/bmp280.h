#ifndef BMP280_H_
#define BMP280_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* Định nghĩa địa chỉ I2C */
#define BMP280_ADDRESS          0x76  // 7-bit address (SDO = GND). Nếu SDO = VDD, sử dụng 0x77.

/* Định nghĩa thanh ghi */
#define BMP280_REG_CALIB00      0x88
#define BMP280_REG_ID           0xD0
#define BMP280_REG_RESET        0xE0
#define BMP280_REG_CTRL_MEAS    0xF4
#define BMP280_REG_CONFIG       0xF5
#define BMP280_REG_DATA         0xF7

/* Giá trị khác */
#define BMP280_CHIP_ID          0x58
#define BMP280_SOFT_RESET       0xB6

/* Cấu trúc quản lý BMP280 */
typedef struct {
    I2C_HandleTypeDef *hi2c;      // Con trỏ tới I2C handle
    uint8_t dev_addr;             // Địa chỉ I2C (7-bit)
    uint16_t timeout;             // Thời gian chờ I2C (ms)
    uint16_t dig_T1;              // Hệ số hiệu chỉnh nhiệt độ
    int16_t dig_T2, dig_T3;       // Hệ số hiệu chỉnh nhiệt độ
    uint16_t dig_P1;              // Hệ số hiệu chỉnh áp suất
    int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9; // Hệ số hiệu chỉnh áp suất
    int32_t t_fine;               // Biến trung gian cho tính toán
    float P0;                     // Áp suất tham chiếu (hPa)
} BMP280_HandleTypedef;

/**
 * @brief Đọc một thanh ghi 8-bit từ BMP280
 * @param bmp Cấu trúc quản lý BMP280
 * @param reg Địa chỉ thanh ghi
 * @retval Giá trị thanh ghi hoặc 0 nếu lỗi
 */
uint8_t bmp280_read8(BMP280_HandleTypedef *bmp, uint8_t reg);

/**
 * @brief Đọc dữ liệu hiệu chỉnh từ BMP280
 * @param bmp Cấu trúc quản lý BMP280
 */
void bmp280_read_calib_data(BMP280_HandleTypedef *bmp);

/**
 * @brief Khởi tạo BMP280
 * @param bmp Cấu trúc quản lý BMP280
 * @retval 1: Thành công, 0: Thất bại
 */
uint8_t BMP280_Init(BMP280_HandleTypedef *bmp);

/**
 * @brief Đọc nhiệt độ và áp suất từ BMP280
 * @param bmp Cấu trúc quản lý BMP280
 * @param temperature Con trỏ để lưu nhiệt độ (°C)
 * @param pressure Con trỏ để lưu áp suất (hPa)
 * @retval 1: Thành công, 0: Thất bại
 */
uint8_t BMP280_ReadTemperaturePressure(BMP280_HandleTypedef *bmp, float *temperature, float *pressure);

/**
 * @brief Tính độ cao từ áp suất
 * @param pressure Áp suất hiện tại (hPa)
 * @param P0 Áp suất tham chiếu (hPa)
 * @retval Độ cao (mét)
 */
float BMP280_CalculateAltitude(float pressure, float P0);

#endif /* BMP280_H_ */
