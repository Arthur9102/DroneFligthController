/**
 * @file    bmp280.c
 * @brief   Driver for BMP280 pressure and temperature sensor
 * @author  Ho Quang Dung
 * @date    2025-05-07
 * @version v1.0
 * @par Copyright
 *          (C) 2025 Ho Quang Dung. All rights reserved.
 * @par History
 *          1: Created
 * @par Reference
 *          BMP280 Datasheet, STM32 HAL Library
 */

#include "bmp280.h"
#include <math.h>

/**
 * @brief Đọc một thanh ghi 8-bit từ BMP280
 * @param bmp Cấu trúc quản lý BMP280
 * @param reg Địa chỉ thanh ghi
 * @retval Giá trị thanh ghi hoặc 0 nếu lỗi
 */
uint8_t bmp280_read8(BMP280_HandleTypedef *bmp, uint8_t reg) {
    uint8_t val;
    if (bmp == NULL || bmp->hi2c == NULL) {
        return 0;
    }
    if (HAL_I2C_Mem_Read(bmp->hi2c, bmp->dev_addr << 1, reg, 1, &val, 1, bmp->timeout) != HAL_OK) {
        return 0;
    }
    return val;
}

/**
 * @brief Đọc dữ liệu hiệu chỉnh từ BMP280
 * @param bmp Cấu trúc quản lý BMP280
 */
void bmp280_read_calib_data(BMP280_HandleTypedef *bmp) {
    if (bmp == NULL || bmp->hi2c == NULL) {
        return;
    }

    uint8_t calib[24];
    if (HAL_I2C_Mem_Read(bmp->hi2c, bmp->dev_addr << 1, BMP280_REG_CALIB00, 1, calib, 24, bmp->timeout) != HAL_OK) {
        return;
    }

    bmp->dig_T1 = (calib[1] << 8) | calib[0];
    bmp->dig_T2 = (calib[3] << 8) | calib[2];
    bmp->dig_T3 = (calib[5] << 8) | calib[4];
    bmp->dig_P1 = (calib[7] << 8) | calib[6];
    bmp->dig_P2 = (calib[9] << 8) | calib[8];
    bmp->dig_P3 = (calib[11] << 8) | calib[10];
    bmp->dig_P4 = (calib[13] << 8) | calib[12];
    bmp->dig_P5 = (calib[15] << 8) | calib[14];
    bmp->dig_P6 = (calib[17] << 8) | calib[16];
    bmp->dig_P7 = (calib[19] << 8) | calib[18];
    bmp->dig_P8 = (calib[21] << 8) | calib[20];
    bmp->dig_P9 = (calib[23] << 8) | calib[22];
}

/**
 * @brief Khởi tạo BMP280
 * @param bmp Cấu trúc quản lý BMP280
 * @retval 1: Thành công, 0: Thất bại
 */
uint8_t BMP280_Init(BMP280_HandleTypedef *bmp) {
    if (bmp == NULL || bmp->hi2c == NULL) {
        return 0;
    }

    // Thực hiện soft reset
    uint8_t reset = BMP280_SOFT_RESET;
    if (HAL_I2C_Mem_Write(bmp->hi2c, bmp->dev_addr << 1, BMP280_REG_RESET, 1, &reset, 1, bmp->timeout) != HAL_OK) {
        return 0;
    }
    HAL_Delay(10); // Chờ reset hoàn tất

    // Xác minh chip ID
    uint8_t id = bmp280_read8(bmp, BMP280_REG_ID);
    if (id != BMP280_CHIP_ID) {
        return 0;
    }

    // Đọc dữ liệu hiệu chỉnh
    bmp280_read_calib_data(bmp);
    if (bmp->dig_T1 == 0 || bmp->dig_P1 == 0) {
        return 0; // Dữ liệu hiệu chỉnh không hợp lệ
    }

    // Cấu hình cảm biến
    uint8_t config = 0xA0; // Standby 1000ms, bộ lọc tắt
    uint8_t ctrl_meas = 0x27; // Nhiệt độ ×1, áp suất ×4, chế độ normal
    if (HAL_I2C_Mem_Write(bmp->hi2c, bmp->dev_addr << 1, BMP280_REG_CONFIG, 1, &config, 1, bmp->timeout) != HAL_OK ||
        HAL_I2C_Mem_Write(bmp->hi2c, bmp->dev_addr << 1, BMP280_REG_CTRL_MEAS, 1, &ctrl_meas, 1, bmp->timeout) != HAL_OK) {
        return 0;
    }

    // Tính trung bình áp suất ban đầu cho P0
    float sum_pressure = 0;
    for (int i = 0; i < 5; i++) {
        float temp, press;
        if (BMP280_ReadTemperaturePressure(bmp, &temp, &press)) {
            sum_pressure += press;
        } else {
            return 0;
        }
        HAL_Delay(50); // Chờ dữ liệu ổn định
    }
    bmp->P0 = sum_pressure / 5;

    return 1; // Thành công
}

/**
 * @brief Đọc nhiệt độ và áp suất từ BMP280
 * @param bmp Cấu trúc quản lý BMP280
 * @param temperature Con trỏ để lưu nhiệt độ (°C)
 * @param pressure Con trỏ để lưu áp suất (hPa)
 * @retval 1: Thành công, 0: Thất bại
 */
uint8_t BMP280_ReadTemperaturePressure(BMP280_HandleTypedef *bmp, float *temperature, float *pressure) {
    if (bmp == NULL || bmp->hi2c == NULL || temperature == NULL || pressure == NULL) {
        return 0;
    }

    uint8_t data[6];
    if (HAL_I2C_Mem_Read(bmp->hi2c, bmp->dev_addr << 1, BMP280_REG_DATA, 1, data, 6, bmp->timeout) != HAL_OK) {
        return 0;
    }

    int32_t adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    int32_t adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);

    // Tính toán nhiệt độ
    int32_t var1, var2;
    var1 = ((((adc_T >> 3) - ((int32_t)bmp->dig_T1 << 1))) * ((int32_t)bmp->dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)bmp->dig_T1)) * ((adc_T >> 4) - ((int32_t)bmp->dig_T1))) >> 12) * ((int32_t)bmp->dig_T3)) >> 14;
    bmp->t_fine = var1 + var2;
    *temperature = ((bmp->t_fine * 5 + 128) >> 8) / 100.0f;

    // Tính toán áp suất
    int64_t var1_p, var2_p, p;
    var1_p = ((int64_t)bmp->t_fine) - 128000;
    var2_p = var1_p * var1_p * (int64_t)bmp->dig_P6;
    var2_p = var2_p + ((var1_p * (int64_t)bmp->dig_P5) << 17);
    var2_p = var2_p + (((int64_t)bmp->dig_P4) << 35);
    var1_p = ((var1_p * var1_p * (int64_t)bmp->dig_P3) >> 8) + ((var1_p * (int64_t)bmp->dig_P2) << 12);
    var1_p = (((((int64_t)1) << 47) + var1_p)) * ((int64_t)bmp->dig_P1) >> 33;

    if (var1_p == 0) {
        *pressure = 0;
        return 0;
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2_p) * 3125) / var1_p;
    var1_p = (((int64_t)bmp->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2_p = (((int64_t)bmp->dig_P8) * p) >> 19;

    p = ((p + var1_p + var2_p) >> 8) + (((int64_t)bmp->dig_P7) << 4);
    *pressure = p / 256.0f / 100.0f; // hPa

    return 1; // Thành công
}

/**
 * @brief Tính độ cao từ áp suất
 * @param pressure Áp suất hiện tại (hPa)
 * @param P0 Áp suất tham chiếu (hPa)
 * @retval Độ cao (mét)
 */
float BMP280_CalculateAltitude(float pressure, float P0) {
    if (P0 <= 0 || pressure <= 0) {
        return 0;
    }
    return 44330.0f * (1.0f - powf(pressure / P0, 0.190294957f)); // Sử dụng hệ số chính xác
}
