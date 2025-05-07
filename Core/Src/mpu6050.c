/**
 * @file    mpu6050.c
 * @brief   Driver for MPU6050 6-axis IMU (accelerometer and gyroscope)
 * @author  Ho Quang Dung
 * @date    2025-05-07
 * @version v1.0
 * @par Copyright
 *          (C) 2025 Ho Quang Dung. All rights reserved.
 * @par History
 *          1: Created
 * @par Reference
 *          MPU6050 Datasheet, STM32 HAL Library
 */

#include "mpu6050.h"

/**
 * @brief Khởi tạo MPU6050
 * @param hmpu Cấu trúc quản lý MPU6050
 * @retval 1: Thành công, 0: Thất bại
 */
uint8_t MPU6050_Init(MPU6050_HandleTypeDef *hmpu) {
    uint8_t data;

    // Kiểm tra con trỏ và thông số
    if (hmpu == NULL || hmpu->hi2c == NULL) {
        return 0;
    }

    // Xác minh ID thiết bị
    if (HAL_I2C_Mem_Read(hmpu->hi2c, hmpu->dev_addr << 1, MPU6050_REG_WHO_AM_I, 1, &data, 1, hmpu->timeout) != HAL_OK) {
        return 0;
    }
    if (data != 0x68) {
        return 0; // ID thiết bị không đúng
    }

    // Reset thiết bị
    data = 0x80; // DEVICE_RESET
    if (HAL_I2C_Mem_Write(hmpu->hi2c, hmpu->dev_addr << 1, MPU6050_REG_PWR_MGMT_1, 1, &data, 1, hmpu->timeout) != HAL_OK) {
        return 0;
    }
    HAL_Delay(100); // Chờ reset hoàn tất

    // Đánh thức thiết bị và chọn nguồn xung PLL
    data = 0x01; // CLKSEL = 1 (PLL với tham chiếu con quay trục X)
    if (HAL_I2C_Mem_Write(hmpu->hi2c, hmpu->dev_addr << 1, MPU6050_REG_PWR_MGMT_1, 1, &data, 1, hmpu->timeout) != HAL_OK) {
        return 0;
    }

    // Đặt tỷ lệ mẫu 1kHz (SMPLRT_DIV = 7: 8kHz / (1 + 7))
    data = 0x07;
    if (HAL_I2C_Mem_Write(hmpu->hi2c, hmpu->dev_addr << 1, MPU6050_REG_SMPLRT_DIV, 1, &data, 1, hmpu->timeout) != HAL_OK) {
        return 0;
    }

    // Cấu hình DLPF (44Hz cho cả con quay và gia tốc)
    data = 0x03; // DLPF_CFG = 3
    if (HAL_I2C_Mem_Write(hmpu->hi2c, hmpu->dev_addr << 1, MPU6050_REG_CONFIG, 1, &data, 1, hmpu->timeout) != HAL_OK) {
        return 0;
    }

    // Đặt phạm vi con quay ±2000°/s
    hmpu->gyro_range = 3;
    data = hmpu->gyro_range << 3; // FS_SEL = 3
    if (HAL_I2C_Mem_Write(hmpu->hi2c, hmpu->dev_addr << 1, MPU6050_REG_GYRO_CONFIG, 1, &data, 1, hmpu->timeout) != HAL_OK) {
        return 0;
    }

    // Đặt phạm vi gia tốc ±16g
    hmpu->accel_range = 3;
    data = hmpu->accel_range << 3; // AFS_SEL = 3
    if (HAL_I2C_Mem_Write(hmpu->hi2c, hmpu->dev_addr << 1, MPU6050_REG_ACCEL_CONFIG, 1, &data, 1, hmpu->timeout) != HAL_OK) {
        return 0;
    }

    // Tắt FIFO
    data = 0x00;
    if (HAL_I2C_Mem_Write(hmpu->hi2c, hmpu->dev_addr << 1, MPU6050_REG_USER_CTRL, 1, &data, 1, hmpu->timeout) != HAL_OK) {
        return 0;
    }

    return 1; // Thành công
}

/**
 * @brief Đọc dữ liệu thô từ MPU6050
 * @param hmpu Cấu trúc quản lý MPU6050
 * @param data Con trỏ tới dữ liệu thô
 * @retval 1: Thành công, 0: Thất bại
 */
uint8_t MPU6050_ReadRawData(MPU6050_HandleTypeDef *hmpu, MPU6050_RawData *data) {
    uint8_t buffer[14];

    // Kiểm tra con trỏ
    if (hmpu == NULL || hmpu->hi2c == NULL || data == NULL) {
        return 0;
    }

    // Đọc 14 byte từ ACCEL_XOUT_H
    if (HAL_I2C_Mem_Read(hmpu->hi2c, hmpu->dev_addr << 1, MPU6050_REG_ACCEL_XOUT_H, 1, buffer, 14, hmpu->timeout) != HAL_OK) {
        return 0;
    }

    // Kết hợp byte cao và thấp
    data->accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);
    data->temp = (int16_t)((buffer[6] << 8) | buffer[7]);
    data->gyro_x = (int16_t)((buffer[8] << 8) | buffer[9]);
    data->gyro_y = (int16_t)((buffer[10] << 8) | buffer[11]);
    data->gyro_z = (int16_t)((buffer[12] << 8) | buffer[13]);

    return 1; // Thành công
}

/**
 * @brief Chuyển đổi dữ liệu thô thành đơn vị vật lý
 * @param hmpu Cấu trúc quản lý MPU6050
 * @param raw_data Dữ liệu thô
 * @param conv_data Dữ liệu đã chuyển đổi
 */
void MPU6050_ConvertData(MPU6050_HandleTypeDef *hmpu, MPU6050_RawData *raw_data, MPU6050_ConvertedData *conv_data) {
    float accel_scale, gyro_scale;

    // Chọn hệ số tỷ lệ cho gia tốc
    switch (hmpu->accel_range) {
        case 0: accel_scale = 16384.0f; break; // ±2g
        case 1: accel_scale = 8192.0f; break;  // ±4g
        case 2: accel_scale = 4096.0f; break;  // ±8g
        case 3: accel_scale = 2048.0f; break;  // ±16g
        default: accel_scale = 16384.0f;
    }

    // Chọn hệ số tỷ lệ cho con quay
    switch (hmpu->gyro_range) {
        case 0: gyro_scale = 131.0f; break;    // ±250°/s
        case 1: gyro_scale = 65.5f; break;     // ±500°/s
        case 2: gyro_scale = 32.8f; break;     // ±1000°/s
        case 3: gyro_scale = 16.4f; break;     // ±2000°/s
        default: gyro_scale = 131.0f;
    }

    // Chuyển đổi dữ liệu
    conv_data->accel_x = raw_data->accel_x / accel_scale;
    conv_data->accel_y = raw_data->accel_y / accel_scale;
    conv_data->accel_z = raw_data->accel_z / accel_scale;
    conv_data->gyro_x = raw_data->gyro_x / gyro_scale;
    conv_data->gyro_y = raw_data->gyro_y / gyro_scale;
    conv_data->gyro_z = raw_data->gyro_z / gyro_scale;
    conv_data->temp = (raw_data->temp / 340.0f) + 36.53f; // Nhiệt độ (°C)
}

/**
 * @brief Đặt tỷ lệ mẫu
 * @param hmpu Cấu trúc quản lý MPU6050
 * @param rate Giá trị SMPLRT_DIV (8kHz / (1 + rate))
 * @retval 1: Thành công, 0: Thất bại
 */
uint8_t MPU6050_SetSampleRate(MPU6050_HandleTypeDef *hmpu, uint8_t rate) {
    if (hmpu == NULL || hmpu->hi2c == NULL) {
        return 0;
    }

    if (HAL_I2C_Mem_Write(hmpu->hi2c, hmpu->dev_addr << 1, MPU6050_REG_SMPLRT_DIV, 1, &rate, 1, hmpu->timeout) != HAL_OK) {
        return 0;
    }
    return 1;
}

/**
 * @brief Đặt phạm vi con quay hồi chuyển
 * @param hmpu Cấu trúc quản lý MPU6050
 * @param range 0: ±250°/s, 1: ±500°/s, 2: ±1000°/s, 3: ±2000°/s
 * @retval 1: Thành công, 0: Thất bại
 */
uint8_t MPU6050_SetGyroRange(MPU6050_HandleTypeDef *hmpu, uint8_t range) {
    if (hmpu == NULL || hmpu->hi2c == NULL || range > 3) {
        return 0;
    }

    uint8_t data = range << 3; // FS_SEL bits 4:3
    if (HAL_I2C_Mem_Write(hmpu->hi2c, hmpu->dev_addr << 1, MPU6050_REG_GYRO_CONFIG, 1, &data, 1, hmpu->timeout) != HAL_OK) {
        return 0;
    }
    hmpu->gyro_range = range;
    return 1;
}

/**
 * @brief Đặt phạm vi gia tốc kế
 * @param hmpu Cấu trúc quản lý MPU6050
 * @param range 0: ±2g, 1: ±4g, 2: ±8g, 3: ±16g
 * @retval 1: Thành công, 0: Thất bại
 */
uint8_t MPU6050_SetAccelRange(MPU6050_HandleTypeDef *hmpu, uint8_t range) {
    if (hmpu == NULL || hmpu->hi2c == NULL || range > 3) {
        return 0;
    }

    uint8_t data = range << 3; // AFS_SEL bits 4:3
    if (HAL_I2C_Mem_Write(hmpu->hi2c, hmpu->dev_addr << 1, MPU6050_REG_ACCEL_CONFIG, 1, &data, 1, hmpu->timeout) != HAL_OK) {
        return 0;
    }
    hmpu->accel_range = range;
    return 1;
}

/**
 * @brief Đặt bộ lọc thông thấp kỹ thuật số (DLPF)
 * @param hmpu Cấu trúc quản lý MPU6050
 * @param dlpf_cfg Cấu hình DLPF (0-6)
 * @retval 1: Thành công, 0: Thất bại
 */
uint8_t MPU6050_SetDLPF(MPU6050_HandleTypeDef *hmpu, uint8_t dlpf_cfg) {
    if (hmpu == NULL || hmpu->hi2c == NULL || dlpf_cfg > 6) {
        return 0;
    }

    uint8_t data = dlpf_cfg; // DLPF_CFG bits 2:0
    if (HAL_I2C_Mem_Write(hmpu->hi2c, hmpu->dev_addr << 1, MPU6050_REG_CONFIG, 1, &data, 1, hmpu->timeout) != HAL_OK) {
        return 0;
    }
    return 1;
}

/**
 * @brief Kích hoạt/tắt FIFO
 * @param hmpu Cấu trúc quản lý MPU6050
 * @param enable 1: Kích hoạt, 0: Tắt
 * @retval 1: Thành công, 0: Thất bại
 */
uint8_t MPU6050_EnableFIFO(MPU6050_HandleTypeDef *hmpu, uint8_t enable) {
    if (hmpu == NULL || hmpu->hi2c == NULL) {
        return 0;
    }

    uint8_t data = enable ? 0x40 : 0x00; // FIFO_EN bit
    if (HAL_I2C_Mem_Write(hmpu->hi2c, hmpu->dev_addr << 1, MPU6050_REG_USER_CTRL, 1, &data, 1, hmpu->timeout) != HAL_OK) {
        return 0;
    }
    return 1;
}

/**
 * @brief Cấu hình dữ liệu lưu vào FIFO
 * @param hmpu Cấu trúc quản lý MPU6050
 * @param accel 1: Lưu gia tốc, 0: Không lưu
 * @param gyro 1: Lưu con quay, 0: Không lưu
 * @param temp 1: Lưu nhiệt độ, 0: Không lưu
 * @retval 1: Thành công, 0: Thất bại
 */
uint8_t MPU6050_ConfigureFIFO(MPU6050_HandleTypeDef *hmpu, uint8_t accel, uint8_t gyro, uint8_t temp) {
    if (hmpu == NULL || hmpu->hi2c == NULL) {
        return 0;
    }

    uint8_t data = 0;
    if (accel) data |= 0x08; // ACCEL_FIFO_EN
    if (gyro)  data |= 0x70; // XG_FIFO_EN, YG_FIFO_EN, ZG_FIFO_EN
    if (temp)  data |= 0x80; // TEMP_FIFO_EN
    if (HAL_I2C_Mem_Write(hmpu->hi2c, hmpu->dev_addr << 1, MPU6050_REG_FIFO_EN, 1, &data, 1, hmpu->timeout) != HAL_OK) {
        return 0;
    }
    return 1;
}

/**
 * @brief Đặt offset cho gia tốc kế và con quay hồi chuyển
 * @param hmpu Cấu trúc quản lý MPU6050
 * @param xa_offs Offset trục X gia tốc
 * @param ya_offs Offset trục Y gia tốc
 * @param za_offs Offset trục Z gia tốc
 * @param xg_offs Offset trục X con quay
 * @param yg_offs Offset trục Y con quay
 * @param zg_offs Offset trục Z con quay
 * @retval 1: Thành công, 0: Thất bại
 */
uint8_t MPU6050_SetOffsets(MPU6050_HandleTypeDef *hmpu, int16_t xa_offs, int16_t ya_offs, int16_t za_offs,
                           int16_t xg_offs, int16_t yg_offs, int16_t zg_offs) {
    if (hmpu == NULL || hmpu->hi2c == NULL) {
        return 0;
    }

    uint8_t buffer[12];
    buffer[0] = (xa_offs >> 8) & 0xFF; buffer[1] = xa_offs & 0xFF;
    buffer[2] = (ya_offs >> 8) & 0xFF; buffer[3] = ya_offs & 0xFF;
    buffer[4] = (za_offs >> 8) & 0xFF; buffer[5] = za_offs & 0xFF;
    buffer[6] = (xg_offs >> 8) & 0xFF; buffer[7] = xg_offs & 0xFF;
    buffer[8] = (yg_offs >> 8) & 0xFF; buffer[9] = yg_offs & 0xFF;
    buffer[10] = (zg_offs >> 8) & 0xFF; buffer[11] = zg_offs & 0xFF;

    if (HAL_I2C_Mem_Write(hmpu->hi2c, hmpu->dev_addr << 1, MPU6050_REG_XA_OFFSET_H, 1, buffer, 12, hmpu->timeout) != HAL_OK) {
        return 0;
    }
    return 1;
}

/**
 * @brief Reset MPU6050
 * @param hmpu Cấu trúc quản lý MPU6050
 * @retval 1: Thành công, 0: Thất bại
 */
uint8_t MPU6050_Reset(MPU6050_HandleTypeDef *hmpu) {
    if (hmpu == NULL || hmpu->hi2c == NULL) {
        return 0;
    }

    uint8_t data;

    // Reset thiết bị
    data = 0x80; // DEVICE_RESET
    if (HAL_I2C_Mem_Write(hmpu->hi2c, hmpu->dev_addr << 1, MPU6050_REG_PWR_MGMT_1, 1, &data, 1, hmpu->timeout) != HAL_OK) {
        return 0;
    }
    HAL_Delay(100); // Chờ reset

    // Reset đường tín hiệu
    data = 0x07; // GYRO_RESET, ACCEL_RESET, TEMP_RESET
    if (HAL_I2C_Mem_Write(hmpu->hi2c, hmpu->dev_addr << 1, MPU6050_REG_SIGNAL_PATH_RESET, 1, &data, 1, hmpu->timeout) != HAL_OK) {
        return 0;
    }
    HAL_Delay(100); // Chờ reset

    return 1;
}
