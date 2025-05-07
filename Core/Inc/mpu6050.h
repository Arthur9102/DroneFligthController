#ifndef MPU6050_H_
#define MPU6050_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* Định nghĩa địa chỉ I2C */
#define MPU6050_ADDRESS         0x68  // 7-bit address (AD0 = 0). Nếu AD0 = 1, sử dụng 0x69.

/* Định nghĩa thanh ghi */
#define MPU6050_REG_SMPLRT_DIV      0x19
#define MPU6050_REG_CONFIG          0x1A
#define MPU6050_REG_GYRO_CONFIG     0x1B
#define MPU6050_REG_ACCEL_CONFIG    0x1C
#define MPU6050_REG_FIFO_EN         0x23
#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_REG_SIGNAL_PATH_RESET 0x68
#define MPU6050_REG_USER_CTRL       0x6A
#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_REG_WHO_AM_I        0x75
#define MPU6050_REG_XA_OFFSET_H     0x06
#define MPU6050_REG_XG_OFFSET_H     0x13

/* Cấu trúc dữ liệu cho MPU6050 */
typedef struct {
    I2C_HandleTypeDef *hi2c;      // Con trỏ tới I2C handle
    uint8_t dev_addr;             // Địa chỉ I2C (7-bit)
    uint8_t accel_range;          // Phạm vi gia tốc (0: ±2g, 1: ±4g, 2: ±8g, 3: ±16g)
    uint8_t gyro_range;           // Phạm vi con quay (0: ±250°/s, 1: ±500°/s, 2: ±1000°/s, 3: ±2000°/s)
    uint16_t timeout;             // Thời gian chờ I2C (ms)
} MPU6050_HandleTypeDef;

/* Cấu trúc dữ liệu thô */
typedef struct {
    int16_t accel_x;  // Gia tốc trục X (LSB)
    int16_t accel_y;  // Gia tốc trục Y (LSB)
    int16_t accel_z;  // Gia tốc trục Z (LSB)
    int16_t temp;     // Nhiệt độ (LSB)
    int16_t gyro_x;   // Góc quay trục X (LSB)
    int16_t gyro_y;   // Góc quay trục Y (LSB)
    int16_t gyro_z;   // Góc quay trục Z (LSB)
} MPU6050_RawData;

/* Cấu trúc dữ liệu đã chuyển đổi */
typedef struct {
    float accel_x;    // Gia tốc trục X (g)
    float accel_y;    // Gia tốc trục Y (g)
    float accel_z;    // Gia tốc trục Z (g)
    float temp;       // Nhiệt độ (°C)
    float gyro_x;     // Góc quay trục X (°/s)
    float gyro_y;     // Góc quay trục Y (°/s)
    float gyro_z;     // Góc quay trục Z (°/s)
} MPU6050_ConvertedData;

/* Nguyên mẫu hàm */

/**
 * @brief Khởi tạo MPU6050
 * @param hmpu Cấu trúc quản lý MPU6050
 * @retval 1: Thành công, 0: Thất bại
 */
uint8_t MPU6050_Init(MPU6050_HandleTypeDef *hmpu);

/**
 * @brief Đọc dữ liệu thô từ MPU6050
 * @param hmpu Cấu trúc quản lý MPU6050
 * @param data Con trỏ tới dữ liệu thô
 * @retval 1: Thành công, 0: Thất bại
 */
uint8_t MPU6050_ReadRawData(MPU6050_HandleTypeDef *hmpu, MPU6050_RawData *data);

/**
 * @brief Chuyển đổi dữ liệu thô thành đơn vị vật lý
 * @param hmpu Cấu trúc quản lý MPU6050
 * @param raw_data Dữ liệu thô
 * @param conv_data Dữ liệu đã chuyển đổi
 */
void MPU6050_ConvertData(MPU6050_HandleTypeDef *hmpu, MPU6050_RawData *raw_data, MPU6050_ConvertedData *conv_data);

/**
 * @brief Đặt tỷ lệ mẫu
 * @param hmpu Cấu trúc quản lý MPU6050
 * @param rate Giá trị SMPLRT_DIV (8kHz / (1 + rate))
 * @retval 1: Thành công, 0: Thất bại
 */
uint8_t MPU6050_SetSampleRate(MPU6050_HandleTypeDef *hmpu, uint8_t rate);

/**
 * @brief Đặt phạm vi con quay hồi chuyển
 * @param hmpu Cấu trúc quản lý MPU6050
 * @param range 0: ±250°/s, 1: ±500°/s, 2: ±1000°/s, 3: ±2000°/s
 * @retval 1: Thành công, 0: Thất bại
 */
uint8_t MPU6050_SetGyroRange(MPU6050_HandleTypeDef *hmpu, uint8_t range);

/**
 * @brief Đặt phạm vi gia tốc kế
 * @param hmpu Cấu trúc quản lý MPU6050
 * @param range 0: ±2g, 1: ±4g, 2: ±8g, 3: ±16g
 * @retval 1: Thành công, 0: Thất bại
 */
uint8_t MPU6050_SetAccelRange(MPU6050_HandleTypeDef *hmpu, uint8_t range);

/**
 * @brief Đặt bộ lọc thông thấp kỹ thuật số (DLPF)
 * @param hmpu Cấu trúc quản lý MPU6050
 * @param dlpf_cfg Cấu hình DLPF (0-6)
 * @retval 1: Thành công, 0: Thất bại
 */
uint8_t MPU6050_SetDLPF(MPU6050_HandleTypeDef *hmpu, uint8_t dlpf_cfg);

/**
 * @brief Kích hoạt/tắt FIFO
 * @param hmpu Cấu trúc quản lý MPU6050
 * @param enable 1: Kích hoạt, 0: Tắt
 * @retval 1: Thành công, 0: Thất bại
 */
uint8_t MPU6050_EnableFIFO(MPU6050_HandleTypeDef *hmpu, uint8_t enable);

/**
 * @brief Cấu hình dữ liệu lưu vào FIFO
 * @param hmpu Cấu trúc quản lý MPU6050
 * @param accel 1: Lưu gia tốc, 0: Không lưu
 * @param gyro 1: Lưu con quay, 0: Không lưu
 * @param temp 1: Lưu nhiệt độ, 0: Không lưu
 * @retval 1: Thành công, 0: Thất bại
 */
uint8_t MPU6050_ConfigureFIFO(MPU6050_HandleTypeDef *hmpu, uint8_t accel, uint8_t gyro, uint8_t temp);

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
                           int16_t xg_offs, int16_t yg_offs, int16_t zg_offs);

/**
 * @brief Reset MPU6050
 * @param hmpu Cấu trúc quản lý MPU6050
 * @retval 1: Thành công турист
 */
uint8_t MPU6050_Reset(MPU6050_HandleTypeDef *hmpu);
#endif
