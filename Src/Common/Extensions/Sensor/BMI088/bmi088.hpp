#pragma once
#include "stdint.h"
#include "./middleware.hpp"

#define BMI088_TEMP_FACTOR               0.125f
#define BMI088_TEMP_OFFSET               23.0f

#define BMI088_WRITE_ACCEL_REG_NUM       6
#define BMI088_WRITE_GYRO_REG_NUM        6

#define BMI088_GYRO_DATA_READY_BIT       0
#define BMI088_ACCEL_DATA_READY_BIT      1
#define BMI088_ACCEL_TEMP_DATA_READY_BIT 2

#define BMI088_LONG_DELAY_TIME           80
#define BMI088_COM_WAIT_SENSOR_TIME      150

#define BMI088_ACCEL_IIC_ADDRESSE        (0x18 << 1)
#define BMI088_GYRO_IIC_ADDRESSE         (0x68 << 1)

#define BMI088_ACCEL_RANGE_3G
// #define BMI088_ACCEL_RANGE_6G
// #define BMI088_ACCEL_RANGE_12G
// #define BMI088_ACCEL_RANGE_24G

#define BMI088_GYRO_RANGE_2000
// #define BMI088_GYRO_RANGE_1000
// #define BMI088_GYRO_RANGE_500
// #define BMI088_GYRO_RANGE_250
// #define BMI088_GYRO_RANGE_125

#define BMI088_ACCEL_3G_SEN  0.0008974358974f
#define BMI088_ACCEL_6G_SEN  0.00179443359375f
#define BMI088_ACCEL_12G_SEN 0.0035888671875f
#define BMI088_ACCEL_24G_SEN 0.007177734375f

#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN 0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN  0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN  0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN  0.000066579027251980956150958662738366f

#pragma pack(1)
typedef struct BMI088_RAW_DATA {
    uint8_t status;
    int16_t accel[3];
    int16_t temperate;
    int16_t gyro[3];
    float time;
} bmi088_raw_data_t;
#pragma pack()

typedef struct BMI088_REAL_DATA {
    uint8_t status;
    float accel[3];
    float temperate;
    float gyro[3];
    float time;
} bmi088_real_data_t;


enum {
    BMI088_NO_ERROR = 0x00,
    BMI088_ACC_PWR_CTRL_ERROR = 0x01,
    BMI088_ACC_PWR_CONF_ERROR = 0x02,
    BMI088_ACC_CONF_ERROR = 0x03,
    BMI088_ACC_SELF_TEST_ERROR = 0x04,
    BMI088_ACC_RANGE_ERROR = 0x05,
    BMI088_INT1_IO_CTRL_ERROR = 0x06,
    BMI088_INT_MAP_DATA_ERROR = 0x07,
    BMI088_GYRO_RANGE_ERROR = 0x08,
    BMI088_GYRO_BANDWIDTH_ERROR = 0x09,
    BMI088_GYRO_LPM1_ERROR = 0x0A,
    BMI088_GYRO_CTRL_ERROR = 0x0B,
    BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
    BMI088_GYRO_INT3_INT4_IO_MAP_ERROR = 0x0D,
    BMI088_NO_SPI = 0x0E,

    BMI088_SELF_TEST_ACCEL_ERROR = 0x80,
    BMI088_SELF_TEST_GYRO_ERROR = 0x40,
    BMI088_NO_SENSOR = 0xFF,
};

class BMI088 : public Middleware {
public:
    uint8_t init(SPI_HandleTypeDef *_spi);
    bmi088_raw_data_t readRaw(); // read raw data in raw format
    bmi088_real_data_t read();   // read raw data in expected format
    inline int16_t getRawTemperature(void) { return rawData_.temperate; }
    inline int16_t getRawAccelX(void) { return rawData_.accel[0]; }
    inline int16_t getRawAccelY(void) { return rawData_.accel[1]; }
    inline int16_t getRawAccelZ(void) { return rawData_.accel[2]; }
    inline int16_t getRawGyroX(void) { return rawData_.gyro[0]; }
    inline int16_t getRawGyroY(void) { return rawData_.gyro[1]; }
    inline int16_t getRawGyroZ(void) { return rawData_.gyro[2]; }
    inline float getAccelMappingVaule(void) { return aTransK_; }
    inline float getGyroMappingVaule(void) { return gTransK_; }
    inline float getTemperature(void) { return data_.temperate; }
    inline float getAccelX(void) { return data_.accel[0]; }
    inline float getAccelY(void) { return data_.accel[1]; }
    inline float getAccelZ(void) { return data_.accel[2]; }
    inline float getGyroX(void) { return data_.gyro[0]; }
    inline float getGyroY(void) { return data_.gyro[1]; }
    inline float getGyroZ(void) { return data_.gyro[2]; }
    inline float getTimestamp(void) { return data_.time; }
    

protected:
    uint8_t accelInit(void);
    uint8_t gyroInit(void);

    void accel_read_single_reg(uint8_t reg, uint8_t &data);
    void accel_read_muli_reg(uint8_t reg, uint8_t *data, uint8_t len);
    void accel_write_single_reg(uint8_t reg, uint8_t data);

    void gyro_read_single_reg(uint8_t reg, uint8_t &data);
    void gyro_read_muli_reg(uint8_t reg, uint8_t *data, uint8_t len);
    void gyro_write_single_reg(uint8_t reg, uint8_t data);

    void write_single_reg(uint8_t reg, uint8_t data);
    void read_single_reg(uint8_t reg, uint8_t *return_data);
    void read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

private:
    // Mapping encoding to international unit parameters
    float aTransK_;
    float gTransK_;
    
    bmi088_real_data_t data_;
    bmi088_raw_data_t rawData_;

    uint32_t sensorTick_ = 0; // sensor time per 39.0625us
    uint32_t sensorTickLast_ = 0; // last sensor time
};
