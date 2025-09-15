#include "./Bmi088.hpp"
#include "./Reg.hpp"

#if defined(BMI088_USE_SPI)
/**
************************************************************************
* @brief:      	BMI088_accel_write_single_reg(reg, data)
* @param:       reg - 寄存器地址
*               data - 写入的数据
* @retval:     	void
* @details:    	通过BMI088加速度计的SPI总线写入单个寄存器的宏定义
************************************************************************
**/
void BMI088::accelWriteSingleReg(uint8_t _reg, uint8_t _data)
{
    ACCEL_NS_L();
    writeSingleReg(_reg, _data);
    ACCEL_NS_H();
}
/**
************************************************************************
* @brief:      	BMI088_accel_read_single_reg(reg, data)
* @param:       reg - 寄存器地址
*               data - 读取的寄存器数据
* @retval:     	void
* @details:    	通过BMI088加速度计的SPI总线读取单个寄存器的宏定义
************************************************************************
**/
void BMI088::accelReadSingleReg(uint8_t _reg, uint8_t &_data)
{
    ACCEL_NS_L();
    read_write_byte((_reg) | 0x80);
    read_write_byte(0x55);
    _data = read_write_byte(0x55);
    ACCEL_NS_H();
}
/**
************************************************************************
* @brief:      	BMI088_accel_read_muli_reg(reg, data, len)
* @param:       reg - 起始寄存器地址
*               data - 存储读取数据的缓冲区
*               len - 要读取的字节数
* @retval:     	void
* @details:    	通过BMI088加速度计的SPI总线连续读取多个寄存器的宏定义
************************************************************************
**/
void BMI088::accelReadMuliReg(uint8_t _reg, uint8_t *_data, uint8_t _len)
{
    ACCEL_NS_L();
    read_write_byte((_reg) | 0x80);
    readMuliReg(_reg, _data, _len);
    ACCEL_NS_H();
}
/**
************************************************************************
* @brief:      	BMI088_gyro_write_single_reg(reg, data)
* @param:       reg - 寄存器地址
*               data - 写入的数据
* @retval:     	void
* @details:    	通过BMI088陀螺仪的SPI总线写入单个寄存器的宏定义
************************************************************************
**/
void BMI088::gyroWriteSingleReg(uint8_t _reg, uint8_t _data)
{
    GYRO_NS_L();
    writeSingleReg((_reg), (_data));
    GYRO_NS_H();
}
/**
************************************************************************
* @brief:      	BMI088_gyro_read_single_reg(reg, data)
* @param:       reg - 寄存器地址
*               data - 读取的寄存器数据
* @retval:     	void
* @details:    	通过BMI088陀螺仪的SPI总线读取单个寄存器的宏定义
************************************************************************
**/
void BMI088::gyroReadSingleReg(uint8_t _reg, uint8_t &_data)
{
    GYRO_NS_L();
    readSingleReg((_reg), &(_data));
    GYRO_NS_H();
}
/**
************************************************************************
* @brief:      	BMI088_gyro_read_muli_reg(reg, data, len)
* @param:       reg - 起始寄存器地址
*               data - 存储读取数据的缓冲区
*               len - 要读取的字节数
* @retval:     	void
* @details:    	通过BMI088陀螺仪的SPI总线连续读取多个寄存器的宏定义
************************************************************************
**/
void BMI088::gyroReadMuliReg(uint8_t _reg, uint8_t *_data, uint8_t _len)
{
    GYRO_NS_L();
    readMuliReg((_reg), (_data), (_len));
    GYRO_NS_H();
}


#elif defined(BMI088_USE_IIC)

#endif
/**
************************************************************************
* @brief:      	write_BMI088_accel_reg_data_error_init(void)
* @param:       void
* @retval:     	void
* @details:    	BMI088加速度传感器寄存器数据写入错误处理初始化
************************************************************************
**/
static constexpr uint8_t
        WRITE_BMI088_ACCEL_REG_DATA_ERROR[BMI088_WRITE_ACCEL_REG_NUM][3] = {
            { BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON,
              BMI088_ACC_PWR_CTRL_ERROR },
            { BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE,
              BMI088_ACC_PWR_CONF_ERROR },
            { BMI088_ACC_CONF,
              BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set,
              BMI088_ACC_CONF_ERROR },
            { BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR },
            { BMI088_INT1_IO_CTRL,
              BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP |
                      BMI088_ACC_INT1_GPIO_LOW,
              BMI088_INT1_IO_CTRL_ERROR },
            { BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT,
              BMI088_INT_MAP_DATA_ERROR }

        };
/**
************************************************************************
* @brief:      	write_BMI088_gyro_reg_data_error_init(void)
* @param:       void
* @retval:     	void
* @details:    	BMI088陀螺仪传感器寄存器数据写入错误处理初始化
************************************************************************
**/
static constexpr uint8_t
        WRITE_BMI088_GYRO_REG_DATA_ERROR[BMI088_WRITE_GYRO_REG_NUM][3] = {
            { BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR },
            { BMI088_GYRO_BANDWIDTH,
              BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set,
              BMI088_GYRO_BANDWIDTH_ERROR },
            { BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE,
              BMI088_GYRO_LPM1_ERROR },
            { BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR },
            { BMI088_GYRO_INT3_INT4_IO_CONF,
              BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW,
              BMI088_GYRO_INT3_INT4_IO_CONF_ERROR },
            { BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3,
              BMI088_GYRO_INT3_INT4_IO_MAP_ERROR }

        };
/**
************************************************************************
* @brief:      	BMI088_init(void)
* @param:       void
* @retval:     	uint8_t - 错误代码
* @details:    	BMI088传感器初始化函数，包括GPIO和SPI初始化，以及加速度和陀螺仪的初始化
************************************************************************
**/
uint8_t BMI088::init(SPI_HandleTypeDef *_spi)
{
    uint8_t error = BMI088_NO_ERROR;

    // Initialize the SPI interface
    if (_spi != nullptr) {
        BMI088_USING_SPI_UNIT = _spi;
    } else {
        return BMI088_NO_SPI; // No SPI interface provided
    }

    error |= accelInit();
    error |= gyroInit();

    return error;
}
/**
************************************************************************
* @brief:      	bmi088_accel_init(void)
* @param:       void
* @retval:     	uint8_t - 错误代码
* @details:    	BMI088加速度传感器初始化函数，包括通信检查、软件复位、配置寄存器写入及检查
************************************************************************
**/
uint8_t BMI088::accelInit()
{
    uint8_t res = 0;
    uint8_t writeRegNum = 0;

    //check commiunication
    accelReadSingleReg(BMI088_ACC_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    accelReadSingleReg(BMI088_ACC_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //accel software reset
    accelWriteSingleReg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    delay_ms(BMI088_LONG_DELAY_TIME);

    //check commiunication is normal after reset
    accelReadSingleReg(BMI088_ACC_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    accelReadSingleReg(BMI088_ACC_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_ACC_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    //set accel sonsor config and check
    for (writeRegNum = 0; writeRegNum < BMI088_WRITE_ACCEL_REG_NUM;
         writeRegNum++) {
        accelWriteSingleReg(WRITE_BMI088_ACCEL_REG_DATA_ERROR[writeRegNum][0],
                            WRITE_BMI088_ACCEL_REG_DATA_ERROR[writeRegNum][1]);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        accelReadSingleReg(WRITE_BMI088_ACCEL_REG_DATA_ERROR[writeRegNum][0],
                           res);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != WRITE_BMI088_ACCEL_REG_DATA_ERROR[writeRegNum][1]) {
            return WRITE_BMI088_ACCEL_REG_DATA_ERROR[writeRegNum][2];
        }
    }
    return BMI088_NO_ERROR;
}

/**
************************************************************************
* @brief:      	bmi088_gyro_init(void)
* @param:       void
* @retval:     	uint8_t - 错误代码
* @details:    	BMI088陀螺仪传感器初始化函数，包括通信检查、软件复位、配置寄存器写入及检查
************************************************************************
**/
uint8_t BMI088::gyroInit(void)
{
    uint8_t writeRegNum = 0;
    uint8_t res = 0;

    //check commiunication
    gyroReadSingleReg(BMI088_GYRO_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    gyroReadSingleReg(BMI088_GYRO_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //reset the gyro sensor
    gyroWriteSingleReg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    delay_ms(BMI088_LONG_DELAY_TIME);
    //check commiunication is normal after reset
    gyroReadSingleReg(BMI088_GYRO_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    gyroReadSingleReg(BMI088_GYRO_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_GYRO_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    //set gyro sonsor config and check
    for (writeRegNum = 0; writeRegNum < BMI088_WRITE_GYRO_REG_NUM;
         writeRegNum++) {
        gyroWriteSingleReg(WRITE_BMI088_GYRO_REG_DATA_ERROR[writeRegNum][0],
                           WRITE_BMI088_GYRO_REG_DATA_ERROR[writeRegNum][1]);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        gyroReadSingleReg(WRITE_BMI088_GYRO_REG_DATA_ERROR[writeRegNum][0],
                          res);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != WRITE_BMI088_GYRO_REG_DATA_ERROR[writeRegNum][1]) {
            return WRITE_BMI088_GYRO_REG_DATA_ERROR[writeRegNum][2];
        }
    }

    return BMI088_NO_ERROR;
}
/**
************************************************************************
* @brief:      	BMI088_read(float gyro[3], float accel[3], float *temperate)
* @param:       gyro - 陀螺仪数据数组 (x, y, z)
* @param:       accel - 加速度计数据数组 (x, y, z)
* @param:       temperate - 温度数据指针
* @retval:     	void
* @details:    	读取BMI088传感器数据，包括加速度、陀螺仪和温度
************************************************************************
**/
Bmi088RealData_t BMI088::read()
{
    uint8_t buf[8] = { 0, 0, 0, 0, 0, 0 };
    int16_t bmi088RawTemp;

    // read accel data
    accelReadMuliReg(BMI088_ACCEL_XOUT_L, buf, 6);
    bmi088RawTemp = static_cast<int16_t>(((buf[1]) << 8) | buf[0]);
    rawData_.accel[0] = bmi088RawTemp;
    data_.accel[0] = static_cast<float>(bmi088RawTemp) * BMI088_ACCEL_SEN;
    bmi088RawTemp = static_cast<int16_t>(((buf[3]) << 8) | buf[2]);
    rawData_.accel[1] = bmi088RawTemp;
    data_.accel[1] = static_cast<float>(bmi088RawTemp) * BMI088_ACCEL_SEN;
    bmi088RawTemp = static_cast<int16_t>(((buf[5]) << 8) | buf[4]);
    rawData_.accel[2] = bmi088RawTemp;
    data_.accel[2] = static_cast<float>(bmi088RawTemp) * BMI088_ACCEL_SEN;

    // read gyro data
    gyroReadMuliReg(BMI088_GYRO_CHIP_ID, buf, 8);
    if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE) {
        bmi088RawTemp = static_cast<int16_t>(((buf[3]) << 8) | buf[2]);
        rawData_.gyro[0] = bmi088RawTemp;
        data_.gyro[0] = static_cast<float>(bmi088RawTemp) * BMI088_GYRO_SEN;
        bmi088RawTemp = static_cast<int16_t>(((buf[5]) << 8) | buf[4]);
        rawData_.gyro[1] = bmi088RawTemp;
        data_.gyro[1] = static_cast<float>(bmi088RawTemp) * BMI088_GYRO_SEN;
        bmi088RawTemp = static_cast<int16_t>(((buf[7]) << 8) | buf[6]);
        rawData_.gyro[2] = bmi088RawTemp;
        data_.gyro[2] = static_cast<float>(bmi088RawTemp) * BMI088_GYRO_SEN;
    }

    // read 24-bits sensor time
    // this register is incremented every 39.0625us
    sensorTickLast_ = sensorTick_;
    accelReadMuliReg(BMI088_SENSORTIME_DATA_L, buf, 3);
    sensorTick_ = (uint32_t)((buf[2] << 16) | (buf[1] << 8) | buf[0]);
    uint32_t tempDeltaTick;
    if (sensorTick_ < sensorTickLast_) {
        tempDeltaTick = 0x00FFFFFF - (sensorTickLast_ - sensorTick_);
    } else {
        tempDeltaTick = sensorTick_ - sensorTickLast_;
    }
    if (tempDeltaTick != 0)
        data_.time = static_cast<float>(tempDeltaTick) *
                     0.0000390625f; // convert to seconds

    // read temperature
    // the temperature data is updated every 1.28s
    accelReadMuliReg(BMI088_TEMP_M, buf, 2);

    bmi088RawTemp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (bmi088RawTemp > 1023) {
        bmi088RawTemp -= 2048;
    }

    rawData_.temperate = bmi088RawTemp;
    data_.temperate = static_cast<float>(bmi088RawTemp) * BMI088_TEMP_FACTOR +
                      BMI088_TEMP_OFFSET;


    return data_;
}

Bmi088RawData_t BMI088::readRaw() { return rawData_; }

#if defined(BMI088_USE_SPI)
/**
************************************************************************
* @brief:      	BMI088_write_single_reg(uint8_t reg, uint8_t data)
* @param:       reg - 寄存器地址
* @param:       data - 写入的数据
* @retval:     	void
* @details:    	向BMI088传感器写入单个寄存器的数据
************************************************************************
**/
void BMI088::writeSingleReg(uint8_t _reg, uint8_t _data)
{
    read_write_byte(_reg);
    read_write_byte(_data);
}
/**
************************************************************************
* @brief:      	BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
* @param:       reg - 寄存器地址
* @param:       return_data - 读取的寄存器数据
* @retval:     	void
* @details:    	从BMI088传感器读取单个寄存器的数据
************************************************************************
**/
void BMI088::readSingleReg(uint8_t _reg, uint8_t *_return_data)
{
    read_write_byte(_reg | 0x80);
    *_return_data = read_write_byte(0x55);
}

//static void BMI088_write_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len )
//{
//    BMI088_read_write_byte( reg );
//    while( len != 0 )
//    {

//        BMI088_read_write_byte( *buf );
//        buf ++;
//        len --;
//    }

//}
/**
************************************************************************
* @brief:      	BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
* @param:       reg - 起始寄存器地址
*               buf - 存储读取数据的缓冲区
*               len - 要读取的字节数
* @retval:     	void
* @details:    	从BMI088传感器连续读取多个寄存器的数据
************************************************************************
**/
void BMI088::readMuliReg(uint8_t _reg, uint8_t *_buf, uint8_t _len)
{
    read_write_byte(_reg | 0x80);

    while (_len != 0) {
        *_buf = read_write_byte(0x55);
        _buf++;
        _len--;
    }
}
#elif defined(BMI088_USE_IIC)

#endif
