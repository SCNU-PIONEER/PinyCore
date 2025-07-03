#include "./bmi088.hpp"
#include "./reg.hpp"


float BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;


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
void BMI088::accel_write_single_reg(uint8_t reg, uint8_t data)
{
    ACCEL_NS_L();
    write_single_reg(reg, data);
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
void BMI088::accel_read_single_reg(uint8_t reg, uint8_t &data)
{
    ACCEL_NS_L();
    read_write_byte((reg) | 0x80);
    read_write_byte(0x55);
    data = read_write_byte(0x55);
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
void BMI088::accel_read_muli_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
    ACCEL_NS_L();
    read_write_byte((reg) | 0x80);
    read_muli_reg(reg, data, len);
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
void BMI088::gyro_write_single_reg(uint8_t reg, uint8_t data)
{
    GYRO_NS_L();
    write_single_reg((reg), (data));
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
void BMI088::gyro_read_single_reg(uint8_t reg, uint8_t &data)
{
    GYRO_NS_L();
    read_single_reg((reg), &(data));
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
void BMI088::gyro_read_muli_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
    GYRO_NS_L();
    read_muli_reg((reg), (data), (len));
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
static uint8_t
        write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] = {
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
static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] = {
    { BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR },
    { BMI088_GYRO_BANDWIDTH,
      BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set,
      BMI088_GYRO_BANDWIDTH_ERROR },
    { BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR },
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
uint8_t BMI088::accelInit(void)
{
    uint8_t res = 0;
    uint8_t write_reg_num = 0;

    //check commiunication
    accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //accel software reset
    accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    delay_ms(BMI088_LONG_DELAY_TIME);

    //check commiunication is normal after reset
    accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_ACC_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    //set accel sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM;
         write_reg_num++) {
        accel_write_single_reg(
                write_BMI088_accel_reg_data_error[write_reg_num][0],
                write_BMI088_accel_reg_data_error[write_reg_num][1]);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        accel_read_single_reg(
                write_BMI088_accel_reg_data_error[write_reg_num][0], res);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_accel_reg_data_error[write_reg_num][1]) {
            return write_BMI088_accel_reg_data_error[write_reg_num][2];
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
    uint8_t write_reg_num = 0;
    uint8_t res = 0;

    //check commiunication
    gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //reset the gyro sensor
    gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    delay_ms(BMI088_LONG_DELAY_TIME);
    //check commiunication is normal after reset
    gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_GYRO_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    //set gyro sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM;
         write_reg_num++) {
        gyro_write_single_reg(
                write_BMI088_gyro_reg_data_error[write_reg_num][0],
                write_BMI088_gyro_reg_data_error[write_reg_num][1]);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        gyro_read_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0],
                             res);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1]) {
            return write_BMI088_gyro_reg_data_error[write_reg_num][2];
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
bmi088_real_data_t BMI088::read()
{
    uint8_t buf[8] = { 0, 0, 0, 0, 0, 0 };
    int16_t bmi088_raw_temp;

    // read accel data
    accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    rawData_.accel[0] = bmi088_raw_temp;
    data_.accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    rawData_.accel[1] = bmi088_raw_temp;
    data_.accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    rawData_.accel[2] = bmi088_raw_temp;
    data_.accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;

    // read gyro data
    gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
    if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE) {
        bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
        rawData_.gyro[0] = bmi088_raw_temp;
        data_.gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
        rawData_.gyro[1] = bmi088_raw_temp;
        data_.gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
        rawData_.gyro[2] = bmi088_raw_temp;
        data_.gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
    }

    // read 24-bits sensor time
    // this register is incremented every 39.0625us
    sensorTickLast_ = sensorTick_;
    accel_read_muli_reg(BMI088_SENSORTIME_DATA_L, buf, 3);
    sensorTick_ = (uint32_t)((buf[2] << 16) | (buf[1] << 8) | buf[0]);
    uint32_t tempDeltaTick;
    if (sensorTick_ < sensorTickLast_) {
        tempDeltaTick = 0x00FFFFFF - (sensorTickLast_ - sensorTick_);
    } else {
        tempDeltaTick = sensorTick_ - sensorTickLast_;
    }
    if(tempDeltaTick != 0)
        data_.time = static_cast<float>(tempDeltaTick) *
                     0.0000390625f; // convert to seconds

    // read temperature
    // the temperature data is updated every 1.28s
    accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (bmi088_raw_temp > 1023) {
        bmi088_raw_temp -= 2048;
    }

    rawData_.temperate = data_.temperate =
            bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
    

    return data_;
}

bmi088_raw_data_t BMI088::readRaw() { return rawData_; }

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
void BMI088::write_single_reg(uint8_t reg, uint8_t data)
{
    read_write_byte(reg);
    read_write_byte(data);
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
void BMI088::read_single_reg(uint8_t reg, uint8_t *return_data)
{
    read_write_byte(reg | 0x80);
    *return_data = read_write_byte(0x55);
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
void BMI088::read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    read_write_byte(reg | 0x80);

    while (len != 0) {
        *buf = read_write_byte(0x55);
        buf++;
        len--;
    }
}
#elif defined(BMI088_USE_IIC)

#endif
