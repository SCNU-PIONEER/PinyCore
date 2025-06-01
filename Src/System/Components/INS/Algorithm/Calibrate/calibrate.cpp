#include <math.h>
#include <stdlib.h>
#include "calibrate.hpp"

#define IMU_RAW_RANGE (32768)

imu_data_fp_t ImuCalibration::Correct(uint16_t gx, uint16_t gy, uint16_t gz,
                                      uint16_t ax, uint16_t ay, uint16_t az,
                                      float temperature)
{
    imu_data_fp_t imu_corr;
    float ax_ub = ax - acc_cali_.accel_offs[0];
    float ay_ub = ay - acc_cali_.accel_offs[1];
    float az_ub = az - acc_cali_.accel_offs[2];
    imu_corr.ax =
            (acc_cali_.accel_T[0][0] * ax_ub + acc_cali_.accel_T[0][1] * ay_ub +
             acc_cali_.accel_T[0][2] * az_ub);
    imu_corr.ay =
            (acc_cali_.accel_T[1][0] * ax_ub + acc_cali_.accel_T[1][1] * ay_ub +
             acc_cali_.accel_T[1][2] * az_ub);
    imu_corr.az =
            (acc_cali_.accel_T[2][0] * ax_ub + acc_cali_.accel_T[2][1] * ay_ub +
             acc_cali_.accel_T[2][2] * az_ub);

    imu_corr.ax *= (aFullscale_ / (float)(IMU_RAW_RANGE));
    imu_corr.ay *= (aFullscale_ / (float)(IMU_RAW_RANGE));
    imu_corr.az *= (aFullscale_ / (float)(IMU_RAW_RANGE));

    imu_corr.gx = (((float)gx) - gyro_cali_.gx_bias -
                   (gyro_cali_.gx_tco_k * temperature + gyro_cali_.gx_tco_b0)) *
                  (gFullscale_ / (float)(IMU_RAW_RANGE));
    imu_corr.gy = (((float)gy) - gyro_cali_.gy_bias -
                   (gyro_cali_.gy_tco_k * temperature + gyro_cali_.gy_tco_b0)) *
                  (gFullscale_ / (float)(IMU_RAW_RANGE));
    imu_corr.gz = (((float)gz) - gyro_cali_.gz_bias -
                   (gyro_cali_.gz_tco_k * temperature + gyro_cali_.gz_tco_b0)) *
                  (gFullscale_ / (float)(IMU_RAW_RANGE));

    return imu_corr;
}
