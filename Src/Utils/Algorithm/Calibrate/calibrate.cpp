#include <math.h>
#include <stdlib.h>
#include "calibrate.hpp"

#define IMU_RAW_RANGE (32768)

imu_data_fp_t ImuCalibration::Correct(const imu_data_raw_t &imu_data)
{
    imu_data_fp_t imu_corr;
    float ax_ub = imu_data.ax - acc_cali_.accel_offs[0];
    float ay_ub = imu_data.ay - acc_cali_.accel_offs[1];
    float az_ub = imu_data.az - acc_cali_.accel_offs[2];
    imu_corr.ax =
            (acc_cali_.accel_T[0][0] * ax_ub + acc_cali_.accel_T[0][1] * ay_ub +
             acc_cali_.accel_T[0][2] * az_ub);
    imu_corr.ay =
            (acc_cali_.accel_T[1][0] * ax_ub + acc_cali_.accel_T[1][1] * ay_ub +
             acc_cali_.accel_T[1][2] * az_ub);
    imu_corr.az =
            (acc_cali_.accel_T[2][0] * ax_ub + acc_cali_.accel_T[2][1] * ay_ub +
             acc_cali_.accel_T[2][2] * az_ub);

    imu_corr.ax *= (imu_data.a_fullscale / (float)(IMU_RAW_RANGE));
    imu_corr.ay *= (imu_data.a_fullscale / (float)(IMU_RAW_RANGE));
    imu_corr.az *= (imu_data.a_fullscale / (float)(IMU_RAW_RANGE));

    imu_corr.gx = (((float)imu_data.gx) - gyro_cali_.gx_bias -
                   (gyro_cali_.gx_tco_k * imu_data.temperature + gyro_cali_.gx_tco_b0)) *
                  (imu_data.g_fullscale / (float)(IMU_RAW_RANGE));
    imu_corr.gy = (((float)imu_data.gy) - gyro_cali_.gy_bias -
                   (gyro_cali_.gy_tco_k * imu_data.temperature + gyro_cali_.gy_tco_b0)) *
                  (imu_data.g_fullscale / (float)(IMU_RAW_RANGE));
    imu_corr.gz = (((float)imu_data.gz) - gyro_cali_.gz_bias -
                   (gyro_cali_.gz_tco_k * imu_data.temperature + gyro_cali_.gz_tco_b0)) *
                  (imu_data.g_fullscale / (float)(IMU_RAW_RANGE));

    return imu_corr;
}
