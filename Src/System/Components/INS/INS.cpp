#include "INS.hpp"

#include "TopicRouter.hpp"

#define CORRECT_IMU_DATA 1

using namespace INS_SYS;

INS::INS()
        : insPub_(new Publisher<INSData_s>(&TopicRouter::instance().insTopic,
                                           &insDat_))
{
}

void INS::init(const AccCali_s &_accCali, const GyroCali_s &_gyroCali)
{
    // Initialize Rotation Matrix
    arm_mat_init_f32(&R_, 3, 3, R_data_);

    // Initialize Body and Earth axis system vectors
    arm_mat_init_f32(&bodyVectorT_, 3, 1, bodyV_data_);
    arm_mat_init_f32(&earthVectorT_, 3, 1, earthV_data_);

    // Initialize IMU calibration
    imuCali_.init(_accCali, _gyroCali);

    // Initialize DCM algorithm
    DCM_.init();

    // Set initial time interval
    this->dt_ = 0.001f; // default to 1 ms
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
void INS::update(IMUSensorRawData_s *_sensorDat, float _dt, float _temperature)
{
    this->dt_ = _dt; // update time interval

    // Update raw IMU data
    rawDat_.a.x = static_cast<float>(_sensorDat->a.x) * _sensorDat->a.transK;
    rawDat_.a.y = static_cast<float>(_sensorDat->a.y) * _sensorDat->a.transK;
    rawDat_.a.z = static_cast<float>(_sensorDat->a.z) * _sensorDat->a.transK;
    rawDat_.g.x = static_cast<float>(_sensorDat->g.x) * _sensorDat->g.transK;
    rawDat_.g.y = static_cast<float>(_sensorDat->g.y) * _sensorDat->g.transK;
    rawDat_.g.z = static_cast<float>(_sensorDat->g.z) * _sensorDat->g.transK;

    float w = insDat_.q[0], x = insDat_.q[1], y = insDat_.q[2],
          z = insDat_.q[3];

    // Update IMU calibration
    imu_data_fp_t fData;
#if CORRECT_IMU_DATA
    fData = imuCali_.correctInt16(_sensorDat->a.transK, _sensorDat->g.transK,
                                  _sensorDat->g.x, _sensorDat->g.y,
                                  _sensorDat->g.z, _sensorDat->a.x,
                                  _sensorDat->a.y, _sensorDat->a.z,
                                  _temperature);
    imuCali_.steadyStateDetection();
#else
    fData.gx = rawDat_.g.x;
    fData.gy = rawDat_.g.y;
    fData.gz = rawDat_.g.z;
    fData.ax = rawDat_.a.x;
    fData.ay = rawDat_.a.y;
    fData.az = rawDat_.a.z;
#endif

    // Update DCM algorithm
    DCM_.update(fData.gx, fData.gy, fData.gz, fData.ax, fData.ay, fData.az,
                this->dt_);

    // Quaternion data
    DCM_.getQuaternion(insDat_.q);

    // Get the Euler angles
    insDat_.roll = DCM_.getRoll();
    insDat_.pitch = DCM_.getPitch();
    insDat_.yaw = DCM_.getYaw();

    // Update the body axis system data
    insDat_.body.gx = fData.gx;
    insDat_.body.gy = fData.gy;
    insDat_.body.gz = fData.gz;
    insDat_.body.ax = fData.ax;
    insDat_.body.ay = fData.ay;
    insDat_.body.az = fData.az;

    // Update Rotation Matrix
#if ROTATION_MATRIX_PITCH_ONLY
    R_data_[0] = cosf(insDat_.pitch);  // R[0][0]
    R_data_[1] = 0.0f;                 // R[0][1]
    R_data_[2] = sinf(insDat_.pitch);  // R[0][2]
    R_data_[3] = 0.0f;                 // R[1][0]
    R_data_[4] = 1.0f;                 // R[1][1]
    R_data_[5] = 0.0f;                 // R[1][2]
    R_data_[6] = -sinf(insDat_.pitch); // R[2][0]
    R_data_[7] = 0.0f;                 // R[2][1]
    R_data_[8] = cosf(insDat_.pitch);  // R[2][2]
#else
    R_data_[0] = 1.f - 2.f * y * y - 2.f * z * z; // R[0][0] = 1-2y^2-2z^2
    R_data_[1] = 2.f * x * y - 2.f * w * z;       // R[0][1] = 2xy - 2wz
    R_data_[2] = 2.f * x * z + 2.f * w * y;       // R[0][2] = 2xz + 2wy
    R_data_[3] = 2.f * x * y + 2.f * w * z;       // R[1][0] = 2xy + 2wz
    R_data_[4] = 1.f - 2.f * x * x - 2.f * z * z; // R[1][1] = 1-2x^2-2z^2
    R_data_[5] = 2.f * y * z - 2.f * w * x;       // R[1][2] = 2yz - 2wx
    R_data_[6] = 2.f * x * z - 2.f * w * y;       // R[2][0] = 2xz - 2wy
    R_data_[7] = 2.f * y * z + 2.f * w * x;       // R[2][1] = 2xy + 2wz
    R_data_[8] = 1.f - 2.f * x * x - 2.f * y * y; // R[2][2] = 1-2x^2-2y^2
#endif

    // Transform body axis data to earth axis system using the rotation matrix
    bodyV_data_[0] = insDat_.body.ax;
    bodyV_data_[1] = insDat_.body.ay;
    bodyV_data_[2] = insDat_.body.az;
    arm_mat_mult_f32(&R_, &bodyVectorT_, &earthVectorT_);
    insDat_.earth.ax = earthV_data_[0];
    insDat_.earth.ay = earthV_data_[1];
    insDat_.earth.az = earthV_data_[2];

    bodyV_data_[0] = insDat_.body.gx;
    bodyV_data_[1] = insDat_.body.gy;
    bodyV_data_[2] = insDat_.body.gz;
    arm_mat_mult_f32(&R_, &bodyVectorT_, &earthVectorT_);
    insDat_.earth.gx = earthV_data_[0];
    insDat_.earth.gy = earthV_data_[1];
    insDat_.earth.gz = earthV_data_[2];

    bodyV_data_[0] = insDat_.body.mx;
    bodyV_data_[1] = insDat_.body.my;
    bodyV_data_[2] = insDat_.body.mz;
    arm_mat_mult_f32(&R_, &bodyVectorT_, &earthVectorT_);
    insDat_.earth.mx = earthV_data_[0];
    insDat_.earth.my = earthV_data_[1];
    insDat_.earth.mz = earthV_data_[2];

    // temperature data
    temperature_ = _temperature;

    // send queue
    insPub_->publish();
}
