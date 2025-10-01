/*
 * ===== Model =====
 * accel_corr = accel_T * (accel_raw - accel_offs)
 *
 * accel_corr[3] - fully corrected acceleration vector in body frame
 * accel_T[3][3] - accelerometers transform matrix, rotation and scaling
 * transform accel_raw[3]  - raw acceleration vector accel_offs[3] -
 * acceleration offset vector
 *
 * ===== Calibration =====
 *
 * Reference vectors
 * accel_corr_ref[6][3] = [  g  0  0 ]     // nose up			ACC_PX
 *                        | -g  0  0 |     // nose down			ACC_NX
 *                        |  0  g  0 |     // left side down	ACC_PY
 *                        |  0 -g  0 |     // right side down	ACC_NY
 *                        |  0  0  g |     // on back			ACC_PZ
 *                        [  0  0 -g ]     // level
 * ACC_NZ accel_raw_ref[6][3]
 *
 * accel_corr_ref[i] = accel_T * (accel_raw_ref[i] - accel_offs), i = 0...5
 *
 * 6 reference vectors * 3 axes = 18 equations
 * 9 (accel_T) + 3 (accel_offs) = 12 unknown constants
 *
 * Find accel_offs
 *
 * accel_offs[i] = (accel_raw_ref[i*2][i] + accel_raw_ref[i*2+1][i]) / 2
 *
 * Find accel_T
 *
 * 9 unknown constants
 * need 9 equations -> use 3 of 6 measurements -> 3 * 3 = 9 equations
 *
 * accel_corr_ref[i*2] = accel_T * (accel_raw_ref[i*2] - accel_offs), i = 0...2
 *
 * Solve separate system for each row of accel_T:
 *
 * accel_corr_ref[j*2][i] = accel_T[i] * (accel_raw_ref[j*2] - accel_offs), j =
 * 0...2
 *
 * A * x = b
 *
 * x = [ accel_T[0][i] ]
 *     | accel_T[1][i] |
 *     [ accel_T[2][i] ]
 *
 * b = [ accel_corr_ref[0][i] ]	// One measurement per side is enough
 *     | accel_corr_ref[2][i] |
 *     [ accel_corr_ref[4][i] ]
 *
 * a[i][j] = accel_raw_ref[i][j] - accel_offs[j], i = 0;2;4, j = 0...2
 *
 * Matrix A is common for all three systems:
 * A = [ a[0][0]  a[0][1]  a[0][2] ]
 *     | a[2][0]  a[2][1]  a[2][2] |
 *     [ a[4][0]  a[4][1]  a[4][2] ]
 *
 * x = A^-1 * b
 *
 * accel_T = A^-1 * g
 */
#include "AccelCali.hpp"

#include "StmLog.hpp"

#include "Matrix.hpp"

#include <cmath>

using namespace INS_SYS;

static constexpr float ORIENTATION_THRESHOLD = 0.7f;
static constexpr float ACCEL_STATIONARY_THRESHOLD = 0.03f;
static constexpr float GYRO_STATIONARY_THRESHOLD = 8e-4f;
static constexpr uint32_t STATIONARY_CHECK_TIME_MS = 1000;
constexpr uint16_t SAMPLE_COUNT = 2000; // Collect for 2 seconds per side

AccelCali &AccelCali::instance()
{
    static AccelCali instance;
    return instance;
}

AccelCali::AccelCali() { startCalibration(); }

void AccelCali::startCalibration()
{
    side_ = 1;
    collectCnt_ = 0;
    memset(refData_, 0, sizeof(float) * 6 * 3);
    stationaryData_ = StationaryData_s();

    LOG::info(
            "AccelCali",
            "now you need to put the sensor on the table and keep it stationary");

    LOG::info{ "AccelCali", "the needed side is %s",
               ORIENTATION_STR[side_].second };
}

void AccelCali::update(int16_t _ax, int16_t _ay, int16_t _az, int16_t _gx,
                       int16_t _gy, int16_t _gz, float _aTransK, float _gTransK)
{
    imuData_.ax = static_cast<float>(_ax) * _aTransK;
    imuData_.ay = static_cast<float>(_ay) * _aTransK;
    imuData_.az = static_cast<float>(_az) * _aTransK;
    imuData_.gx = static_cast<float>(_gx) * _gTransK;
    imuData_.gy = static_cast<float>(_gy) * _gTransK;
    imuData_.gz = static_cast<float>(_gz) * _gTransK;

    if (isStationary() && isCorrectOrientation()) {
        if (collectData()) {
            if (side_ < 6) {
                side_++;
                LOG::info{ "AccelCali", "the needed side is %s",
                           ORIENTATION_STR[side_].second };
            } else {
                calculateCalibrationParams();
                saveCalibrationParams();
                LOG::info("AccelCali", "calibration finished");
                startCalibration(); // restart
            }
        }
    }
}

bool AccelCali::isStationary()
{
    stationaryData_.accelSamples[stationaryData_.index][0] = imuData_.ax;
    stationaryData_.accelSamples[stationaryData_.index][1] = imuData_.ay;
    stationaryData_.accelSamples[stationaryData_.index][2] = imuData_.az;
    stationaryData_.gyroSamples[stationaryData_.index][0] = imuData_.gx;
    stationaryData_.gyroSamples[stationaryData_.index][1] = imuData_.gy;
    stationaryData_.gyroSamples[stationaryData_.index][2] = imuData_.gz;

    stationaryData_.index =
            (stationaryData_.index + 1) % STATIONART_DETECTION_BUFFER_SIZE;
    if (!stationaryData_.isFull && stationaryData_.index == 0) {
        stationaryData_.isFull = true;
    }

    if (!stationaryData_.isFull) {
        return false;
    }

    float accelMean[3] = { 0, 0, 0 };
    float accelVariance[3] = { 0, 0, 0 };
    float gyroMean[3] = { 0, 0, 0 };
    float gyroVariance[3] = { 0, 0, 0 };

    // calc accelMean
    for (auto &accelSample : stationaryData_.accelSamples) {
        for (uint8_t j = 0; j < 3; j++) {
            accelMean[j] += accelSample[j];
        }
    }
    for (float &i : accelMean) {
        i /= STATIONART_DETECTION_BUFFER_SIZE;
    }

    // calc accelVariance
    for (auto &accelSample : stationaryData_.accelSamples) {
        for (int j = 0; j < 3; j++) {
            float diff = accelSample[j] - accelMean[j];
            accelVariance[j] += diff * diff;
        }
    }

    for (float &i : accelVariance) {
        i /= 10.0f;
    }

    // calc gyroMean
    for (auto &gyroSample : stationaryData_.gyroSamples) {
        for (uint8_t j = 0; j < 3; j++) {
            gyroMean[j] += gyroSample[j];
        }
    }

    for (float &i : gyroMean) {
        i /= STATIONART_DETECTION_BUFFER_SIZE;
    }

    // calc gyroVariance
    for (auto &gyroSample : stationaryData_.gyroSamples) {
        for (int j = 0; j < 3; j++) {
            float diff = gyroSample[j] - gyroMean[j];
            gyroVariance[j] += diff * diff;
        }
    }

    for (float &i : gyroVariance) {
        i /= STATIONART_DETECTION_BUFFER_SIZE;
    }

    bool accelStationary = true;
    bool gyroStationary = true;

    for (int j = 0; j < 3; j++) {
        if (accelVariance[j] > ACCEL_STATIONARY_THRESHOLD) {
            accelStationary = false;
        }

        if (gyroVariance[j] > GYRO_STATIONARY_THRESHOLD) {
            gyroStationary = false;
        }
    }

    return accelStationary && gyroStationary;
}

bool AccelCali::isCorrectOrientation()
{
    Orientation_e currOrientation = Orientation_e::UNKOWN;
    float magnitude =
            sqrtf((imuData_.ax * imuData_.ax) + (imuData_.ay * imuData_.ay) +
                  (imuData_.az * imuData_.az));
    float normAx = imuData_.ax / magnitude;
    float normAy = imuData_.ay / magnitude;
    float normAz = imuData_.az / magnitude;

    if (normAx > ORIENTATION_THRESHOLD) {
        currOrientation = Orientation_e::X_UP;
    } else if (normAx < -ORIENTATION_THRESHOLD) {
        currOrientation = Orientation_e::X_DOWN;
    } else if (normAy > ORIENTATION_THRESHOLD) {
        currOrientation = Orientation_e::Y_UP;
    } else if (normAy < -ORIENTATION_THRESHOLD) {
        currOrientation = Orientation_e::Y_DOWN;
    } else if (normAz > ORIENTATION_THRESHOLD) {
        currOrientation = Orientation_e::Z_UP;
    } else if (normAz < -ORIENTATION_THRESHOLD) {
        currOrientation = Orientation_e::Z_DOWN;
    }

    return static_cast<Orientation_e>(side_) == currOrientation;
}

bool AccelCali::collectData()
{
    if (collectCnt_ >= SAMPLE_COUNT) {
        refData_[side_ - 1][0] /= SAMPLE_COUNT;
        refData_[side_ - 1][1] /= SAMPLE_COUNT;
        refData_[side_ - 1][2] /= SAMPLE_COUNT;
        collectCnt_ = 0;
        LOG::info("AccelCali", "Collecting data for side %d, 100%% done",
                  side_);
        return true;
    } else {
        refData_[side_ - 1][0] += imuData_.ax;
        refData_[side_ - 1][1] += imuData_.ay;
        refData_[side_ - 1][2] += imuData_.az;
        collectCnt_++;
        if (collectCnt_ == static_cast<uint16_t>(SAMPLE_COUNT * 0.25f)) {
            LOG::info("AccelCali", "Collecting data for side %d, 25%% done",
                      side_);
        } else if (collectCnt_ == static_cast<uint16_t>(SAMPLE_COUNT * 0.5f)) {
            LOG::info("AccelCali", "Collecting data for side %d, 50%% done",
                      side_);
        } else if (collectCnt_ == static_cast<uint16_t>(SAMPLE_COUNT * 0.75f)) {
            LOG::info("AccelCali", "Collecting data for side %d, 75%% done",
                      side_);
        }
        return false;
    }
}

void AccelCali::calculateCalibrationParams()
{
    // calc offset
    for (uint8_t i = 0; i < 3; i++) {
        caliParams_.accelOffs[i] =
                (refData_[i * 2][i] + refData_[(i * 2) + 1][i]) / 2.0f;
    }
    // NOLINTNEXTLINE
    Matrix<3, 3> A;
    float b[3]{};

    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            A[i][j] = refData_[i * 2][j] - caliParams_.accelOffs[j];
        }
    }

    Matrix<3, 3> invA = A.inv();

    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            b[j] = (j == i) ? G : 0.0f;
        }

        // x = A^{-1} * b
        for (uint8_t j = 0; j < 3; j++) {
            caliParams_.accelT[i][j] = 0;
            for (uint8_t k = 0; k < 3; k++) {
                caliParams_.accelT[i][j] += invA[j][k] * b[k];
            }
        }
    }

    LOG::info("AccelCali", ".accelT =  { %.6ff, %.6ff, %.6ff },",
              caliParams_.accelT[0][0], caliParams_.accelT[0][1],
              caliParams_.accelT[0][2]);
    LOG::info("AccelCali", "            { %.6ff, %.6ff, %.6ff },",
              caliParams_.accelT[1][0], caliParams_.accelT[1][1],
              caliParams_.accelT[1][2]);
    LOG::info("AccelCali", "            { %.6ff, %.6ff, %.6ff }",
              caliParams_.accelT[2][0], caliParams_.accelT[2][1],
              caliParams_.accelT[2][2]);
    LOG::info("AccelCali", ".accelOffs = { %.6ff, %.6ff, %.6ff }",
              caliParams_.accelOffs[0], caliParams_.accelOffs[1],
              caliParams_.accelOffs[2]);
}

void AccelCali::saveCalibrationParams()
{
    // TODO: save to flash or EEPROM
}
