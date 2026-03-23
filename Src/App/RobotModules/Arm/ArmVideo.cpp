#include "ArmVideo.hpp"
#include "StmLog.hpp"
#include "main.h"
#include <algorithm>
#include <cstdint>

using namespace ARM;

static constexpr const char *TAG = "VideoCtrl";

VideoCtrl::VideoCtrl() : pitch(&htim2, TIM_CHANNEL_1), yaw(&htim2, TIM_CHANNEL_3) {}

// uint32_t pitchang = 1000;
// uint32_t yawang = 1000;
//
// struct Ang {
//     uint32_t pitch = 1500;
//     uint32_t yaw = 1460;
// } ang1;

void VideoCtrl::init()
{
    pitch.start();
    yaw.start();

    pitch.setDutyCycle(PITCH_RESET_ANG);
    yaw.setDutyCycle(YAW_RESET_ANG);

    LOG::info(TAG, "init");
}

void VideoCtrl::loop(const float _pitch, const float _yaw)
{
    // -1.33 - 0
    // 1600 - 2400
    // float scale1 = (_pitch - (-1.33f)) / (0 - (-1.33f));
    // pitchAng = static_cast<uint32_t>(PITCH_ANG_MIN + scale1 * (PITCH_ANG_MAX - PITCH_ANG_MIN));
    // pitchAng = std::clamp(pitchAng, PITCH_ANG_MIN, PITCH_ANG_MAX);

    pitchAng = (uint32_t)((float)pitchAng + _pitch * 5);
    pitchAng = std::clamp(pitchAng, PITCH_ANG_MIN, PITCH_ANG_MAX);

    yawAng = (uint32_t)((float)yawAng + _yaw * 5);
    yawAng = std::clamp(yawAng, YAW_ANG_MIN, YAW_ANG_MAX);

    pitch.setDutyCycle(pitchAng);
    yaw.setDutyCycle(yawAng);
}

void VideoCtrl::stop()
{
    pitch.stop();
    yaw.stop();

    LOG::info(TAG, "stop");
}
