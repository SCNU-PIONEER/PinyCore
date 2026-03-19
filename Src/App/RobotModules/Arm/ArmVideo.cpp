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

    pitch.setDutyCycle(pitchAng);
    yaw.setDutyCycle(yawAng);

    LOG::info(TAG, "init");
}

void VideoCtrl::loop(const uint32_t _pitch, const uint32_t _yaw)
{
    uint32_t pitchAngScale = 100;
    pitchAng = std::clamp((pitchAng + _pitch * pitchAngScale), PITCH_ANG_MIN, PITCH_ANG_MAX);
    yawAng = std::clamp((yawAng + _yaw), YAW_ANG_MIN, YAW_ANG_MAX);
    
    pitch.setDutyCycle(pitchAng);
    yaw.setDutyCycle(yawAng);
}

void VideoCtrl::stop()
{
    pitch.stop();
    yaw.stop();

    LOG::info(TAG, "stop");
}
