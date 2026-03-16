#include "ArmVideo.hpp"
#include "StmLog.hpp"
#include "main.h"

using namespace ARM;

static constexpr const char *TAG = "Video";

VideoCtrl::VideoCtrl() : pitch(&htim2, TIM_CHANNEL_1), yaw(&htim2, TIM_CHANNEL_3) {}

uint32_t pitchang = 1000;
uint32_t yawang = 1000;

struct Ang {
    uint32_t pitch = 1500;
    uint32_t yaw = 1460;
} ang1;

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
    pitch.setDutyCycle(ang1.pitch);
    yaw.setDutyCycle(ang1.yaw);

    // pitch.setDutyCycle(_pitch);
    // yaw.setDutyCycle(_yaw);
}

void VideoCtrl::stop()
{
    pitch.stop();
    yaw.stop();

    LOG::info(TAG, "stop");
}
