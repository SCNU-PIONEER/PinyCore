#pragma once

#include "dsp/transform_functions.h"
#include "StmLog.hpp"

template <uint16_t Size> class FFT {
public:
    FFT() = default;
    FFT(const char _name[16], float _sampleRate) : fftSize(Size), sampleRate(_sampleRate)
    {
        snprintf(name, sizeof(name), "%s:\n", _name);
        arm_rfft_fast_init_f32(&rfftInstance, fftSize);
    }
    ~FFT() = default;
    void ftProcess(float _inputData)
    {
        if (dataIndex < fftSize) {
            timeData[dataIndex] = _inputData;
            dataIndex++;
        } else {
            dataIndex = 0;
            SEGGER_RTT_WriteString(0, name);
            for (uint16_t i = 0; i < fftSize; i++)
                infoLog(timeData[i]);
            SEGGER_RTT_WriteString(0, "end\n");

            arm_rfft_fast_f32(&rfftInstance, timeData, freqData, 0);
            arm_cmplx_mag_f32(freqData, magnitudeData, fftSize / 2);
            for (uint16_t i = 0; i < fftSize / 2; i++)
                infoLog(magnitudeData[i]);
            SEGGER_RTT_WriteString(0, "end\n");
        }
    };

    void ftProcess(const float _inputData[Size])
    {
        arm_rfft_fast_f32(&rfftInstance, _inputData, freqData, 0);
        arm_cmplx_mag_f32(freqData, magnitudeData, fftSize / 2);
        SEGGER_RTT_WriteString(0, name);
        for (uint16_t i = 0; i < fftSize / 2; i++)
            infoLog(magnitudeData[i]);
        SEGGER_RTT_WriteString(0, "end\n");
    };

    void infoLog(float _data)
    {
        uint16_t len = snprintf(buffer, sizeof(buffer), "%.4f,", _data);
        LOG::Logger::instance().raw(buffer, len);
    }

private:
    arm_rfft_fast_instance_f32 rfftInstance;
    float timeData[Size] = {};
    float freqData[Size] = {};
    float magnitudeData[Size / 2] = {};

    uint16_t fftSize = 0;
    uint16_t dataIndex = 0;
    float sampleRate = 1000.0f; //抽样频率，即电机反馈频率

    char name[16] = "NULL";
    char buffer[16] = {};
};