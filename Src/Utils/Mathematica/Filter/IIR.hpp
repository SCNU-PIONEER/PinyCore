#pragma once

namespace FILTER {

class IIR2 {
    static constexpr float NUM_2[3] = { 0.02785976604f, 0.05571953207f,
                                        0.02785976604f };
    static constexpr float DEN_2[3] = { 1.f, -1.475480437f, 0.5869194865f };

public:
    float process(float _in)
    {
        float out = (NUM_2[0] * _in) + (NUM_2[1] * x_[0]) + (NUM_2[2] * x_[1]) -
                    (DEN_2[1] * y_[0]) - (DEN_2[2] * y_[1]);

        x_[1] = x_[0];
        x_[0] = _in;
        y_[1] = y_[0];
        y_[0] = out;

        return out;
    }

private:
    float x_[2]{}, y_[2]{};
};

class IIR3 {
    static constexpr float NUM_3[4] = { 0.001567010302f, 0.004701030906f,
                                        0.004701030906f, 0.001567010302f };
    static constexpr float DEN_3[4] = { 1.f, -2.498608351f, 2.115254164f,
                                        -0.6041097045f };

public:
    float process(float _in)
    {
        float out = (NUM_3[0] * _in) + (NUM_3[1] * x_[0]) + (NUM_3[2] * x_[1]) +
                    (NUM_3[3] * x_[2]) - (DEN_3[1] * y_[0]) -
                    (DEN_3[2] * y_[1]) - (DEN_3[3] * y_[2]);

        x_[2] = x_[1];
        x_[1] = x_[0];
        x_[0] = _in;

        y_[2] = y_[1];
        y_[1] = y_[0];
        y_[0] = out;

        return out;
    }

private:
    float x_[3]{}, y_[3]{};
};

} // namespace FILTER
