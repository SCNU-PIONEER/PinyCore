#pragma once

#define MAX_FILTER_CH_2        4
#define MAX_FILTER_CH_3        4
#define MAX_FILTER_CH_5        3
#define MAX_ONE_EURO_FILTER_CH 3

const float NUM_3[4] = { 0.001567010302, 0.004701030906, 0.004701030906,
                         0.001567010302 };
const float DEN_3[4] = { 1, -2.498608351, 2.115254164, -0.6041097045 };

inline float iir_filter_3(float x, unsigned int ch)
{
    static float y3, x3_n1[MAX_FILTER_CH_3], x3_n2[MAX_FILTER_CH_3],
            x3_n3[MAX_FILTER_CH_3], y3_n1[MAX_FILTER_CH_3],
            y3_n2[MAX_FILTER_CH_3], y3_n3[MAX_FILTER_CH_3];
    y3 = NUM_3[0] * x + NUM_3[1] * x3_n1[ch] + NUM_3[2] * x3_n2[ch] +
         NUM_3[3] * x3_n3[ch] - DEN_3[1] * y3_n1[ch] - DEN_3[2] * y3_n2[ch] -
         DEN_3[3] * y3_n3[ch];
    // if(y<EPS && y>-EPS) y = 0.0;
    y3_n3[ch] = y3_n2[ch];
    y3_n2[ch] = y3_n1[ch];
    y3_n1[ch] = y3;
    x3_n3[ch] = x3_n2[ch];
    x3_n2[ch] = x3_n1[ch];
    x3_n1[ch] = x;
    return y3;
}
