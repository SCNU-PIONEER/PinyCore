#pragma once

#define T_ACC_CNT 100

static inline float s_curve(float v_max, float cnt)
{
    float cntcnt;
    if (cnt < 0.0f) {
        cnt = -cnt;
        v_max = -v_max;
    }
    cntcnt = cnt / ((float)T_ACC_CNT);
    if (cnt < T_ACC_CNT / 2.0f) {
        return 2.0f * v_max * (cntcnt * cntcnt);
    } else if (cnt < T_ACC_CNT) {
        cntcnt = cntcnt - 1.0f;
        return v_max * (1.0f - 2.0f * (cntcnt * cntcnt));
    } else
        return v_max;
}
