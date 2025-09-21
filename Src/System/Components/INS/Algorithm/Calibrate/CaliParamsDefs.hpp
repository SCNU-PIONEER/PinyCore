#pragma once

namespace INS_SYS {

typedef struct {
    float accelT[3][3];
    float accelOffs[3];
} AccCaliParams_s;

typedef struct {
    float gxBias, gyBias, gzBias;
    float gxTcoK, gxTcoB0;
    float gyTcoK, gyTcoB0;
    float gzTcoK, gzTcoB0;
} GyroCaliParams_s;

} // namespace INS_SYS