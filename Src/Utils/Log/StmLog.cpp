#include <stdio.h>
#include <cmath>
#include <cstdlib>
#include "StmLog.hpp"

using namespace LOG;

void Logger::clear()
{
    constexpr char CLR_SEQ[] = "  \x1B[2J";
    send(reinterpret_cast<const uint8_t *>(CLR_SEQ), sizeof(CLR_SEQ) - 1);
}

void Logger::float2Str(char *_str, size_t _buffer_size, float _va)
{
    // 处理符号
    int flag = _va < 0;
    _va = fabsf(_va);

    // 四舍五入到小数点后3位
    float rounded = roundf(_va * 1000) / 1000;

    int head = (int)rounded;
    int point = (int)((rounded - (float)head) * 1000);

    // 确保point在0-999范围内
    point = abs(point) % 1000;

    if (flag) {
        snprintf(_str, _buffer_size, "-%d.%03d", head, point);
    } else {
        snprintf(_str, _buffer_size, "%d.%03d", head, point);
    }
}
