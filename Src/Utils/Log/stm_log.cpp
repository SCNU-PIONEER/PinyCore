#include <stdio.h>
#include <cmath>
#include <cstdlib>
#include "stm_log.hpp"

namespace LOG {

void Logger::clear() { SEGGER_RTT_WriteString(0, "  " RTT_CTRL_CLEAR); }

void Logger::Float2Str(char *str, size_t buffer_size, float va)
{
    // 处理符号
    int flag = va < 0;
    va = fabsf(va);

    // 四舍五入到小数点后3位
    float rounded = roundf(va * 1000) / 1000;

    int head = (int)rounded;
    int point = (int)((rounded - head) * 1000);

    // 确保point在0-999范围内
    point = abs(point) % 1000;

    if (flag) {
        snprintf(str, buffer_size, "-%d.%03d", head, point);
    } else {
        snprintf(str, buffer_size, "%d.%03d", head, point);
    }
}

}
