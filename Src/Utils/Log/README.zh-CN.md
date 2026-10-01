# Piny Log

# 指标
1. 基本功能：输出信息,可变参数（c风格), 自动换行
2. 可选性：颜色、文件定位、等级、时间戳
3. 安全性：线程安全, 类型检查
4. 拓展性: 支持 RTT, UART, FireWater, JustFloat

# example

```cpp
    LOG::Logger &log = LOG::Logger::instance();

    uint8_t i = 10;
    std::string str = "PinyCore init start.";

    log.raw("Piny");

    log.info("Piny", "num i: %d", i);
    log.info("Piny", "str: %s", str.c_str());

    // 便携的调用方式
    LOG::info("Piny", "This is a info.");
    LOG::warn("Piny", "This is a warn.");
    LOG::error("Piny", "This is a error.");
    LOG::debug("Piny", "This is a debug.");

    // VOFA+ 波形协议
    LOG::fireWater(roll, yaw, pitch);   // FireWater 文本帧
    LOG::justFloat(roll, yaw, pitch);   // JustFloat 二进制帧
```

## 文本日志

| API | 说明 |
|---|---|
| `LOG::info(type, fmt, ...)` | INFO 级日志 |
| `LOG::debug(type, fmt, ...)` | DEBUG 级日志 |
| `LOG::warn(type, fmt, ...)` | WARN 级日志 |
| `LOG::error(type, fmt, ...)` | ERROR 级日志 |
| `LOG::raw(fmt, ...)` | 原始文本输出，无等级 |
| `LOG::CHECK(cond)` | 条件检查，失败记 error 并挂起（while(true)） |

等级颜色（`LOG_SHOW_COLOR` 开启时）：不同等级日志展现不同颜色。

## FireWater（VOFA+ 文本波形帧）

```cpp
LOG::fireWater(ch0, ch1, ...);   // 帧: "ch0,ch1,...,chN\n"
```

- 使用固定栈缓冲（可通过menuconfig配置） `MAX_RAW_LENGTH`，超长截断但仍保留末尾换行；

## JustFloat（VOFA+ 二进制波形帧）

```cpp
LOG::justFloat(ch0, ch1, ...);   // N 个小端序 float + 帧尾
```

- 浮点不丢精度、适合高精度波形，性能稍逊于FireWater；

# 输出后端

所有输出统一经过 `Logger::raw` -> `Logger::send`，写入已启用的后端（RTT / UART）。

## RTT
SEGGER RTT 输出，汇编级锁，线程安全；单例确保全局唯一；单次调用单次 write，不会串行。

### 多缓冲区

```cpp
SEGGER_RTT_ConfigUpBuffer(1, "Performance", PerfBuffer, sizeof(PerfBuffer),
                          SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
```

## UART
`UartOutput`：UART + DMA 环形缓冲（4 x 256B），原子状态机管理发送；中断回调推进缓冲状态，发送失败返回 `false`。

# 特性

## 安全性
### 线程安全
1. RTT提供汇编级的锁机制
2. 单例模式确保全局唯一
3. 提供分线程多缓冲区
4. 单次调用write，不会出现串行

# 边缘情况
- 协议通道统一转 `float`：传 `double`/`int` 会截断精度；
- FireWater 文本帧受 `MAX_RAW_LENGTH` 限制，超长截断（仍保留 `\n`）；
- JustFloat 帧长 `N*4+4`，需确认后端（RTT / UART DMA 缓冲）能一次容纳整帧；

# Reference
[Segger RTT](https://kb.segger.com/RTT)
