#pragma once

#include "sdkconfig.h"
#if LOG_OUTPUT_RTT
#include "Output/RttOutput.hpp"
#endif
#if LOG_OUTPUT_UART
#include "Output/UartOutput.hpp"
#endif
#include "StmLogMsg.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string_view>
#include <utility>
#include "Singleton.hpp"

#define LOCATION std::source_location::current()

namespace LOG {

constexpr size_t MAX_RAW_LENGTH = LOG_MAX_RAW_LENGTH;
constexpr size_t MAX_LOG_LENGTH = LOG_MAX_LOG_LENGTH;

class Logger : public Singleton<Logger> {
public:
    bool init()
    {
        bool rslt = false;
#if LOG_OUTPUT_RTT
        rslt |= rttOutput_.init();
#endif
#if LOG_OUTPUT_UART
        rslt |= uartOutput_.init();
#endif
        return rslt;
    }

    bool send(const uint8_t *_data, size_t _size)
    {
        bool rslt = false;
#if LOG_OUTPUT_RTT
        rslt |= rttOutput_.send(_data, _size);
#endif
#if LOG_OUTPUT_UART
        rslt |= uartOutput_.send(_data, _size);
#endif
        return rslt;
    }

    bool raw(const uint8_t *_data, size_t _size) { return send(_data, _size); }

    template <typename... Args> bool raw(const char *_format, Args &&..._args)
    {
        char buffer[MAX_RAW_LENGTH];
        const int length = snprintf(buffer, sizeof(buffer), _format, std::forward<Args>(_args)...);
        const size_t size = std::min(static_cast<size_t>(length), sizeof(buffer) - 1);
        return raw(reinterpret_cast<const uint8_t *>(buffer), size);
    }

    template <typename... Args>
    bool info(std::source_location _loc, std::string_view _type, const char *_format, Args &&..._args)
    {
        return log(LogParams{ .loc = _loc, .type = _type, .format = _format, .level = Level_e::INFO },
                   std::forward<Args>(_args)...);
    }

    template <typename... Args>
    bool debug(std::source_location _loc, std::string_view _type, const char *_format, Args &&..._args)
    {
        return log(LogParams{ .loc = _loc, .type = _type, .format = _format, .level = Level_e::DEBUGGING },
                   std::forward<Args>(_args)...);
    }

    template <typename... Args>
    bool warn(std::source_location _loc, std::string_view _type, const char *_format, Args &&..._args)
    {
        return log(LogParams{ .loc = _loc, .type = _type, .format = _format, .level = Level_e::WARN },
                   std::forward<Args>(_args)...);
    }

    template <typename... Args>
    bool error(std::source_location _loc, std::string_view _type, const char *_format, Args &&..._args)
    {
        return log(LogParams{ .loc = _loc, .type = _type, .format = _format, .level = Level_e::ERROR },
                   std::forward<Args>(_args)...);
    }

    template <typename Func> void check(std::source_location _loc, Func &&_operation)
    {
        stm_err_t err = std::forward<Func>(_operation)();
        if (unlikely(err != 0)) {
            error(_loc, "check", "error code: %d", err);
            while (true) {
            }
        }
    }

    /**
     * @brief FireWater protocol convenience overload using a fixed stack buffer.
     */
    template <typename... Args> void fireWater(Args &&..._channels)
    {
        char buffer[MAX_RAW_LENGTH];
        const size_t size = fireWater(buffer, sizeof(buffer), std::forward<Args>(_channels)...);
        Logger::instance().raw(reinterpret_cast<const uint8_t *>(buffer), size);
    }

    /**
     * @brief JustFloat protocol: N little-endian floats followed by the tail
     *        {0x00, 0x00, 0x80, 0x7f}.
     */
    template <typename... Args> void justFloat(Args &&..._channels)
    {
        constexpr size_t CHANNEL_COUNT = sizeof...(Args);
        constexpr size_t TAIL_SIZE = 4;
        constexpr size_t FRAME_SIZE = (CHANNEL_COUNT * sizeof(float)) + TAIL_SIZE;

        std::array<uint8_t, FRAME_SIZE> frame{};

        size_t offset = 0;
        auto writeChannel = [&](float _value) {
            writeFloatLE(frame.data() + offset, _value);
            offset += sizeof(float);
        };

        (writeChannel(static_cast<float>(std::forward<Args>(_channels))), ...);

        constexpr uint8_t TAIL[TAIL_SIZE] = { 0x00, 0x00, 0x80, 0x7f };
        std::memcpy(frame.data() + offset, TAIL, sizeof(TAIL));

        Logger::instance().raw(frame.data(), frame.size());
    }

    __always_inline void rawData(const uint8_t *_data, size_t _size) { Logger::instance().raw(_data, _size); }

    void clear();

    void float2Str(char *_str, size_t _buffer_size, float _va);

    template <typename... Args> bool log(const LogParams &_params, Args &&..._args)
    {
        char buffer[MAX_LOG_LENGTH];
        char *ptr = buffer;
        const char *end = buffer + MAX_LOG_LENGTH;
        size_t len;

#if LOG_SHOW_COLOR
        auto color = getLevelColor(_params.level);
        len = std::min(color.size(), static_cast<size_t>(end - ptr));
        memcpy(ptr, color.data(), len);
        ptr += len;
#endif

#if LOG_SHOW_LOCATION
        std::string_view file(_params.loc.file_name());
        if (auto pos = file.find_last_of("/\\"); pos != std::string_view::npos) {
            file = file.substr(pos + 1);
        }
        len = snprintf(ptr, end - ptr, "-%.*s:%ld", static_cast<int>(file.size()), file.data(), _params.loc.line());
        ptr += std::min(len, static_cast<size_t>(end - ptr));
#endif

#if LOG_SHOW_TIMESTAMP
        len = snprintf(ptr, end - ptr, "-%lu", xTaskGetTickCount());
        ptr += std::min(len, static_cast<size_t>(end - ptr));
#endif

        // log type
        len = snprintf(ptr, end - ptr, "-%.*s", static_cast<int>(_params.type.size()), _params.type.data());
        ptr += len;

        // + ": "
        len = std::min(sizeof(": ") - 1, static_cast<size_t>(end - ptr));
        memcpy(ptr, ": ", len);
        ptr += len;

        // message
        len = snprintf(ptr, end - ptr, _params.format, std::forward<Args>(_args)...);
        ptr += std::min(len, static_cast<size_t>(end - ptr));

        // + RST_SEQ
        constexpr char RST_SEQ[] = "\x1B[0m\r\n";
        len = std::min(sizeof(RST_SEQ) - 1, static_cast<size_t>(end - ptr));
        memcpy(ptr, RST_SEQ, len);
        ptr += len;

        size_t totalLen = ptr - buffer;

        return send(reinterpret_cast<const uint8_t *>(buffer), totalLen);
    }

protected:
    Logger(const Logger &);
    Logger &operator=(const Logger &);
    Logger() = default;
    friend class Singleton<Logger>;

private:
    /**
     * @brief FireWater protocol: CSV-style text frame "ch0,ch1,...,chN\\n".
     *        Writes into the caller-provided buffer and returns the written size.
     */
    template <typename... Args> size_t fireWater(char *_buffer, size_t _bufferSize, Args &&..._channels)
    {
        if (_bufferSize == 0) {
            return 0;
        }

        // Reserve the last byte for the mandatory newline.
        char *ptr = _buffer;
        char *const end = _buffer + _bufferSize - 1;

        bool first = true;
        auto append = [&](float _value) {
            if (ptr >= end) {
                return;
            }
            if (!first) {
                *ptr++ = ',';
                if (ptr >= end) {
                    return;
                }
            }
            first = false;

            const int written = this->appendFireWaterChannel(ptr, static_cast<size_t>(end - ptr), _value);
            if (written > 0) {
                ptr += std::min(static_cast<size_t>(written), static_cast<size_t>(end - ptr));
            }
        };

        (append(static_cast<float>(std::forward<Args>(_channels))), ...);

        *ptr++ = '\n';
        return static_cast<size_t>(ptr - _buffer);
    }

    __always_inline void writeFloatLE(uint8_t *_dst, float _value)
    {
        uint32_t bits = 0;
        static_assert(sizeof(bits) == sizeof(_value), "float must be 32-bit");
        std::memcpy(&bits, &_value, sizeof(bits));
        _dst[0] = static_cast<uint8_t>(bits & 0xFFu);
        _dst[1] = static_cast<uint8_t>((bits >> 8) & 0xFFu);
        _dst[2] = static_cast<uint8_t>((bits >> 16) & 0xFFu);
        _dst[3] = static_cast<uint8_t>((bits >> 24) & 0xFFu);
    }

    __always_inline int appendFireWaterChannel(char *_ptr, size_t _remaining, float _value)
    {
        return snprintf(_ptr, _remaining, "%g", _value);
    }

#if LOG_OUTPUT_RTT
    RttOutput rttOutput_;
#endif
#if LOG_OUTPUT_UART
    UartOutput uartOutput_;
#endif
};

//  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ some preset ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// NOLINTBEGIN
template <typename... Args> struct info {
    constexpr info(std::string_view _type, const char *_format, Args &&..._args,
                   std::source_location _loc = std::source_location::current())
    {
        Logger::instance().log(LogParams{ .loc = _loc, .type = _type, .format = _format, .level = Level_e::INFO },
                               std::forward<Args>(_args)...);
    }
};
template <typename... Args> info(std::string_view _type, const char *_format, Args &&..._args) -> info<Args...>;

template <typename... Args> struct warn {
    constexpr warn(std::string_view _type, const char *_format, Args &&..._args,
                   std::source_location _loc = std::source_location::current())
    {
        Logger::instance().log(LogParams{ .loc = _loc, .type = _type, .format = _format, .level = Level_e::WARN },
                               std::forward<Args>(_args)...);
    }
};
template <typename... Args> warn(std::string_view _type, const char *_format, Args &&..._args) -> warn<Args...>;

template <typename... Args> struct error {
    constexpr error(std::string_view _type, const char *_format, Args &&..._args,
                    std::source_location _loc = std::source_location::current())
    {
        Logger::instance().log(LogParams{ .loc = _loc, .type = _type, .format = _format, .level = Level_e::ERROR },
                               std::forward<Args>(_args)...);
    }
};
template <typename... Args> error(std::string_view _type, const char *_format, Args &&..._args) -> error<Args...>;

template <typename... Args> struct debug {
    constexpr debug(std::string_view _type, const char *_format, Args &&..._args,
                    std::source_location _loc = std::source_location::current())
    {
        Logger::instance().log(LogParams{ .loc = _loc, .type = _type, .format = _format, .level = Level_e::DEBUGGING },
                               std::forward<Args>(_args)...);
    }
};
template <typename... Args> debug(std::string_view _type, const char *_format, Args &&..._args) -> debug<Args...>;

template <typename... Args> struct raw {
    constexpr raw(const char *_format, Args &&..._args)
    {
        Logger::instance().raw(_format, std::forward<Args>(_args)...);
    }
};
template <typename... Args> raw(const char *_format, Args &&..._args) -> raw<Args...>; // support CTAD

template <typename... Args> struct fireWater {
    constexpr fireWater(Args &&..._channels) { Logger::instance().fireWater(std::forward<Args>(_channels)...); }
};
template <typename... Args> fireWater(Args &&..._channels) -> fireWater<Args...>;

template <typename... Args> struct justFloat {
    constexpr justFloat(Args &&..._channels) { Logger::instance().justFloat(std::forward<Args>(_channels)...); }
};
template <typename... Args> justFloat(Args &&..._channels) -> justFloat<Args...>;

template <typename T> void CHECK(T &&_condition, std::source_location _loc = std::source_location::current())
{
    if constexpr (std::is_invocable_v<T>) {
        /* 处理可调用对象 */
        Logger::instance().check(_loc, std::forward<T>(_condition));
    } else {
        /* 处理原始值 */
        Logger::instance().check(_loc, [&] { return !static_cast<bool>(_condition); });
    }
}
// NOLINTEND

} // namespace LOG
