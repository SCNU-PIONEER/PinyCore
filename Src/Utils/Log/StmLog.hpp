#pragma once

#include "sdkconfig.h"
#if LOG_OUTPUT_RTT
#include "Output/RttOutput.hpp"
#endif
#if LOG_OUTPUT_UART
#include "Output/UartOutput.hpp"
#endif
#include "StmLogMsg.hpp"
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string_view>
#include <utility>
#include "Singleton.hpp"

#define LOCATION std::source_location::current()

namespace LOG {

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

    void raw(const uint8_t *_data, size_t _size) { (void)send(_data, _size); }

    template <typename... Args> void raw(const char *_format, Args &&..._args)
    {
        constexpr size_t MAX_RAW_LENGTH = 128;
        char buffer[MAX_RAW_LENGTH];
        const int length = snprintf(buffer, sizeof(buffer), _format, std::forward<Args>(_args)...);
        if (length <= 0) {
            return;
        }
        const size_t size = std::min(static_cast<size_t>(length), sizeof(buffer) - 1);
        raw(reinterpret_cast<const uint8_t *>(buffer), size);
    }

    template <typename... Args>
    bool info(std::source_location _loc, std::string_view _type, const char *_format, Args &&..._args)
    {
        return log(LogParams{ .loc = _loc, .type = _type, .format = _format, .level = Level::INFO },
                   std::forward<Args>(_args)...);
    }

    template <typename... Args>
    bool debug(std::source_location _loc, std::string_view _type, const char *_format, Args &&..._args)
    {
        return log(LogParams{ .loc = _loc, .type = _type, .format = _format, .level = Level::DEBUGGING },
                   std::forward<Args>(_args)...);
    }

    template <typename... Args>
    bool warn(std::source_location _loc, std::string_view _type, const char *_format, Args &&..._args)
    {
        return log(LogParams{ .loc = _loc, .type = _type, .format = _format, .level = Level::WARN },
                   std::forward<Args>(_args)...);
    }

    template <typename... Args>
    bool error(std::source_location _loc, std::string_view _type, const char *_format, Args &&..._args)
    {
        return log(LogParams{ .loc = _loc, .type = _type, .format = _format, .level = Level::ERROR },
                   std::forward<Args>(_args)...);
    }

    /**
    * @brief 完美转发检验错误
    */
    template <typename Func> void check(std::source_location _loc, Func &&_operation)
    {
        stm_err_t err = _operation();
        if (unlikely(err != 0)) {
            error(_loc, "check", "error code: %d", err);
            while (true) {
            }
        }
    }

    /**
    * @brief 清屏
    */
    void clear();

    /**
    * @brief 浮点数转字符串
    */
    void float2Str(char *_str, size_t _buffer_size, float _va);

    void disable() { config.enable = false; }

    void enable() { config.enable = true; }

    void setLevel(Level _level) { config.level = _level; }

    void setColor(bool _enable) { config.showColor = _enable; }

    void setLocation(bool _enable) { config.showlocation = _enable; }

    void setName(std::string_view _name) { config.name = _name; }

    void setProto(Proto _proto) { config.proto = _proto; }

    void setConfig(const Config &_config) { config = _config; }

    /**
    * @brief 完美转发打印函数,自带换行
    */
    template <typename... Args> bool log(const LogParams &_params, Args &&..._args)
    {
        if (!config.enable)
            return false;

        constexpr size_t MAX_LOG_LENGTH = 128;
        char buffer[MAX_LOG_LENGTH];
        char *ptr = buffer;
        const char *end = buffer + MAX_LOG_LENGTH;

        // 写入颜色控制码（如果启用）
        if (config.showColor) [[likely]] {
            auto color = getLevelColor(_params.level);
            size_t len = std::min(color.size(), static_cast<size_t>(end - ptr));
            memcpy(ptr, color.data(), len);
            ptr += len;
        }

        // 写入位置信息（如果启用）
        if (config.showlocation) [[likely]] {
            std::string_view file(_params.loc.file_name());
            if (auto pos = file.find_last_of("/\\"); pos != std::string_view::npos) {
                file = file.substr(pos + 1);
            }
            size_t len = snprintf(ptr, end - ptr, " [%.*s:%ld]: ", static_cast<int>(file.size()), file.data(),
                                  _params.loc.line());
            ptr += std::min(len, static_cast<size_t>(end - ptr));
        }

        // 写入日志类型
        size_t len = std::min(_params.type.size(), static_cast<size_t>(end - ptr));
        memcpy(ptr, _params.type.data(), len);
        ptr += len;

        // 写入分隔符
        len = std::min(sizeof(": ") - 1, static_cast<size_t>(end - ptr));
        memcpy(ptr, ": ", len);
        ptr += len;

        if (_params.level == Level::RAW) {
            len = snprintf(ptr, end - ptr, _params.format, std::forward<Args>(_args)...);
            ptr += std::min(len, static_cast<size_t>(end - ptr));
        } else {
            len = snprintf(ptr, end - ptr, _params.format, std::forward<Args>(_args)...);
            ptr += std::min(len, static_cast<size_t>(end - ptr));
        }

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
#if LOG_OUTPUT_RTT
    RttOutput rttOutput_;
#endif
#if LOG_OUTPUT_UART
    UartOutput uartOutput_;
#endif

    Config config;
};

//  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ some preset ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// NOLINTBEGIN
template <typename... Args> struct info {
    constexpr info(std::string_view _type, const char *_format, Args &&..._args,
                   std::source_location _loc = std::source_location::current())
    {
        Logger::instance().log(LogParams{ .loc = _loc, .type = _type, .format = _format, .level = Level::INFO },
                               std::forward<Args>(_args)...);
    }
};
template <typename... Args> info(std::string_view _type, const char *_format, Args &&..._args) -> info<Args...>;

template <typename... Args> struct warn {
    constexpr warn(std::string_view _type, const char *_format, Args &&..._args,
                   std::source_location _loc = std::source_location::current())
    {
        Logger::instance().log(LogParams{ .loc = _loc, .type = _type, .format = _format, .level = Level::WARN },
                               std::forward<Args>(_args)...);
    }
};
template <typename... Args> warn(std::string_view _type, const char *_format, Args &&..._args) -> warn<Args...>;

template <typename... Args> struct error {
    constexpr error(std::string_view _type, const char *_format, Args &&..._args,
                    std::source_location _loc = std::source_location::current())
    {
        Logger::instance().log(LogParams{ .loc = _loc, .type = _type, .format = _format, .level = Level::ERROR },
                               std::forward<Args>(_args)...);
    }
};
template <typename... Args> error(std::string_view _type, const char *_format, Args &&..._args) -> error<Args...>;

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
