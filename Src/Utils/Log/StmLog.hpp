#pragma once

#include "SEGGER_RTT.h"
#include "StmLogMsg.hpp"
#include <string_view>
#include <cstring>

namespace LOG {

#define LOCATION std::source_location::current()

#define LOG_PROTO(type, color, format, ...)                                    \
    SEGGER_RTT_printf(0, "  %s%s" format "\r\n%s", color, type, ##__VA_ARGS__, \
                      RTT_CTRL_RESET)

// #define ERROR_CHECK(x) check([&]() { return (x); }, __FILE__, __LINE__, #x)

class Logger {
public:
    inline static Logger &instance()
    {
        static Logger instance;
        return instance;
    }

    template <typename... Args> void raw(const char *_format, Args &&..._args)
    {
        SEGGER_RTT_printf(0, _format, std::forward<Args>(_args)...);
    }

    template <typename... Args>
    void info(std::source_location _loc, std::string_view _type,
              const char *_format, Args &&..._args)
    {
        log(LogParams{ .loc = _loc,
                       .type = _type,
                       .format = _format,
                       .level = Level::Info },
            std::forward<Args>(_args)...);
    }

    template <typename... Args>
    void debug(std::source_location _loc, std::string_view _type,
               const char *_format, Args &&..._args)
    {
        log(LogParams{ .loc = _loc,
                       .type = _type,
                       .format = _format,
                       .level = Level::Debug },
            std::forward<Args>(_args)...);
    }

    template <typename... Args>
    void warn(std::source_location _loc, std::string_view _type,
              const char *_format, Args &&..._args)
    {
        log(LogParams{ .loc = _loc,
                       .type = _type,
                       .format = _format,
                       .level = Level::Warn },
            std::forward<Args>(_args)...);
    }

    template <typename... Args>
    void error(std::source_location _loc, std::string_view _type,
               const char *_format, Args &&..._args)
    {
        log(LogParams{ .loc = _loc,
                       .type = _type,
                       .format = _format,
                       .level = Level::Error },
            std::forward<Args>(_args)...);
    }


    /**
    * @brief 完美转发检验错误,请用宏STM_ERROR_CHECK
    */
    // template <typename Func>
    // void check(Func &&_operation, const char *_file, int _line,
    //            const char *_expr)
    // {
    //     stm_err_t _err = _operation();
    //     if (unlikely(_err != 0)) {
    //         printf("ERROR: ", RED, "Check failed at %s:%d\nExpr: %s\nError: %d",
    //                _file, _line, _expr, _err);
    //     }
    // }

    /**
    * @brief 清屏
    */
    void clear();

    /**
    * @brief 浮点数转字符串
    */
    void Float2Str(char *str, size_t buffer_size, float va);

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
    template <typename... Args>
    void log(const LogParams &_params, Args &&..._args)
    {
        if (!config.enable)
            return;

        constexpr size_t MAX_LOG_LENGTH = 128;
        char buffer[MAX_LOG_LENGTH];
        char *ptr = buffer;
        const char *end = buffer + MAX_LOG_LENGTH;

        // 写入颜色控制码（如果启用）
        if (config.showColor) [[likely]] {
            auto _color = getLevelColor(_params.level);
            size_t len =
                    std::min(_color.size(), static_cast<size_t>(end - ptr));
            memcpy(ptr, _color.data(), len);
            ptr += len;
        }

        // 写入位置信息（如果启用）
        if (config.showlocation) [[likely]] {
            std::string_view file(_params.loc.file_name());
            if (auto pos = file.find_last_of("/\\");
                pos != std::string_view::npos) {
                file = file.substr(pos + 1);
            }
            size_t len =
                    snprintf(ptr, end - ptr,
                             " [%.*s:%ld]: ", static_cast<int>(file.size()),
                             file.data(), _params.loc.line());
            ptr += std::min(len, static_cast<size_t>(end - ptr));
        }

        // 写入日志类型
        size_t len =
                std::min(_params.type.size(), static_cast<size_t>(end - ptr));
        memcpy(ptr, _params.type.data(), len);
        ptr += len;

        // 写入分隔符
        len = std::min(sizeof(": ") - 1, static_cast<size_t>(end - ptr));
        memcpy(ptr, ": ", len);
        ptr += len;

        if (_params.level == Level::Raw) {
            len = snprintf(ptr, end - ptr, _params.format,
                           std::forward<Args>(_args)...);
            ptr += std::min(len, static_cast<size_t>(end - ptr));
        } else {
            len = snprintf(ptr, end - ptr, _params.format,
                           std::forward<Args>(_args)...);
            ptr += std::min(len, static_cast<size_t>(end - ptr));
        }

        len = std::min(sizeof(RTT_CTRL_RESET "\r\n") - 1,
                       static_cast<size_t>(end - ptr));
        memcpy(ptr, RTT_CTRL_RESET "\r\n", len);
        ptr += len;

        size_t total_len = ptr - buffer;

        SEGGER_RTT_Write(0, buffer, total_len);
    }

protected:
    Logger(const Logger &);
    Logger &operator=(const Logger &);
    Logger() = default;

private:
    Config config;
};


//  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ some preset ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
template <typename... Args> struct info {
    constexpr info(std::string_view _type, const char *_format, Args &&..._args,
                   std::source_location _loc = std::source_location::current())
    {
        Logger::instance().log(LogParams{ .loc = _loc,
                                          .type = _type,
                                          .format = _format,
                                          .level = Level::Info },
                               std::forward<Args>(_args)...);
    }
};
template <typename... Args>
info(std::string_view _type, const char *_format, Args &&...args)
        -> info<Args...>;

template <typename... Args> struct warn {
    constexpr warn(std::string_view _type, const char *_format, Args &&..._args,
                   std::source_location _loc = std::source_location::current())
    {
        Logger::instance().log(LogParams{ .loc = _loc,
                                          .type = _type,
                                          .format = _format,
                                          .level = Level::Warn },
                               std::forward<Args>(_args)...);
    }
};

template <typename... Args>
warn(std::string_view _type, const char *_format, Args &&...args)
        -> warn<Args...>;

template <typename... Args> struct error {
    constexpr error(std::string_view _type, const char *_format,
                    Args &&..._args,
                    std::source_location _loc = std::source_location::current())
    {
        Logger::instance().log(LogParams{ .loc = _loc,
                                          .type = _type,
                                          .format = _format,
                                          .level = Level::Warn },
                               std::forward<Args>(_args)...);
    }
};
template <typename... Args>
error(std::string_view _type, const char *_format, Args &&...args)
        -> error<Args...>;

}
