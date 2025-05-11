#pragma once

// #ifdef USE_LOG
#include "SEGGER_RTT.h"
#include "stm_log_msg.hpp"
#include <string_view>

namespace LOG {


class Logger {
public:
    inline static Logger &instance()
    {
        static Logger instance;
        return instance;
    }

    template <typename... Args> void printf(const char *format, Args &&..._args)
    {
        SEGGER_RTT_printf(0, format, std::forward<Args>(_args)...);
        SEGGER_RTT_WriteString(0, RTT_CTRL_RESET "\r\n");
    }

    template <typename... Args>
    void info(std::string_view _type, const char *_format, Args &&..._args)
    {
        config.level = Level::Info;
        log(LogParams{ .type = _type, .format = _format },
            std::forward<Args>(_args)...);
    }

    template <typename... Args>
    void warn(std::string_view _type, const char *_format, Args &&..._args)
    {
        config.level = Level::Warn;
        log(LogParams{ .type = _type, .format = _format },
            std::forward<Args>(_args)...);
    }

    template <typename... Args>
    void error(std::string_view _type, const char *_format, Args &&..._args)
    {
        config.level = Level::Error;
        log(LogParams{ .type = _type, .format = _format },
            std::forward<Args>(_args)...);
    }


    /**
    * @brief 完美转发检验错误,请用宏STM_ERROR_CHECK
    */
    // template <typename Func>
    // void check(Func &&_operation, const char *_file, int _line,
    //            const char *_expr)

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

    // void setName(std::string_view _name) { config.name = _name; }

    void setProto(Proto _proto) { config.proto = _proto; }

    void setConfig(const Config &_config) { config = _config; }


protected:
    Logger(const Logger &);
    Logger &operator=(const Logger &);
    Logger() = default;

private:
    /**
    * @brief 完美转发打印函数,自带换行
    */
    template <typename... Args>
    void log(const LogParams &_params, Args &&..._args)
    {
        if (!config.enable)
            return;

        if (config.level == Level::Raw) {
            SEGGER_RTT_printf(0, _params.format, std::forward<Args>(_args)...);
            return;
        }

        /* 输出日志头（源码位置+颜色+类型) */
        if (config.showColor) [[likely]] {
            auto _color = get_level_color(config.level);
            SEGGER_RTT_Write(0, _color.data(), _color.size());
        }

        if (config.showlocation) [[likely]] {
            std::string_view file(_params.loc.file_name());
            if (auto pos = file.find_last_of("/\\");
                pos != std::string_view::npos) {
                file = file.substr(pos + 1);
            }
            SEGGER_RTT_printf(0, " [%s:%d]: ", file.data(), _params.loc.line());
        }

        SEGGER_RTT_Write(0, _params.type.data(), _params.type.size());
        SEGGER_RTT_WriteString(0, ": ");

        /* 格式化用户内容（通过va_list转发 */
        SEGGER_RTT_printf(0, _params.format, std::forward<Args>(_args)...);

        /* 重置样式 */
        SEGGER_RTT_WriteString(0, RTT_CTRL_RESET "\r\n");
    }

    Config config;
};

}
