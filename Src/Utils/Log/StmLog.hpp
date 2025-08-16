#pragma once

#include "SEGGER_RTT.h"
#include "StmLogMsg.hpp"
#include <string_view>
#include <cstring>
#include "Vofa.hpp"

#define LOCATION std::source_location::current()

namespace LOG {

class Logger {
public:
    static Logger &instance();

    Logger() = default;
    ~Logger() = default;

    void setModuleLevel(const std::string &_module, Level_e _level)
    {
        config.moduleControl.setModuleLevel(_module, _level);
    }

    Level_e getModuleLevel(const std::string &_module) const
    {
        return config.moduleControl.getModuleLevel(_module);
    }

    template <typename... Args> void raw(const char *_format, Args &&..._args)
    {
        SEGGER_RTT_printf(0, _format, std::forward<Args>(_args)...);
    }

    // 日志接口：新增_module参数（模块标签），保留_type参数
    template <typename... Args>
    void info(std::source_location _loc, std::string_view _module,
              std::string_view _type, const char *_format, Args &&..._args)
    {
        log(LogParams{ .loc = _loc,
                       .type = _type,
                       .module = _module, // 传入模块标签
                       .format = _format,
                       .level = Level_e::INFO },
            std::forward<Args>(_args)...);
    }

    template <typename... Args>
    void debug(std::source_location _loc, std::string_view _module,
               std::string_view _type, const char *_format, Args &&..._args)
    {
        log(LogParams{ .loc = _loc,
                       .type = _type,
                       .module = _module,
                       .format = _format,
                       .level = Level_e::DEBUGGING },
            std::forward<Args>(_args)...);
    }

    template <typename... Args>
    void warn(std::source_location _loc, std::string_view _module,
              std::string_view _type, const char *_format, Args &&..._args)
    {
        log(LogParams{ .loc = _loc,
                       .type = _type,
                       .module = _module,
                       .format = _format,
                       .level = Level_e::WARN },
            std::forward<Args>(_args)...);
    }

    template <typename... Args>
    void error(std::source_location _loc, std::string_view _module,
               std::string_view _type, const char *_format, Args &&..._args)
    {
        log(LogParams{ .loc = _loc,
                       .type = _type,
                       .module = _module,
                       .format = _format,
                       .level = Level_e::ERROR },
            std::forward<Args>(_args)...);
    }

    template <typename Func>
    void check(std::source_location _loc, Func &&_operation)
    {
        stm_err_t err = std::forward<Func>(_operation)();
        if (unlikely(err != 0)) {
            // 模块标签设为"CHECK"，保留原有type逻辑
            error(_loc, "CHECK", "error", "error code: %d", err);
            while (1) {
            }
        }
    }

    void enableVofa(UART_HandleTypeDef *_huart, DMA_HandleTypeDef *_hdma)
    {
        vofa_.init(_huart, _hdma);
        config.vofaEnabled = true;
    }

    void setVofaProtocol(Proto_e _proto)
    {
        if (_proto == Proto_e::VOFA_FIREWATER ||
            _proto == Proto_e::VOFA_JUSTFLOAT) {
            config.proto = _proto;
        }
    }

    void clear();

    void float2Str(char *_str, size_t _buffer_size, float _va);

    void disable() { config.enable = false; }

    void enable() { config.enable = true; }

    void setLevel(Level_e _level) { config.level = _level; }

    void setColor(bool _enable) { config.showColor = _enable; }

    void setLocation(bool _enable) { config.showlocation = _enable; }

    void setName(std::string_view _name) { config.name = _name; }

    void setProto(Proto_e _proto) { config.proto = _proto; }

    void setConfig(const Config &_config) { config = _config; }

    template <typename... Args>
    void log(const LogParams &_params, Args &&..._args)
    {
        if (!config.enable)
            return;

        // 基于模块标签（_params.module）进行级别过滤
        Level_e moduleLevel =
                config.moduleControl.getModuleLevel(_params.module);
        if (moduleLevel == Level_e::NONE) {
            return;
        }
        if (_params.level > moduleLevel) {
            return;
        }

        constexpr size_t MAX_LOG_LENGTH = 128;
        char buffer[MAX_LOG_LENGTH];
        char *ptr = buffer;
        const char *end = buffer + MAX_LOG_LENGTH;

        if (config.showColor) [[likely]] {
            auto color = getLevelColor(_params.level);
            size_t len = std::min(color.size(), static_cast<size_t>(end - ptr));
            memcpy(ptr, color.data(), len);
            ptr += len;
        }

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

        // 输出模块标签（新增）
        size_t len = snprintf(ptr, end - ptr, "[%.*s] ",
                              static_cast<int>(_params.module.size()),
                              _params.module.data());
        ptr += std::min(len, static_cast<size_t>(end - ptr));

        // 保留原有type输出
        len = std::min(_params.type.size(), static_cast<size_t>(end - ptr));
        memcpy(ptr, _params.type.data(), len);
        ptr += len;

        len = std::min(sizeof(": ") - 1, static_cast<size_t>(end - ptr));
        memcpy(ptr, ": ", len);
        ptr += len;

        len = snprintf(ptr, end - ptr, _params.format,
                       std::forward<Args>(_args)...);
        ptr += std::min(len, static_cast<size_t>(end - ptr));


        len = std::min(sizeof(RTT_CTRL_RESET "\r\n") - 1,
                       static_cast<size_t>(end - ptr));
        memcpy(ptr, RTT_CTRL_RESET "\r\n", len);
        ptr += len;

        size_t totalLen = ptr - buffer;

        SEGGER_RTT_Write(0, buffer, totalLen);
    }

protected:
    Logger(const Logger &);
    Logger &operator=(const Logger &);

private:
    Config config;
    Vofa &vofa_ = Vofa::instance();
};


// 预设日志构造器（新增_module参数）
template <typename... Args> struct Info_s {
    constexpr
    Info_s(std::string_view _module, std::string_view _type,
           const char *_format, Args &&..._args,
           std::source_location _loc = std::source_location::current())
    {
        auto &logger = Logger::instance();
        if (logger.getModuleLevel(std::string(_module)) <= Level_e::INFO) {
            Logger::instance().log(LogParams{ .loc = _loc,
                                              .type = _type,
                                              .module = _module, // 传入模块标签
                                              .format = _format,
                                              .level = Level_e::INFO },
                                   std::forward<Args>(_args)...);
        }
    }
};
template <typename... Args>
Info_s(std::string_view _module, std::string_view _type, const char *_format,
       Args &&..._args) -> Info_s<Args...>;

template <typename... Args> struct Warn_s {
    constexpr
    Warn_s(std::string_view _module, std::string_view _type,
           const char *_format, Args &&..._args,
           std::source_location _loc = std::source_location::current())
    {
        Logger::instance().log(LogParams{ .loc = _loc,
                                          .type = _type,
                                          .module = _module,
                                          .format = _format,
                                          .level = Level_e::WARN },
                               std::forward<Args>(_args)...);
    }
};
template <typename... Args>
Warn_s(std::string_view _module, std::string_view _type, const char *_format,
       Args &&..._args) -> Warn_s<Args...>;

template <typename... Args> struct Error_s {
    constexpr
    Error_s(std::string_view _module, std::string_view _type,
            const char *_format, Args &&..._args,
            std::source_location _loc = std::source_location::current())
    {
        Logger::instance().log(LogParams{ .loc = _loc,
                                          .type = _type,
                                          .module = _module,
                                          .format = _format,
                                          .level = Level_e::ERROR },
                               std::forward<Args>(_args)...);
    }
};
template <typename... Args>
Error_s(std::string_view _module, std::string_view _type, const char *_format,
        Args &&..._args) -> Error_s<Args...>;


template <typename T> void CHECK(T &&_condition)
{
    if constexpr (std::is_invocable_v<T>) {
        Logger::instance().check(LOCATION, std::forward<T>(_condition));
    } else {
        Logger::instance().check(LOCATION,
                                 [&] { return static_cast<bool>(_condition); });
    }
}

} // namespace LOG
