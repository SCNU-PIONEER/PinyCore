#pragma once

#include "SEGGER_RTT.h"
#include "StmLogMsg.hpp"
#include <string_view>
#include <cstring>
#include <vector>
#include "Vofa.hpp"

#define LOCATION std::source_location::current()

namespace LOG {

class Logger {
public:
    static Logger &instance();


    void setTypeLevel(const std::string &_type, Level_e _level)
    {
        config.typeControl.setTypeLevel(_type, _level);
    }

    Level_e getTypeLevel(std::string_view _type) const
    {
        auto it = config.typeControl.type_levels_.find(std::string(_type));
        return (it != config.typeControl.type_levels_.end()) ? it->second :
                                                               config.level;
    }

    std::unordered_map<std::string, Level_e> getTypeLevels() const
    {
        return config.typeControl.type_levels_;
    }

    Level_e getGlobalLevel() const { return config.level; }

    void removeTypeLevel(const std::string &_type)
    {
        config.typeControl.removeTypeLevel(_type);
    }

    bool hasTypeLevel(const std::string &_type) const
    {
        return config.typeControl.hasTypeLevel(_type);
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
                       .level = Level_e::INFO },
            std::forward<Args>(_args)...);
    }

    template <typename... Args>
    void debug(std::source_location _loc, std::string_view _type,
               const char *_format, Args &&..._args)
    {
        log(LogParams{ .loc = _loc,
                       .type = _type,
                       .format = _format,
                       .level = Level_e::DEBUGGING },
            std::forward<Args>(_args)...);
    }

    template <typename... Args>
    void warn(std::source_location _loc, std::string_view _type,
              const char *_format, Args &&..._args)
    {
        log(LogParams{ .loc = _loc,
                       .type = _type,
                       .format = _format,
                       .level = Level_e::WARN },
            std::forward<Args>(_args)...);
    }

    template <typename... Args>
    void error(std::source_location _loc, std::string_view _type,
               const char *_format, Args &&..._args)
    {
        log(LogParams{ .loc = _loc,
                       .type = _type,
                       .format = _format,
                       .level = Level_e::ERROR },
            std::forward<Args>(_args)...);
    }

    /**
    * @brief 完美转发检验错误
    */
    template <typename Func>
    void check(std::source_location _loc, Func &&_operation)
    {
        stm_err_t err = std::forward<Func>(_operation)();
        if (unlikely(err != 0)) {
            error(_loc, "check", "error code: %d", err);
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

    /**
    * @brief 清屏
    */
    void clear();

    /**
    * @brief 浮点数转字符串
    */
    void float2Str(char *_str, size_t _buffer_size, float _va);

    void handleLogCmd(const std::string_view &_cmd);

    void disable() { config.enable = false; }

    void enable() { config.enable = true; }

    void setLevel(Level_e _level) { config.level = _level; }

    void setColor(bool _enable) { config.showColor = _enable; }

    void setLocation(bool _enable) { config.showlocation = _enable; }

    void setName(std::string_view _name) { config.name = _name; }

    void setProto(Proto_e _proto) { config.proto = _proto; }

    void setConfig(const Config &_config) { config = _config; }

    /**
    * @brief 完美转发打印函数,自带换行
    */
    template <typename... Args>
    void log(const LogParams &_params, Args &&..._args)
    {
        if (!config.enable)
            return;

        // 获取模块级别和全局级别
        Level_e typeLevel = getTypeLevel(_params.type);
        Level_e effectiveLevel = std::min(typeLevel, config.level);

        // 应用更严格的级别过滤
        if (effectiveLevel == Level_e::NONE) {
            return;
        }
        if (_params.level > effectiveLevel) {
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

        size_t len = snprintf(ptr, end - ptr, "[%.*s] ",
                              static_cast<int>(_params.type.size()),
                              _params.type.data());
        ptr += std::min(len, static_cast<size_t>(end - ptr));

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
    Logger() { SEGGER_RTT_Init(); }

private:
    Config config;
    Vofa &vofa_ = Vofa::instance();

    int handleLogSet(const std::vector<std::string> &_args);
    int handleLogGet(const std::vector<std::string> &_args);
    int handleLogGlobal(const std::vector<std::string> &_args);
    int handleLogDel(const std::vector<std::string> &_args);
    std::vector<std::string> splitStr(const std::string_view &_str,
                                      char _delimiter);
};


//  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ some preset ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// NOLINTBEGIN
template <typename... Args> struct info {
    constexpr info(std::string_view _type, const char *_format, Args &&..._args,
                   std::source_location _loc = std::source_location::current())
    {
        Logger::instance().log(LogParams{ .loc = _loc,
                                          .type = _type,
                                          .format = _format,
                                          .level = Level_e::INFO },
                               std::forward<Args>(_args)...);
    }
};
template <typename... Args>
info(std::string_view _type, const char *_format, Args &&..._args)
        -> info<Args...>;

template <typename... Args> struct warn {
    constexpr warn(std::string_view _type, const char *_format, Args &&..._args,
                   std::source_location _loc = std::source_location::current())
    {
        Logger::instance().log(LogParams{ .loc = _loc,
                                          .type = _type,
                                          .format = _format,
                                          .level = Level_e::WARN },
                               std::forward<Args>(_args)...);
    }
};
template <typename... Args>
warn(std::string_view _type, const char *_format, Args &&..._args)
        -> warn<Args...>;

template <typename... Args> struct error {
    constexpr error(std::string_view _type, const char *_format,
                    Args &&..._args,
                    std::source_location _loc = std::source_location::current())
    {
        Logger::instance().log(LogParams{ .loc = _loc,
                                          .type = _type,
                                          .format = _format,
                                          .level = Level_e::ERROR },
                               std::forward<Args>(_args)...);
    }
};
template <typename... Args>
error(std::string_view _type, const char *_format, Args &&..._args)
        -> error<Args...>;


template <typename T> void CHECK(T &&_condition)
{
    if constexpr (std::is_invocable_v<T>) {
        /* 处理可调用对象 */
        Logger::instance().check(LOCATION, std::forward<T>(_condition));
    } else {
        /* 处理原始值 */
        Logger::instance().check(LOCATION,
                                 [&] { return static_cast<bool>(_condition); });
    }
}
// NOLINTEND

} // namespace LOG
