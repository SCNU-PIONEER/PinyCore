#pragma once

#include "SEGGER_RTT.h"
#include "StmLogMsg.hpp"
#include <string_view>
#include <cstring>
#include "FreeRTOS.h"
#include "semphr.h"
#include "Vofa.hpp"

#define LOCATION std::source_location::current()

namespace LOG {

class Logger {
public:
    static Logger &instance()
    {
        static Logger instance;
        return instance;
    }


    Logger() : log_mutex_(xSemaphoreCreateMutex())
    {
        configASSERT(log_mutex_ != NULL);
    }

    ~Logger()
    {
        if (log_mutex_ != NULL) {
            vSemaphoreDelete(log_mutex_);
        }
    }

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
        stm_err_t err = _operation();
        if (unlikely(err != 0)) {
            error(_loc, "check", "error code: %d", err);
            while (1) {
            }
        }
    }
    //add vofa
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

        // 模块级别过滤（先检查级别避免不必要的锁）
        if (config.moduleControl.getModuleLevel(_params.type) ==
            Level_e::NONE) {
            return; // 直接返回，不输出任何日志
        }
        if (_params.level > config.moduleControl.getModuleLevel(_params.type)) {
            return;
        }

        // 获取日志输出锁
        if (xSemaphoreTake(log_mutex_, portMAX_DELAY) == pdTRUE) {
            constexpr size_t MAX_LOG_LENGTH = 128;
            char buffer[MAX_LOG_LENGTH];
            char *ptr = buffer;
            const char *end = buffer + MAX_LOG_LENGTH;

            // 写入颜色控制码（如果启用）
            if (config.showColor) [[likely]] {
                auto color = getLevelColor(_params.level);
                size_t len =
                        std::min(color.size(), static_cast<size_t>(end - ptr));
                memcpy(ptr, color.data(), len);
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
            size_t len = std::min(_params.type.size(),
                                  static_cast<size_t>(end - ptr));
            memcpy(ptr, _params.type.data(), len);
            ptr += len;

            // 写入分隔符
            len = std::min(sizeof(": ") - 1, static_cast<size_t>(end - ptr));
            memcpy(ptr, ": ", len);
            ptr += len;

            if (_params.level == Level_e::RAW) {
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

            size_t totalLen = ptr - buffer;

            SEGGER_RTT_Write(0, buffer, totalLen);
            xSemaphoreGive(log_mutex_);
        }
    }

protected:
    Logger(const Logger &);
    Logger &operator=(const Logger &);
    // Logger() = default;

private:
    Config config;
    SemaphoreHandle_t log_mutex_; // 日志输出互斥量
    Vofa &vofa_ = Vofa::instance();
};


//  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ some preset ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// NOLINTBEGIN
template <typename... Args> struct info {
    constexpr info(std::string_view _type, const char *_format, Args &&..._args,
                   std::source_location _loc = std::source_location::current())
    {
        auto &logger = Logger::instance();
        // 检查模块级别是否允许输出
        if (logger.getModuleLevel(std::string(_type)) <= Level_e::INFO) {
            Logger::instance().log(LogParams{ .loc = _loc,
                                              .type = _type,
                                              .format = _format,
                                              .level = Level_e::INFO },
                                   std::forward<Args>(_args)...);
        }
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
