#pragma once

#include <string>
#include <cstdint>
#include <source_location>
#include <unordered_map>

namespace LOG {

/* Definitions for error constants. */
#define STM_OK                   0 /*!< stm_err_t value indicating success (no error) */
#define STM_FAIL                 (-1) /*!< Generic stm_err_t code indicating failure */

#define STM_ERR_NO_MEM           0x101 /*!< Out of memory */
#define STM_ERR_INVALID_ARG      0x102 /*!< Invalid argument */
#define STM_ERR_INVALID_STATE    0x103 /*!< Invalid state */
#define STM_ERR_INVALID_SIZE     0x104 /*!< Invalid size */
#define STM_ERR_NOT_FOUND        0x105 /*!< Requested resource not found */
#define STM_ERR_NOT_SUPPORTED    0x106 /*!< Operation or feature not supported */
#define STM_ERR_TIMEOUT          0x107 /*!< Operation timed out */
#define STM_ERR_INVALID_RSTMONSE 0x108 /*!< Received response was invalid */
#define STM_ERR_INVALID_CRC      0x109 /*!< CRC or checksum was invalid */
#define STM_ERR_INVALID_VERSION  0x10A /*!< Version was invalid */
#define STM_ERR_INVALID_MAC      0x10B /*!< MAC address was invalid */
#define STM_ERR_NOT_FINISHED     0x10C /*!< Operation has not fully completed */
#define STM_ERR_NOT_ALLOWED      0x10D /*!< Operation is not allowed */

#define STM_ERR_FLASH_BASE       0x6000 /*!< Starting number of flash error codes */

#define STM_ERR_HW_CRYPTO_BASE \
    0xc000 /*!< Starting number of HW cryptography module error codes */
#define STM_ERR_MEMPROT_BASE \
    0xd000 /*!< Starting number of Memory Protection API error codes */

typedef int stm_err_t;

constexpr auto RED = "\x1B[1;31m";
constexpr auto GREEN = "\x1B[1;32m";
constexpr auto YELLOW = "\x1B[1;33m";
constexpr auto BLUE = "\x1B[1;35m";
constexpr auto MAGENTA = "\x1B[1;35m";
constexpr auto CYAN = "\x1B[1;36m";
constexpr auto WHITE = "\x1B[1;37m";

static constexpr size_t COLOR_RESET = 0;   // reset
static constexpr size_t COLOR_TRACE = 7;   // cyan
static constexpr size_t COLOR_DEBUG = 6;   // magenta
static constexpr size_t COLOR_INFO = 3;    // green
static constexpr size_t COLOR_WARNING = 4; // yellow
static constexpr size_t COLOR_ERROR = 2;   // red
static constexpr size_t COLOR_FATAL = 9;   // bright red
static constexpr size_t COLOR_STEP = 5;    // blue
//
enum class Level_e : uint8_t {
    NONE,      // 完全禁用日志 (0)
    ERROR,     // 关键错误，系统无法自行恢复 (1)
    WARN,      // 警告条件，已采取恢复措施 (2)
    INFO,      // 正常运行信息 (3)
    DEBUGGING, // 调试信息（变量、指针等）(4)
    VERBOSE,   // 高频/详细调试信息 (5)
    RAW        // 原始输出（不受级别控制）(6)
};

enum class Proto_e : uint8_t {
    RTT,
    VOFA_FIREWATER,
    VOFA_JUSTFLOAT,
    // LOG_UART, // INFO: waiting for design
    // LOG_USB,  // INFO: waiting for design
};

class ModuleControl {
public:
    ModuleControl() = default;
    ~ModuleControl() = default;

    void setModuleLevel(std::string_view _module, Level_e _level)
    {
        module_levels_[std::string(_module)] = _level;
    }

    Level_e getModuleLevel(std::string_view _module) const
    {
        auto it = module_levels_.find(std::string(_module));
        return (it != module_levels_.end()) ? it->second : global_level_;
    }

    // 设置全局级别
    void setGlobalLevel(Level_e _level) { global_level_ = _level; }

private:
    std::unordered_map<std::string, Level_e> module_levels_; // 模块级别映射表
    Level_e global_level_ = Level_e::INFO;                   // 全局默认日志级别
};

class Config {
public:
    std::string name{ "" };         // log directory
    Level_e level{ Level_e::INFO }; // Log level
    Proto_e proto{ Proto_e::RTT };  // Log protocol
    bool showColor{ true };         // show full file paths in logs
    bool showlocation{ true };      // show full file paths in logs
    bool enable{ true };            // Enable or disable logging
    uint8_t bufNum{ 0 };            // Buffer number
    ModuleControl moduleControl;    // 模块级别控制器
    bool vofaEnabled{ false };      // 是否启用VOFA协议
};

class LogParams {
public:
    std::source_location loc = std::source_location::current();
    std::string_view type;
    std::string_view module; // 新增模块标签
    const char *format;
    Level_e level{ Level_e::INFO };
};

constexpr std::string_view getLevelColor(Level_e _level)
{
    switch (_level) {
    case Level_e::RAW:
        return "";
    case Level_e::INFO:
        return GREEN;
    case Level_e::WARN:
        return YELLOW;
    case Level_e::ERROR:
        return RED;
    case Level_e::DEBUGGING:
        return BLUE;
    case Level_e::VERBOSE:
        return WHITE;
    default:
        return "";
    }
}


#if !defined(unlikely)
#if defined(__GNUC__) || defined(__clang__)
#define unlikely(x) __builtin_expect(!!(x), 0)
#else
#define unlikely(x) (x)
#endif

#endif

} // namespace LOG
