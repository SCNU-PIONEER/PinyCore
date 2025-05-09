#pragma once

#include <string>
#include <source_location>

/* Definitions for error constants. */
#define STM_OK                   0 /*!< stm_err_t value indicating success (no error) */
#define STM_FAIL                 -1 /*!< Generic stm_err_t code indicating failure */

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
enum class Level {
    Raw,   // Raw log output
    Info,  // Information messages which describe normal flow of events
    Warn,  // Error conditions from which recovery measures have been taken
    Error, // Critical errors, software module can not recover on its own
    Debug, // Extra information which is not necessary for normal use (values, pointers, sizes, etc)
    Verbose // Bigger chunks of debugging information, or frequent messages which can potentially flood the output
};

enum class Proto {
    RTT,
    VOFA_FireWater,
    VOFA_JustFloat,
    // LOG_UART, // INFO: waiting for design
    // LOG_USB,  // INFO: waiting for design
};

class Config {
public:
    std::string name{ "" };     // log directory
    Level level{ Level::Info }; // Log level
    Proto proto{ Proto::RTT };  // Log protocol
    bool showColor{ true };     // show full file paths in logs
    bool showlocation{ true };  // show full file paths in logs
    bool enable{ true };        // Enable or disable logging
};

struct LogParams {
    std::source_location loc = std::source_location::current();
    std::string_view type;
    const char *format;
};

constexpr std::string_view get_level_color(Level level)
{
    switch (level) {
    case Level::Info:
        return GREEN;
    case Level::Warn:
        return YELLOW;
    case Level::Error:
        return RED;
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
