#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <vector>
#include "StmLog.hpp"

using namespace LOG;

Logger &Logger::instance()
{
    static Logger instance;
    return instance;
}

void Logger::clear() { SEGGER_RTT_WriteString(0, "  " RTT_CTRL_CLEAR); }

void Logger::float2Str(char *_str, size_t _buffer_size, float _va)
{
    // 处理符号
    int flag = _va < 0;
    _va = fabsf(_va);

    // 四舍五入到小数点后3位
    float rounded = roundf(_va * 1000) / 1000;

    int head = (int)rounded;
    int point = (((int)rounded - head) * 1000);

    // 确保point在0-999范围内
    point = abs(point) % 1000;

    if (flag) {
        snprintf(_str, _buffer_size, "-%d.%03d", head, point);
    } else {
        snprintf(_str, _buffer_size, "%d.%03d", head, point);
    }
}

static std::vector<std::string> splitStr(const std::string &_str,
                                         char _delimiter)
{
    std::vector<std::string> tokens;
    std::string token;

    for (char c : _str) {
        if (c == _delimiter) {
            if (!token.empty()) {
                tokens.push_back(token);
                token.clear();
            }
        } else if (std::isspace(static_cast<unsigned char>(c)) || c == '\n' ||
                   c == '\r') {
            continue;
        } else {
            token += c;
        }
    }

    if (!token.empty()) {
        tokens.push_back(token);
    }

    return tokens;
}

static int handleLogSet(const std::vector<std::string> &_args)
{
    Logger &logger = Logger::instance();
    // 参数检查：log set <module> <level> → 至少3个参数（args[0]=set, args[1]=模块名, args[2]=级别）
    if (_args.size() < 3) {
        logger.error(
                LOCATION, "LOG_CMD",
                "log set: missing params! Usage: log set <module> <level>");
        return 0;
    }
    const std::string &module = _args[1];
    const std::string &levelStr = _args[2];
    Level_e targetLevel = strToLogLevel(levelStr);

    if (targetLevel == Level_e::NONE && levelStr != "NONE") {
        logger.error(
                LOCATION, "LOG_CMD",
                "log set: invalid level! Only UPPERCASE supported: NONE/ERROR/WARN/INFO/DEBUG/VERBOSE");
        return 0;
    }

    logger.setTypeLevel(module, targetLevel);
    logger.info(LOCATION, "LOG_CMD", "Success! Module: %s, Level: %s",
                module.c_str(), logLevelToStr(targetLevel));
    return 0;
}

static int handleLogGet(const std::vector<std::string> &_args)
{
    Logger &logger = Logger::instance();
    // 情况1：查询指定模块（args[1]=模块名）
    if (_args.size() >= 2) {
        const std::string &module = _args[1];
        Level_e moduleLevel = logger.getTypeLevel(module);
        logger.info(LOCATION, "LOG_CMD", "Module: %s, Level: %s",
                    module.c_str(), logLevelToStr(moduleLevel));
        return 0;
    } //else need add

    // 情况2：查询所有已配置模块
    auto allModules = logger.getTypeLevels();
    if (allModules.empty()) {
        logger.info(LOCATION, "LOG_CMD",
                    "No custom module levels (Global Level: %s)",
                    logLevelToStr(logger.getGlobalLevel()));
        return 0;
    } else {
        logger.info(LOCATION, "LOG_CMD", "All module levels:");
        for (const auto &[module, level] : allModules) {
            logger.info(LOCATION, "LOG_CMD", "  %s: %s", module.c_str(),
                        logLevelToStr(level));
        }
    }

    return 0;
}

static int handleLogGlobal(const std::vector<std::string> &_args)
{
    Logger &logger = Logger::instance();
    // 参数检查：log global <level> → 至少2个参数（args[0]=global, args[1]=级别）
    if (_args.size() < 2) {
        logger.error(LOCATION, "LOG_CMD",
                     "log global: missing params! Usage: log global <level>");
        return 0;
    }

    const std::string &levelStr = _args[1];
    Level_e targetLevel = strToLogLevel(levelStr);
    if (targetLevel == Level_e::NONE && levelStr != "NONE" &&
        levelStr != "none") {
        logger.error(
                LOCATION, "LOG_CMD",
                "log global: invalid level! Supported: NONE/ERROR/WARN/INFO/DEBUG/VERBOSE");
        return 0;
    }

    logger.setLevel(targetLevel);
    logger.info(LOCATION, "LOG_CMD", "Success! Global Level: %s",
                logLevelToStr(targetLevel));
    return 0;
}

static int handleLogDel(const std::vector<std::string> &_args)
{
    Logger &logger = Logger::instance();
    // 参数检查：log del <module> → 至少2个参数（args[0]=del, args[1]=模块名）
    if (_args.size() < 2) {
        logger.error(LOCATION, "LOG_CMD",
                     "log del: missing params! Usage: log del <module>");
        return 0;
    }

    const std::string &module = _args[1];
    if (!logger.hasTypeLevel(module)) {
        logger.error(
                LOCATION, "LOG_CMD",
                "log del: module '%s' has no custom level (already uses global)",
                module.c_str());
        return 0;
    }

    logger.removeTypeLevel(module);
    logger.info(LOCATION, "LOG_CMD",
                "Success! Module '%s' removed (now uses global level: %s)",
                module.c_str(), logLevelToStr(logger.getGlobalLevel()));
    return 0;
}

namespace LOG {
void handleLogCmd(const std::string &_cmd)
{
    std::vector<std::string> args = splitStr(_cmd, ' ');
    if (args.empty()) {
        Logger::instance().error(LOCATION, "LOG_CMD", "Empty log command!");
        return;
    }

    const std::string &subcmd = args[0];
    if (subcmd == "set")
        handleLogSet(args);
    else if (subcmd == "get")
        handleLogGet(args);
    else if (subcmd == "global")
        handleLogGlobal(args);
    else if (subcmd == "del")
        handleLogDel(args);
    else
        Logger::instance().error(LOCATION, "LOG_CMD",
                                 "Invalid subcmd! Supported: set/get/global");
}

} // namespace LOG