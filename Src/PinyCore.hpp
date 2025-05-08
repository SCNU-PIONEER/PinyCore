#pragma once

#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

void initPinyCore();

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include <utility>
class PinyCore {
    using LoggerCallback = void (*)(const char *, const char *, const char *,
                                    ...);
    inline static LoggerCallback activeLogger_;
public:
    inline static PinyCore* instance() {
        static PinyCore instance_;
        return &instance_;
    }

    PinyCore(const PinyCore &) = delete;
    PinyCore &operator=(const PinyCore &) = delete;

    inline void registerLogger(LoggerCallback _logger) {
        activeLogger_ = std::move(_logger);
    }

    template<typename... Args>
    inline void log(const char* _type, const char* _color, const char* _format, Args... _args) {
        if (activeLogger_) {
            activeLogger_(_type, _color, _format, _args...);
        }
    }

    void bspInit(); // TODO:
    void osInit(); // TODO:
    void coreInit(); // TODO:

    void init();

private:
    PinyCore() = default;
};

#endif
