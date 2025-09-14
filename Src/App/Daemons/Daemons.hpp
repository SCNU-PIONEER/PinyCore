#pragma once

#include <list>
#include <cstdint>
#include <functional>

class Daemons {
public:
    static Daemons &instance();

    void schedule(std::function<void()> _func);

    /* 200hz */
    void update();

private:
    Daemons();

    std::list<std::function<void()> > cb;

    static constexpr uint8_t SEND_INTERVAL = 5;
};
