#pragma once

#include <cstdint>
#include <vector>
#include <functional>

class Daemons {
public:
    Daemons();
    void schedule(std::function<void()> _func);

    /* 200hz */
    void update();

private:
    std::vector<std::function<void()> > cb;
    static constexpr uint8_t SEND_INTERVAL = 5;
};
