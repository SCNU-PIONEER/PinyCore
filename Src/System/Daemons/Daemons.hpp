#pragma once

#include <cstdint>
#include <vector>
#include <functional>
#include "Singleton.hpp"

class Daemons : public Singleton<Daemons> {
public:
    void init();

    void schedule(std::function<void()> _func);

    /* 200hz */
    void update();

private:
    Daemons() = default;
    friend class Singleton<Daemons>;

    std::vector<std::function<void()> > cb;
    static constexpr uint8_t SEND_INTERVAL = 5;
};
