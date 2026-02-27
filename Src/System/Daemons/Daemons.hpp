#pragma once

#include <cstdint>
#include <vector>
#include <functional>

class Daemons {
public:
    static Daemons &instance()
    {
        static Daemons instance;
        return instance;
    }

    void init();

    void schedule(std::function<void()> _func);

    /* 200hz */
    void update();

private:
    Daemons(const Daemons &);
    Daemons &operator=(const Daemons &);
    Daemons() = default;

    std::vector<std::function<void()> > cb;
    static constexpr uint8_t SEND_INTERVAL = 5;
};
