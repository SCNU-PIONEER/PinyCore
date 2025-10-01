#pragma once

template <typename Derived> class BspBase {
public:
    static Derived &instance()
    {
        static Derived instance;
        return instance;
    }

protected:
    BspBase(const BspBase &);
    BspBase &operator=(const BspBase &);
    BspBase() = default;
};
