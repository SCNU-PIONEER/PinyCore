#pragma once

/**
 * @example: 
 *
 * class MySingleton : public Singleton<MySingleton> {
 * private:
 *    MySingleton() = default;
 *    friend class Singleton<MySingleton>;
 * };
 *
 * @debug add watch:
 * Singleton<MySingleton>::instance()::instance
 *
 */

template <typename Derived> class Singleton {
public:
    static Derived &instance()
    {
        static Derived instance;
        return instance;
    }

    Singleton(const Singleton &) = delete;
    Singleton &operator=(const Singleton &) = delete;

    Singleton(Singleton &&) = delete;
    Singleton &operator=(Singleton &&) = delete;

protected:
    Singleton() = default;
};
