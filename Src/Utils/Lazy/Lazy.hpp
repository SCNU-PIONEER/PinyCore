/**
 *  @file Lazy.hpp
 *  @brief 'std::optional' tiny version
 *         延迟初始化，构造入参放在init函数中
 *         比原来类大4字节（bool标志位）
 *         不支持拷贝构造和赋值
 */

#pragma once

#include <bits/move.h>
#include <bits/stl_construct.h>
#include <type_traits>
#include "StmLog.hpp"

template <typename T> class Lazy {
public:
    constexpr explicit Lazy() : empty() {}

    constexpr ~Lazy()
    {
        if (engaged_ && !std::is_trivially_destructible_v<T>) {
            obj.~T();
        }
    }

    template <typename... Args>
    requires std::is_constructible_v<T, Args...> constexpr T &
    init(Args &&..._args) noexcept(std::is_nothrow_constructible_v<T, Args...>)
    {
        if (!engaged_) {
            construct(std::forward<Args>(_args)...);
            engaged_ = true;
        }
        return obj;
    }

    constexpr explicit operator bool() const noexcept { return engaged_; }

    constexpr T &get() noexcept
    {
        LOG::CHECK(&this->obj);
        return obj;
    }

    constexpr T *operator->()
    {
        LOG::CHECK(&this->obj);
        return std::addressof(obj);
    }

    constexpr T &operator*()
    {
        LOG::CHECK(&this->obj);
        return obj;
    }

    Lazy(const Lazy &) = delete;
    Lazy &operator=(const Lazy &) = delete;

private:
    template <typename... Args> constexpr void construct(Args &&..._args)
    {
        new (std::addressof(obj)) T(std::forward<Args>(_args)...);
    }

    struct Empty_s {};

    union {
        T obj;
        Empty_s empty;
    };

    bool engaged_ = false;
};
