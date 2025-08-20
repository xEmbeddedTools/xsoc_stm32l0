#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// CMSIS
#include <stm32l0xx.h>

// xmcu
#include <soc/Scoped_guard.hpp>
#include <xmcu/Duration.hpp>
#include <xmcu/Non_copyable.hpp>
#include <xmcu/bit.hpp>
#include <xmcu/non_constructible.hpp>

namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals {
class option_bytes : private non_constructible
{
public:
    class unlocker : private non_constructible
    {
    public:
        static void unlock();
        static bool unlock(Milliseconds a_timeout);

        static void lock();
    };

    struct secure_flash : private non_constructible
    {
        static std::uint32_t get_start_address();
        static std::uint32_t get_start_address(Milliseconds a_timeout);
    };

    struct BOR : private non_constructible
    {
        enum class Level : std::uint8_t
        {
            disabled = 0,
            _1 = 0x8u,
            _2 = 0x9u,
            _3 = 0xAu,
            _4 = 0xBu,
            _5 = 0xCu
        };

        static bool set(Level a_level);
        static Level get_level();
    };
    struct USER : private non_constructible
    {
        static std::uint32_t get();
    };

    static void reload();
};

} // namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals

namespace xmcu::soc {
template<> class Scoped_guard<st::arm::m0::l0::rm0451::peripherals::option_bytes::unlocker> : private Non_copyable
{
public:
    Scoped_guard()
        : unlocked(false)
    {
        st::arm::m0::l0::rm0451::peripherals::option_bytes::unlocker::unlock();
        this->unlocked = (false == bit::flag::is(FLASH->PECR, FLASH_PECR_OPTLOCK));
    }

    Scoped_guard(Milliseconds a_timeout)
        : unlocked(st::arm::m0::l0::rm0451::peripherals::option_bytes::unlocker::unlock(a_timeout))
    {
    }

    ~Scoped_guard()
    {
        st::arm::m0::l0::rm0451::peripherals::option_bytes::unlocker::lock();
    }

    bool is_unlocked() const
    {
        return this->unlocked;
    }

private:
    bool unlocked;
};
} // namespace xmcu::soc