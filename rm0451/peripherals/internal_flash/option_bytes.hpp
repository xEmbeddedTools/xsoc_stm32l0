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
class option_bytes : private xmcu::non_constructible
{
public:
    class unlocker : private xmcu::non_constructible
    {
    public:
        static void unlock();
        static bool unlock(xmcu::Milliseconds a_timeout);

        static void lock();
    };

    struct secure_flash : private xmcu::non_constructible
    {
        static std::uint32_t get_start_address();
        static std::uint32_t get_start_address(xmcu::Milliseconds a_timeout);
    };

    struct BOR : private xmcu::non_constructible
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

        using enum Level;

        static bool set(Level level_a);
        static bool set(Level level_a, xmcu::Milliseconds timeout_a);

        static Level get();
    };

    struct RDP : private xmcu::non_constructible
    {
        enum class Level : std::uint32_t
        {
            _0 = 0xAAu,
            _1 = 0xBBu,
            _2 = 0xCCu
        };

        using enum Level;

        static bool set(Level level_a);
        static bool set(Level level_a, xmcu::Milliseconds timeout_a);

        static Level get();
    };

    struct USR : private xmcu::non_constructible
    {
        enum class Flags : std::uint32_t
        {
            boot_1 = FLASH_OPTR_BOOT1 >> 16u,
            rst_stby = FLASH_OPTR_nRST_STDBY >> 16u,
            rst_stop = FLASH_OPTR_nRST_STOP >> 16u,
            iwdg_sw = FLASH_OPTR_IWDG_SW >> 16u
        };

        using enum Flags;

        static bool set(Flags flags_a);
        static bool set(Flags flags_a, xmcu::Milliseconds timeout_a);

        static Flags get();
    };

    static bool launch();
    static bool launch(xmcu::Milliseconds a_timeout);
};

constexpr option_bytes::USR::Flags operator|(option_bytes::USR::Flags left_a, option_bytes::USR::Flags right_a)
{
    return static_cast<option_bytes::USR::Flags>(static_cast<std::uint32_t>(left_a) |
                                                 static_cast<std::uint32_t>(right_a));
}
constexpr option_bytes::USR::Flags operator&(option_bytes::USR::Flags left_a, option_bytes::USR::Flags right_a)
{
    return static_cast<option_bytes::USR::Flags>(static_cast<std::uint32_t>(left_a) &
                                                 static_cast<std::uint32_t>(right_a));
}
constexpr option_bytes::USR::Flags operator~(option_bytes::USR::Flags left_a)
{
    return static_cast<option_bytes::USR::Flags>(~static_cast<std::uint32_t>(left_a));
}
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