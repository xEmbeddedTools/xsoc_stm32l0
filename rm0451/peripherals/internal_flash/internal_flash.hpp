#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// externals
#include <stm32l0xx.h>

// xmcu
#include <rm0451/defs.hpp>
#include <rm0451/peripherals/internal_flash/internal_flash_ll.hpp>
#include <soc/Scoped_guard.hpp>
#include <soc/st/arm/m0/nvic.hpp>
#include <xmcu/Duration.hpp>
#include <xmcu/Limited.hpp>
#include <xmcu/Not_null.hpp>
#include <xmcu/bit.hpp>
#include <xmcu/non_constructible.hpp>
#include <xmcu/various.hpp>

namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals {
class internal_flash : private non_constructible
{
public:
    using Word = std::uint64_t;

    struct s : private non_constructible
    {
        static constexpr std::size_t start = FLASH_BASE;
        static constexpr std::size_t page_size_in_bytes = 128u;

#if STM32L010XX_CATEGORY == 1
        static constexpr std::size_t pages_count = 128u;
#elif STM32L010XX_CATEGORY == 2
        static constexpr std::size_t pages_count = 256u;
#elif STM32L010XX_CATEGORY == 3
        static_assert(false,
                      "TODO: For 32 Kbyte category 3 devices, the Flash program memory is divided into 256 pages of "
                      "128 bytes each. Which ones?");
        static constexpr std::size_t pages_count = 512u;
#elif STM32L010XX_CATEGORY == 5
        static constexpr std::size_t pages_count = 1024u;
#else
        static_assert(false, "Unkown MCU model");
#endif
        static constexpr std::size_t size_in_bytes = page_size_in_bytes * pages_count;
    };

    enum class Latency : std::uint32_t
    {
        _0u,
        _1 = static_cast<std::uint32_t>(ll::internal_flash::ACR::latency),
    };

    enum class Cache_mode : std::uint32_t
    {
        disabled = 0x0u,
        prefetech = static_cast<std::uint32_t>(ll::internal_flash::ACR::prften)
    };

    enum class Status_flag : std::uint32_t
    {
        ok = 0x0u,
        read_protection_error = static_cast<std::uint32_t>(ll::internal_flash::SR::rderr),
        write_protection_error = static_cast<std::uint32_t>(ll::internal_flash::SR::wrperr),
        size_error = static_cast<std::uint32_t>(ll::internal_flash::SR::sizerr),
        programming_aligment_error = static_cast<std::uint32_t>(ll::internal_flash::SR::pgaerr),
        fetch_wr_abort_error = static_cast<std::uint32_t>(ll::internal_flash::SR::fwwer),
        overwrite_not_zero_error = static_cast<std::uint32_t>(ll::internal_flash::SR::notzeroerr),
        option_bytes_load_error = static_cast<std::uint32_t>(ll::internal_flash::SR::optverr),
        locked = 0x80000000
    };

    class unlocker : private non_constructible
    {
    public:
        static void unlock();
        static bool unlock(Milliseconds a_timeout);

        static void lock();
    };
    class cache_disabler : private non_constructible
    {
    public:
        static void disable();
        static bool disable(Milliseconds a_timeout);

        static void enable();
        static bool enable(Milliseconds a_timeout);

    private:
        static inline Cache_mode cache_mode = various::get_enum_incorrect_value<Cache_mode>();
    };

    class polling : private non_constructible
    {
    public:
        enum class Bank_id : std::uint32_t
        {
            _0
        };

        struct Result
        {
            Status_flag status = various::get_enum_incorrect_value<Status_flag>();
            std::size_t words = 0;
        };

        static Result write(Limited<std::uint32_t, s::start, s::start + s::size_in_bytes> a_address,
                            Not_null<const Word*> a_p_data,
                            std::size_t a_size_in_double_words);
        static Result write(Limited<std::uint32_t, s::start, s::start + s::size_in_bytes> a_address,
                            Not_null<const Word*> a_p_data,
                            std::size_t a_size_in_double_words,
                            Milliseconds a_timeout);

        static Result read(Limited<std::uint32_t, s::start, s::start + s::size_in_bytes> a_address,
                           Not_null<void*> a_p_data,
                           Limited<std::size_t, 1, s::page_size_in_bytes> a_size_in_bytes);
        static Result read(Limited<std::uint32_t, s::start, s::start + s::size_in_bytes> a_address,
                           Not_null<void*> a_p_data,
                           Limited<std::size_t, 1, s::page_size_in_bytes> a_size_in_bytes,
                           Milliseconds a_timeout);

        static Result erase_page(Limited<std::uint32_t, 0u, s::pages_count - 1> a_page_index);
        static Result erase_page(Limited<std::uint32_t, 0u, s::pages_count - 1> a_page_index, Milliseconds a_timeout);

        static Result erase_bank(Bank_id a_id);
        static Result erase_bank(Bank_id a_id, Milliseconds a_timeout);
    };

    static void set_latency(Latency a_latency);
    static bool set_latency(Latency a_latency, Milliseconds a_timeout);

    static void set_cache_mode(Cache_mode a_cache_mode);
    static bool set_cache_mode(Cache_mode a_cache_mode, Milliseconds a_timeout);

    static Cache_mode get_cache_mode()
    {
        return static_cast<Cache_mode>(bit::flag::get(FLASH->ACR, FLASH_ACR_PRFTEN));
    }

    static Latency get_latency()
    {
        return static_cast<Latency>(bit::flag::get(FLASH->ACR, FLASH_ACR_LATENCY));
    }
};

} // namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals

namespace xmcu::soc {
template<> class Scoped_guard<st::arm::m0::l0::rm0451::peripherals::internal_flash::unlocker> : private Non_copyable
{
public:
    Scoped_guard()
        : unlocked(false)
    {
        st::arm::m0::l0::rm0451::peripherals::internal_flash::unlocker::unlock();
        this->unlocked = true;
    }

    Scoped_guard(Milliseconds a_timeout)
        : unlocked(st::arm::m0::l0::rm0451::peripherals::internal_flash::unlocker::unlock(a_timeout))
    {
    }

    ~Scoped_guard()
    {
        st::arm::m0::l0::rm0451::peripherals::internal_flash::unlocker::lock();
    }

    bool is_unlocked() const
    {
        return this->unlocked;
    }

private:
    bool unlocked;
};
template<> class Scoped_guard<st::arm::m0::l0::rm0451::peripherals::internal_flash::cache_disabler>
    : private Non_copyable
{
public:
    Scoped_guard()
        : disabled(false)
    {
        st::arm::m0::l0::rm0451::peripherals::internal_flash::cache_disabler::disable();
        this->disabled = true;
    }

    Scoped_guard(Milliseconds a_timeout)
        : disabled(st::arm::m0::l0::rm0451::peripherals::internal_flash::cache_disabler::disable(a_timeout))
    {
    }

    ~Scoped_guard()
    {
        st::arm::m0::l0::rm0451::peripherals::internal_flash::cache_disabler::enable();
    }

    bool is_disabled() const
    {
        return this->disabled;
    }

private:
    bool disabled;
};
} // namespace xmcu::soc