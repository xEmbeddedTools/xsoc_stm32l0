/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// xmcu
#include <rm0451/peripherals/internal_flash/internal_flash.hpp>
#include <rm0451/utils/delay.hpp>
#include <rm0451/utils/tick_counter.hpp>
#include <rm0451/utils/wait_until.hpp>
#include <xmcu/bit.hpp>

// std
#include <cstring>

namespace {
using namespace xmcu;
using namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals;

void clear_FLASH_SR_errors()
{
    ll::internal_flash::Registers* p_reg = ll::internal_flash::registers();
    bit::flag::set(&(p_reg->sr),
                   ll::internal_flash::SR::rderr | ll::internal_flash::SR::wrperr | ll::internal_flash::SR::sizerr |
                       ll::internal_flash::SR::pgaerr);
}

bool is_FLASH_SR_error()
{
    return bit::is_any(ll::internal_flash::registers()->sr,
                       ll::internal_flash::SR::rderr | ll::internal_flash::SR::wrperr | ll::internal_flash::SR::sizerr |
                           ll::internal_flash::SR::pgaerr);
}

internal_flash::Status_flag get_status_flag_from_FLASH_SR()
{
    ll::internal_flash::Registers* p_reg = ll::internal_flash::registers();

    auto sr_flags =
        bit::flag::get(p_reg->sr,
                       ll::internal_flash::SR::rderr | ll::internal_flash::SR::wrperr | ll::internal_flash::SR::sizerr |
                           ll::internal_flash::SR::pgaerr | ll::internal_flash::SR::fwwer |
                           ll::internal_flash::SR::notzeroerr | ll::internal_flash::SR::optverr);

    if (static_cast<ll::internal_flash::SR::Data>(0x0u) == sr_flags &&
        true == bit::flag::is(p_reg->sr, ll::internal_flash::SR::bsy))
    {
        return internal_flash::Status_flag::locked;
    }

    return static_cast<internal_flash::Status_flag>(sr_flags);
}
} // namespace

namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals {
using namespace xmcu;
using namespace xmcu::soc::st::arm::m0::l0::rm0451::system;
using namespace utils;

void internal_flash::unlocker::unlock()
{
    ll::internal_flash::Registers* p_reg = ll::internal_flash::registers();

    wait_until::all_bits_are_cleared(p_reg->sr, ll::internal_flash::SR::bsy);

    if (true == bit::flag::is(p_reg->pecr, ll::internal_flash::PECR::prg_lock))
    {
        Scoped_guard<nvic> interrupt_guard;

        if (true == bit::flag::is(p_reg->pecr, ll::internal_flash::PECR::pe_lock))
        {
            p_reg->pekeyr = 0x89ABCDEFu;
            p_reg->pekeyr = 0x02030405u;
        }

        p_reg->prgkeyr = 0x8C9DAEBFu;
        p_reg->prgkeyr = 0x13141516u;
    }

    wait_until::all_bits_are_cleared(p_reg->pecr, ll::internal_flash::PECR::prg_lock);
}
bool internal_flash::unlocker::unlock(Milliseconds a_timeout)
{
    ll::internal_flash::Registers* p_reg = ll::internal_flash::registers();
    bool isCleared = wait_until::all_bits_are_cleared(p_reg->sr, ll::internal_flash::SR::bsy, a_timeout);
    if (false == isCleared)
    {
        return false;
    }

    if (true == bit::flag::is(p_reg->pecr, ll::internal_flash::PECR::prg_lock))
    {
        Scoped_guard<nvic> interrupt_guard;

        if (true == bit::flag::is(p_reg->pecr, ll::internal_flash::PECR::pe_lock))
        {
            p_reg->pekeyr = 0x89ABCDEFu;
            p_reg->pekeyr = 0x02030405u;
        }

        p_reg->prgkeyr = 0x8C9DAEBFu;
        p_reg->prgkeyr = 0x13141516u;
    }

    return false == bit::flag::is(p_reg->pecr, ll::internal_flash::PECR::prg_lock);
}
void internal_flash::unlocker::lock()
{
    ll::internal_flash::Registers* p_reg = ll::internal_flash::registers();
    bit::flag::set(&(p_reg->pecr), ll::internal_flash::PECR::prg_lock);
}

void internal_flash::cache_disabler::disable()
{
    cache_mode = get_cache_mode();
    set_cache_mode(internal_flash::Cache_mode::disabled);
}
bool internal_flash::cache_disabler::disable(Milliseconds a_timeout)
{
    const std::uint64_t start = tick_counter<Milliseconds>::get();

    cache_mode = get_cache_mode();
    return set_cache_mode(internal_flash::Cache_mode::disabled,
                          a_timeout.get() - (tick_counter<Milliseconds>::get() - start));
}
void internal_flash::cache_disabler::enable()
{
    set_cache_mode(cache_mode);
}
bool internal_flash::cache_disabler::enable(Milliseconds a_timeout)
{
    return set_cache_mode(cache_mode, a_timeout);
}

void internal_flash::set_latency(Latency a_latency)
{
    ll::internal_flash::Registers* p_reg = ll::internal_flash::registers();

    bit::flag::set(
        &(p_reg->acr), ll::internal_flash::ACR::latency, static_cast<ll::internal_flash::ACR::Flag>(a_latency));
    wait_until::all_bits_are_set(p_reg->acr, static_cast<ll::internal_flash::ACR::Flag>(a_latency));
}
bool internal_flash::set_latency(Latency a_latency, Milliseconds a_timeout)
{
    ll::internal_flash::Registers* p_reg = ll::internal_flash::registers();

    bit::flag::set(
        &(p_reg->acr), ll::internal_flash::ACR::latency, static_cast<ll::internal_flash::ACR::Flag>(a_latency));
    return wait_until::all_bits_are_set(p_reg->acr, static_cast<ll::internal_flash::ACR::Flag>(a_latency), a_timeout);
}

void internal_flash::set_cache_mode(Cache_mode a_cache_mode)
{
    ll::internal_flash::Registers* p_reg = ll::internal_flash::registers();
    bit::flag::set(
        &(p_reg->acr), ll::internal_flash::ACR::prften, static_cast<ll::internal_flash::ACR::Flag>(a_cache_mode));
}
bool internal_flash::set_cache_mode(Cache_mode a_cache_mode, Milliseconds a_timeout)
{
    ll::internal_flash::Registers* p_reg = ll::internal_flash::registers();
    bit::flag::set(
        &(p_reg->acr), ll::internal_flash::ACR::prften, static_cast<ll::internal_flash::ACR::Flag>(a_cache_mode));
    return wait_until::all_bits_are_set(
        p_reg->acr, static_cast<ll::internal_flash::ACR::Flag>(a_cache_mode), a_timeout);
}

internal_flash::polling::Result
internal_flash::polling::write(Limited<std::uint32_t, s::start, s::start + s::size_in_bytes> a_address,
                               Not_null<const Word*> a_p_data,
                               std::size_t a_size_in_double_words)
{
    hkm_assert(a_size_in_double_words > 0);

    ll::internal_flash::Registers* p_reg = ll::internal_flash::registers();

    Scoped_guard<unlocker> unlock_guard;

    if (true == is_FLASH_SR_error())
    {
        clear_FLASH_SR_errors();
    }

    Scoped_guard<nvic> interrupt_guard;
    Scoped_guard<cache_disabler> cache_guard;
    bit::flag::set(&p_reg->pecr, ll::internal_flash::PECR::prog);
    for (std::size_t i = 0; i < a_size_in_double_words;)
    {
        wait_until::all_bits_are_cleared(p_reg->sr, ll::internal_flash::SR::bsy);

        volatile std::uint32_t* p_address = reinterpret_cast<volatile std::uint32_t*>(a_address.get());
        *(p_address + i * 2u + 0u) = static_cast<std::uint32_t>(a_p_data[i] >> 0x00u);
        __ISB();
        *(p_address + i * 2u + 1u) = static_cast<std::uint32_t>(a_p_data[i] >> 0x20u);

        wait_until::all_bits_are_cleared(p_reg->sr, ll::internal_flash::SR::bsy);
        if (true == bit::flag::is(p_reg->sr, ll::internal_flash::SR::eop))
        {
            bit::flag::set(&(p_reg->sr), ll::internal_flash::SR::eop);
        }

        i++;
    }

    bit::flag::clear(&p_reg->pecr, ll::internal_flash::PECR::prog);
    return { get_status_flag_from_FLASH_SR(), a_size_in_double_words };
}

internal_flash::polling::Result
internal_flash::polling::write(Limited<std::uint32_t, s::start, s::start + s::size_in_bytes> a_address,
                               Not_null<const Word*> a_p_data,
                               std::size_t a_size_in_double_words,
                               Milliseconds a_timeout)
{
    hkm_assert(a_size_in_double_words > 0);

    ll::internal_flash::Registers* p_reg = ll::internal_flash::registers();

    const std::uint64_t start = tick_counter<Milliseconds>::get();
    Scoped_guard<unlocker> unlock_guard(a_timeout.get() - (tick_counter<Milliseconds>::get() - start));

    if (true == unlock_guard.is_unlocked())
    {
        if (true == is_FLASH_SR_error())
        {
            clear_FLASH_SR_errors();
        }

        bool timeout = false;
        std::size_t i = 0;
        Scoped_guard<nvic> interrupt_guard;
        Scoped_guard<cache_disabler> cache_guard;
        bit::flag::set(&p_reg->pecr, ll::internal_flash::PECR::prog);
        while (i < a_size_in_double_words && false == timeout)
        {
            timeout = false ==
                      wait_until::all_bits_are_cleared(p_reg->sr,
                                                       ll::internal_flash::SR::bsy,
                                                       a_timeout.get() - (tick_counter<Milliseconds>::get() - start));

            if (false == timeout)
            {
                volatile std::uint32_t* p_address = reinterpret_cast<volatile std::uint32_t*>(a_address.get());

                *(p_address + i * 2u + 0u) = static_cast<std::uint32_t>(a_p_data[i] >> 0x00u);
                __ISB();
                *(p_address + i * 2u + 1u) = static_cast<std::uint32_t>(a_p_data[i] >> 0x20u);

                timeout = false == wait_until::all_bits_are_cleared(p_reg->sr,
                                                                    ll::internal_flash::SR::bsy,
                                                                    a_timeout.get() -
                                                                        (tick_counter<Milliseconds>::get() - start));

                if (false == timeout)
                {
                    if (true == bit::flag::is(p_reg->sr, ll::internal_flash::SR::eop))
                    {
                        bit::flag::set(&(p_reg->sr), ll::internal_flash::SR::eop);
                    }
                    i++;
                }
            }
        }
        bit::flag::clear(&p_reg->pecr, ll::internal_flash::PECR::prog);
        return { get_status_flag_from_FLASH_SR(), i };
    }
    return { Status_flag::locked, 0x0u };
}

internal_flash::polling::Result
internal_flash::polling::read(Limited<std::uint32_t, s::start, s::start + s::size_in_bytes> a_address,
                              Not_null<void*> a_p_data,
                              Limited<std::size_t, 1u, s::page_size_in_bytes> a_size_in_bytes)
{
    Scoped_guard<unlocker> unlock_guard;

    if (true == unlock_guard.is_unlocked())
    {
        std::memcpy(a_p_data, reinterpret_cast<const uint8_t*>(a_address.get()), a_size_in_bytes);
    }

    return { get_status_flag_from_FLASH_SR(), a_size_in_bytes };
}

internal_flash::polling::Result
internal_flash::polling::read(Limited<std::uint32_t, s::start, s::start + s::size_in_bytes> a_address,
                              Not_null<void*> a_p_data,
                              Limited<std::size_t, 1, s::page_size_in_bytes> a_size_in_bytes,
                              Milliseconds a_timeout)
{
    hkm_assert(a_timeout > 0_ms);

    const std::uint64_t start = tick_counter<Milliseconds>::get();
    Scoped_guard<unlocker> unlock_guard(a_timeout.get() - (tick_counter<Milliseconds>::get() - start));

    if (true == unlock_guard.is_unlocked())
    {
        std::memcpy(a_p_data, reinterpret_cast<const uint8_t*>(a_address.get()), a_size_in_bytes);
        return { get_status_flag_from_FLASH_SR(), a_size_in_bytes };
    }

    return { Status_flag::locked, 0x0u };
}

internal_flash::polling::Result
internal_flash::polling::erase_page(Limited<std::uint32_t, 0u, s::pages_count - 1> a_page_index)
{
    Scoped_guard<unlocker> unlock_guard;

    ll::internal_flash::Registers* p_reg = ll::internal_flash::registers();

    if (true == is_FLASH_SR_error())
    {
        clear_FLASH_SR_errors();
    }

    do
    {
        wait_until::all_bits_are_cleared(p_reg->sr, ll::internal_flash::SR::bsy);

        Scoped_guard<nvic> interrupt_guard;
        Scoped_guard<cache_disabler> cache_guard;

        bit::flag::set(&p_reg->pecr, ll::internal_flash::PECR::erase | ll::internal_flash::PECR::prog);
        volatile std::uint32_t* page_address =
            reinterpret_cast<volatile std::uint32_t*>(s::start + s::page_size_in_bytes * a_page_index);
        *page_address = 0u;
        wait_until::all_bits_are_cleared(p_reg->sr, ll::internal_flash::SR::bsy);

        if (true == bit::flag::is(p_reg->sr, ll::internal_flash::SR::eop))
        {
            bit::flag::set(&p_reg->sr, ll::internal_flash::SR::eop);
        }
        else
        {
            // TODO: error handling
        }
        bit::flag::clear(&p_reg->pecr, ll::internal_flash::PECR::erase | ll::internal_flash::PECR::prog);
    } while (false);

    return { .status = get_status_flag_from_FLASH_SR(), .words = 1u };
}

internal_flash::polling::Result
internal_flash::polling::erase_page(Limited<std::uint32_t, 0u, s::pages_count - 1> a_page_index, Milliseconds a_timeout)
{
    const std::uint64_t start = tick_counter<Milliseconds>::get();
    Scoped_guard<unlocker> unlock_guard(a_timeout.get() - (tick_counter<Milliseconds>::get() - start));

    ll::internal_flash::Registers* p_reg = ll::internal_flash::registers();

    if (true == unlock_guard.is_unlocked())
    {
        if (true == is_FLASH_SR_error())
        {
            clear_FLASH_SR_errors();
        }

        bool is_timeout = false;
        do
        {
            wait_until::all_bits_are_cleared(
                p_reg->sr, ll::internal_flash::SR::bsy, a_timeout.get() - (tick_counter<Milliseconds>::get() - start));

            Scoped_guard<nvic> interrupt_guard;
            Scoped_guard<cache_disabler> cache_guard;

            bit::flag::set(&p_reg->pecr, ll::internal_flash::PECR::erase | ll::internal_flash::PECR::prog);
            volatile std::uint32_t* page_address =
                reinterpret_cast<volatile std::uint32_t*>(s::start + s::page_size_in_bytes * a_page_index);
            *page_address = 0u;
            is_timeout = wait_until::all_bits_are_cleared(
                p_reg->sr, ll::internal_flash::SR::bsy, a_timeout.get() - (tick_counter<Milliseconds>::get() - start));
            bit::flag::clear(&p_reg->pecr, ll::internal_flash::PECR::erase | ll::internal_flash::PECR::prog);

            if (false == is_timeout) // TODO: it doesn't look like proper handling
            {
                return { .status = get_status_flag_from_FLASH_SR(), .words = 1u };
            }
        } while (false);
    }
    return { Status_flag::locked, 0u };
}
} // namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals