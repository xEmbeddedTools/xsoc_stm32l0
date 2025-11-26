/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// this
#include <rm0451/peripherals/internal_flash/option_bytes.hpp>

// externals
#include <stm32l0xx.h>

// xmcu
#include <rm0451/peripherals/internal_flash/internal_flash.hpp>
#include <rm0451/utils/tick_counter.hpp>
#include <rm0451/utils/wait_until.hpp>
#include <soc/Scoped_guard.hpp>
#include <soc/st/arm/m0/nvic.hpp>
#include <xmcu/bit.hpp>

// std
#include <memory>

namespace {
using namespace xmcu;

struct Slot
{
    std::uint8_t byte_0 = 0x0u;
    std::uint8_t byte_1 = 0x0u;
    std::uint8_t complementary_byte_0 = 0x0u;
    std::uint8_t complementary_byte_1 = 0x0u;

    Slot() = default;
    Slot(volatile const Slot& slot_a)
        : byte_0(slot_a.byte_0)
        , byte_1(slot_a.byte_1)
        , complementary_byte_0(slot_a.complementary_byte_0)
        , complementary_byte_1(slot_a.complementary_byte_1)
    {
    }

    // return value cannot be reference!
    Slot operator=(const volatile Slot& slot_a) volatile
    {
        *(reinterpret_cast<volatile std::uint32_t*>(this)) =
            *(reinterpret_cast<volatile const std::uint32_t*>(&slot_a));
        return *this;
    }
};

struct Option_bytes
{
    volatile Slot slot_0;
    volatile Slot slot_1;
    volatile Slot slot_2;
    volatile Slot slot_3;
    volatile Slot slot_4;
};

static_assert(sizeof(Slot) == sizeof(std::uint32_t));
static_assert(sizeof(Option_bytes) == sizeof(Slot) * 5u);

Option_bytes* p_option_bytes = reinterpret_cast<Option_bytes*>(OB_BASE);
} // namespace

namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals {
using namespace xmcu;
using namespace xmcu::soc::st::arm::m0::l0::rm0451::utils;
using namespace xmcu::soc::st::arm::m0::l0::rm0451::system;

void option_bytes::unlocker::unlock()
{
    wait_until::all_bits_are_cleared(FLASH->SR, FLASH_SR_BSY);
    if (true == bit::flag::is(FLASH->PECR, FLASH_PECR_OPTLOCK))
    {
        Scoped_guard<nvic> interrupt_guard;
        if (true == bit::flag::is(FLASH->PECR, FLASH_PECR_PELOCK))
        {
            FLASH->PEKEYR = 0x89ABCDEFu;
            FLASH->PEKEYR = 0x02030405u;
        }

        FLASH->OPTKEYR = 0xFBEAD9C8u;
        FLASH->OPTKEYR = 0x24252627u;
    }
}
bool option_bytes::unlocker::unlock(Milliseconds a_timeout)
{
    bool cleared = wait_until::all_bits_are_cleared(FLASH->SR, FLASH_SR_BSY, a_timeout);
    if (false == cleared)
    {
        return false;
    }

    if (true == bit::flag::is(FLASH->PECR, FLASH_PECR_OPTLOCK))
    {
        Scoped_guard<nvic> interrupt_guard;
        if (true == bit::flag::is(FLASH->PECR, FLASH_PECR_PELOCK))
        {
            FLASH->PEKEYR = 0x89ABCDEFu;
            FLASH->PEKEYR = 0x02030405u;
        }

        FLASH->OPTKEYR = 0xFBEAD9C8u;
        FLASH->OPTKEYR = 0x24252627u;
    }

    return true;
}
void option_bytes::unlocker::lock()
{
    bit::flag::set(&(FLASH->PECR), FLASH_PECR_OPTLOCK);
}

bool option_bytes::RDP::set(Level level_a)
{
    Scoped_guard<internal_flash::unlocker> flash_guard;
    Scoped_guard<option_bytes::unlocker> ob_guard;

    Slot slot = p_option_bytes->slot_0;
    bit::flag::set(&slot.byte_0, 0xFFu, static_cast<std::uint8_t>(level_a));
    slot.complementary_byte_0 = static_cast<std::uint8_t>(~(slot.byte_0));
    p_option_bytes->slot_0 = slot;

    wait_until::all_bits_are_cleared(FLASH->SR, FLASH_SR_BSY);

    if (true == bit::flag::is(FLASH->SR, FLASH_SR_EOP))
    {
        FLASH->SR = FLASH_SR_EOP;
        return true;
    }

    return false;
}
bool option_bytes::RDP::set(Level level_a, Milliseconds timeout_a)
{
    const std::uint64_t timeout_timestamp = tick_counter<Milliseconds>::get() + timeout_a.get();
    Scoped_guard<internal_flash::unlocker> flash_guard(timeout_timestamp - tick_counter<Milliseconds>::get());

    if (true == flash_guard.is_unlocked())
    {
        Scoped_guard<option_bytes::unlocker> ob_guard(timeout_timestamp - tick_counter<Milliseconds>::get());

        if (true == ob_guard.is_unlocked())
        {
            Slot slot = p_option_bytes->slot_0;
            bit::flag::set(&slot.byte_0, 0xFFu, static_cast<std::uint8_t>(level_a));
            slot.complementary_byte_0 = static_cast<std::uint8_t>(~(slot.byte_0));
            p_option_bytes->slot_0 = slot;

            if (true == wait_until::all_bits_are_cleared(
                            FLASH->SR, FLASH_SR_BSY, timeout_timestamp - tick_counter<Milliseconds>::get()))
            {
                if (true == bit::flag::is(FLASH->SR, FLASH_SR_EOP))
                {
                    FLASH->SR = FLASH_SR_EOP;
                    return true;
                }
            }
        }
    }

    return false;
}

option_bytes::RDP::Level option_bytes::RDP::get()
{
    const std::uint32_t reg_val = bit::flag::get(FLASH->OPTR, FLASH_OPTR_RDPROT);

    switch (reg_val)
    {
        case 0xAAu:
            return Level::_0;
        case 0xCCu:
            return Level::_2;
    }

    return Level::_1;
}

bool option_bytes::BOR::set(Level level_a)
{
    Scoped_guard<internal_flash::unlocker> flash_guard;
    Scoped_guard<option_bytes::unlocker> ob_guard;

    Slot slot = p_option_bytes->slot_1;
    bit::flag::set(&slot.byte_0, 0xFu, static_cast<std::uint8_t>(level_a));
    slot.complementary_byte_0 = static_cast<std::uint8_t>(~(slot.byte_0));
    p_option_bytes->slot_1 = slot;

    wait_until::all_bits_are_cleared(FLASH->SR, FLASH_SR_BSY);

    if (true == bit::flag::is(FLASH->SR, FLASH_SR_EOP))
    {
        FLASH->SR = FLASH_SR_EOP;
        return true;
    }

    return false;
}
bool option_bytes::BOR::set(Level level_a, Milliseconds timeout_a)
{
    const std::uint64_t timeout_timestamp = tick_counter<Milliseconds>::get() + timeout_a.get();
    Scoped_guard<internal_flash::unlocker> flash_guard(timeout_timestamp - tick_counter<Milliseconds>::get());

    if (true == flash_guard.is_unlocked())
    {
        Scoped_guard<option_bytes::unlocker> ob_guard(timeout_timestamp - tick_counter<Milliseconds>::get());

        if (true == ob_guard.is_unlocked())
        {
            Slot slot = p_option_bytes->slot_1;
            bit::flag::set(&slot.byte_0, 0xFu, static_cast<std::uint8_t>(level_a));
            slot.complementary_byte_0 = static_cast<std::uint8_t>(~(slot.byte_0));
            p_option_bytes->slot_1 = slot;

            if (true == wait_until::all_bits_are_cleared(
                            FLASH->SR, FLASH_SR_BSY, timeout_timestamp - tick_counter<Milliseconds>::get()))
            {
                if (true == bit::flag::is(FLASH->SR, FLASH_SR_EOP))
                {
                    FLASH->SR = FLASH_SR_EOP;
                    return true;
                }
            }
        }
    }

    return false;
}

option_bytes::BOR::Level option_bytes::BOR::get()
{
    return static_cast<Level>(bit::flag::get(FLASH->OPTR, FLASH_OPTR_BOR_LEV) >> FLASH_OPTR_BOR_LEV_Pos);
}

bool option_bytes::USR::set(Flags flags_a)
{
    Scoped_guard<internal_flash::unlocker> flash_guard;

    Scoped_guard<option_bytes::unlocker> ob_guard;

    std::uint8_t byte_0_mask = (static_cast<std::uint32_t>(flags_a)) & 0xFFu;
    std::uint8_t byte_1_mask = (static_cast<std::uint32_t>(flags_a) >> 8u) & 0xFFu;

    Slot slot = p_option_bytes->slot_1;

    bit::flag::set(&slot.byte_0, 0x70u, byte_0_mask);
    bit::flag::set(&slot.byte_1, 0x80u, byte_1_mask);

    slot.complementary_byte_0 = static_cast<std::uint8_t>(~(slot.byte_0));
    slot.complementary_byte_1 = static_cast<std::uint8_t>(~(slot.byte_1));

    p_option_bytes->slot_1 = slot;

    wait_until::all_bits_are_cleared(FLASH->SR, FLASH_SR_BSY);

    if (true == bit::flag::is(FLASH->SR, FLASH_SR_EOP))
    {
        FLASH->SR = FLASH_SR_EOP;
        return true;
    }

    return false;
}
bool option_bytes::USR::set(Flags flags_a, xmcu::Milliseconds timeout_a)
{
    const std::uint64_t timeout_timestamp = tick_counter<Milliseconds>::get() + timeout_a.get();
    Scoped_guard<internal_flash::unlocker> flash_guard(timeout_timestamp - tick_counter<Milliseconds>::get());

    if (true == flash_guard.is_unlocked())
    {
        Scoped_guard<option_bytes::unlocker> ob_guard(timeout_timestamp - tick_counter<Milliseconds>::get());

        if (true == ob_guard.is_unlocked())
        {
            std::uint8_t byte_0_mask = (static_cast<std::uint32_t>(flags_a)) & 0xFFu;
            std::uint8_t byte_1_mask = (static_cast<std::uint32_t>(flags_a) >> 8u) & 0xFFu;

            Slot slot = p_option_bytes->slot_1;

            bit::flag::set(&slot.byte_0, 0x70u, byte_0_mask);
            bit::flag::set(&slot.byte_1, 0x80u, byte_1_mask);

            slot.complementary_byte_0 = static_cast<std::uint8_t>(~(slot.byte_0));
            slot.complementary_byte_1 = static_cast<std::uint8_t>(~(slot.byte_1));

            p_option_bytes->slot_1 = slot;

            wait_until::all_bits_are_cleared(
                FLASH->SR, FLASH_SR_BSY, timeout_timestamp - tick_counter<Milliseconds>::get());

            if (true == bit::flag::is(FLASH->SR, FLASH_SR_EOP))
            {
                FLASH->SR = FLASH_SR_EOP;
                return true;
            }
        }
    }

    return false;
}

option_bytes::USR::Flags option_bytes::USR::get()
{
    return static_cast<Flags>((FLASH->OPTR >> 16u) & 0x8070u /*mask for bits: 15,6,5,4*/);
}

bool option_bytes::launch()
{
    Scoped_guard<internal_flash::unlocker> flash_guard;
    Scoped_guard<option_bytes::unlocker> ob_guard;

    bit::flag::set(&FLASH->PECR, FLASH_PECR_OBL_LAUNCH);
    return false; // we should never get to this point
}
bool option_bytes::launch(Milliseconds timeout_a)
{
    const std::uint64_t timeout_timestamp = tick_counter<Milliseconds>::get() + timeout_a.get();
    Scoped_guard<internal_flash::unlocker> flash_guard(timeout_timestamp - tick_counter<Milliseconds>::get());

    if (true == flash_guard.is_unlocked())
    {
        Scoped_guard<option_bytes::unlocker> ob_guard(timeout_timestamp - tick_counter<Milliseconds>::get());

        if (true == ob_guard.is_unlocked())
        {
            bit::flag::set(&FLASH->PECR, FLASH_PECR_OBL_LAUNCH);
            return false; // we should never get to this point
        }
    }

    return false;
}
} // namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals