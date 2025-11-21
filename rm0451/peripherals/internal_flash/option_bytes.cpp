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
#include <cstring>

namespace {
using namespace xmcu;

struct Slot
{
    std::uint8_t byte_0 = 0x0u;
    std::uint8_t byte_1 = 0x0u;
    std::uint8_t complementary_byte_0 = 0x0u;
    std::uint8_t complementary_byte_1 = 0x0u;
};

static_assert(sizeof(Slot) == sizeof(std::uint32_t));

struct map : private non_constructible
{
    static constexpr std::uint32_t slot_0 = OB_BASE;
    static constexpr std::uint32_t slot_1 = slot_0 + 4u;
    static constexpr std::uint32_t slot_2 = slot_1 + 4u;
    static constexpr std::uint32_t slot_3 = slot_2 + 4u;
    static constexpr std::uint32_t slot_4 = slot_3 + 4u;
};
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

    if (true == flash_guard.is_unlocked())
    {
        Scoped_guard<option_bytes::unlocker> ob_guard;

        if (true == ob_guard.is_unlocked())
        {
            Slot slot_0_entry = *(reinterpret_cast<Slot*>(map::slot_0));
            bit::flag::set(&slot_0_entry.byte_0, 0xFFu, static_cast<std::uint8_t>(level_a));
            slot_0_entry.complementary_byte_0 = static_cast<std::uint8_t>(~(slot_0_entry.byte_0));

            std::uint32_t value = 0x0u;
            std::memcpy(&value, &slot_0_entry, sizeof(slot_0_entry));
            std::memcpy((reinterpret_cast<std::uint32_t*>(map::slot_0)), &value, sizeof(value));

            wait_until::all_bits_are_cleared(FLASH->SR, FLASH_SR_BSY);

            if (true == bit::flag::is(FLASH->SR, FLASH_SR_EOP))
            {
                FLASH->SR = FLASH_SR_EOP;
                return true;
            }
        }
    }

    return false;
}
bool option_bytes::RDP::set(Level level_a, Milliseconds timeout_a)
{
    const std::uint64_t start = tick_counter<Milliseconds>::get();

    Scoped_guard<internal_flash::unlocker> flash_guard(timeout_a.get() - (tick_counter<Milliseconds>::get() - start));

    if (true == flash_guard.is_unlocked())
    {
        Scoped_guard<option_bytes::unlocker> ob_guard(timeout_a.get() - (tick_counter<Milliseconds>::get() - start));

        if (true == ob_guard.is_unlocked())
        {
            Slot slot_0_entry = *(reinterpret_cast<Slot*>(map::slot_0));
            bit::flag::set(&slot_0_entry.byte_0, 0xFFu, static_cast<std::uint8_t>(level_a));
            slot_0_entry.complementary_byte_0 = static_cast<std::uint8_t>(~(slot_0_entry.byte_0));

            std::uint32_t value = 0x0u;
            std::memcpy(&value, &slot_0_entry, sizeof(slot_0_entry));
            std::memcpy((reinterpret_cast<std::uint32_t*>(map::slot_0)), &value, sizeof(value));

            if (true == wait_until::all_bits_are_cleared(
                            FLASH->SR, FLASH_SR_BSY, timeout_a.get() - (tick_counter<Milliseconds>::get() - start)))
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

    if (true == flash_guard.is_unlocked())
    {
        Scoped_guard<option_bytes::unlocker> ob_guard;

        if (true == ob_guard.is_unlocked())
        {
            Slot slot_1_entry = *(reinterpret_cast<Slot*>(map::slot_1));
            bit::flag::set(&slot_1_entry.byte_0, 0xFu, static_cast<std::uint8_t>(level_a));
            slot_1_entry.complementary_byte_0 = static_cast<std::uint8_t>(~(slot_1_entry.byte_0));

            std::uint32_t value = 0x0u;
            std::memcpy(&value, &slot_1_entry, sizeof(slot_1_entry));
            std::memcpy((reinterpret_cast<std::uint32_t*>(map::slot_1)), &value, sizeof(value));

            wait_until::all_bits_are_cleared(FLASH->SR, FLASH_SR_BSY);

            if (true == bit::flag::is(FLASH->SR, FLASH_SR_EOP))
            {
                FLASH->SR = FLASH_SR_EOP;
                return true;
            }
        }
    }

    return false;
}
bool option_bytes::BOR::set(Level level_a, Milliseconds timeout_a)
{
    const std::uint64_t start = tick_counter<Milliseconds>::get();

    Scoped_guard<internal_flash::unlocker> flash_guard(timeout_a.get() - (tick_counter<Milliseconds>::get() - start));

    if (true == flash_guard.is_unlocked())
    {
        Scoped_guard<option_bytes::unlocker> ob_guard(timeout_a.get() - (tick_counter<Milliseconds>::get() - start));

        if (true == ob_guard.is_unlocked())
        {
            Slot slot_1_entry = *(reinterpret_cast<Slot*>(map::slot_1));
            bit::flag::set(&slot_1_entry.byte_0, 0xFu, static_cast<std::uint8_t>(level_a));
            slot_1_entry.complementary_byte_0 = static_cast<std::uint8_t>(~(slot_1_entry.byte_0));

            std::uint32_t value = 0x0u;
            std::memcpy(&value, &slot_1_entry, sizeof(slot_1_entry));
            std::memcpy((reinterpret_cast<std::uint32_t*>(map::slot_1)), &value, sizeof(value));

            if (true == wait_until::all_bits_are_cleared(
                            FLASH->SR, FLASH_SR_BSY, timeout_a.get() - (tick_counter<Milliseconds>::get() - start)))
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

    if (true == flash_guard.is_unlocked())
    {
        Scoped_guard<option_bytes::unlocker> ob_guard;

        if (true == ob_guard.is_unlocked())
        {
            std::uint8_t byte_0_mask = (static_cast<std::uint32_t>(flags_a)) & 0xFFu;
            std::uint8_t byte_1_mask = (static_cast<std::uint32_t>(flags_a) >> 8u) & 0xFFu;

            Slot slot_1_entry = *(reinterpret_cast<Slot*>(map::slot_1));

            bit::flag::set(&slot_1_entry.byte_0, 0x70u, byte_0_mask);
            bit::flag::set(&slot_1_entry.byte_1, 0x80u, byte_1_mask);

            slot_1_entry.complementary_byte_0 = static_cast<std::uint8_t>(~(slot_1_entry.byte_0));
            slot_1_entry.complementary_byte_1 = static_cast<std::uint8_t>(~(slot_1_entry.byte_1));

            std::uint32_t value = 0x0u;
            std::memcpy(&value, &slot_1_entry, sizeof(slot_1_entry));
            std::memcpy((reinterpret_cast<std::uint32_t*>(map::slot_1)), &value, sizeof(value));

            wait_until::all_bits_are_cleared(FLASH->SR, FLASH_SR_BSY);

            if (true == bit::flag::is(FLASH->SR, FLASH_SR_EOP))
            {
                FLASH->SR = FLASH_SR_EOP;
                return true;
            }
        }
    }

    return false;
}
bool option_bytes::USR::set(Flags flags_a, xmcu::Milliseconds timeout_a)
{
    const std::uint64_t start = tick_counter<Milliseconds>::get();

    Scoped_guard<internal_flash::unlocker> flash_guard(timeout_a.get() - (tick_counter<Milliseconds>::get() - start));

    if (true == flash_guard.is_unlocked())
    {
        Scoped_guard<option_bytes::unlocker> ob_guard(timeout_a.get() - (tick_counter<Milliseconds>::get() - start));

        if (true == ob_guard.is_unlocked())
        {
            std::uint8_t byte_0_mask = (static_cast<std::uint32_t>(flags_a)) & 0xFFu;
            std::uint8_t byte_1_mask = (static_cast<std::uint32_t>(flags_a) >> 8u) & 0xFFu;

            Slot slot_1_entry = *(reinterpret_cast<Slot*>(map::slot_1));

            bit::flag::set(&slot_1_entry.byte_0, 0x70u, byte_0_mask);
            bit::flag::set(&slot_1_entry.byte_1, 0x80u, byte_1_mask);

            slot_1_entry.complementary_byte_0 = static_cast<std::uint8_t>(~(slot_1_entry.byte_0));
            slot_1_entry.complementary_byte_1 = static_cast<std::uint8_t>(~(slot_1_entry.byte_1));

            std::uint32_t value = 0x0u;
            std::memcpy(&value, &slot_1_entry, sizeof(slot_1_entry));
            std::memcpy((reinterpret_cast<std::uint32_t*>(map::slot_1)), &value, sizeof(value));

            wait_until::all_bits_are_cleared(
                FLASH->SR, FLASH_SR_BSY, timeout_a.get() - (tick_counter<Milliseconds>::get() - start));

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

    if (true == flash_guard.is_unlocked())
    {
        Scoped_guard<option_bytes::unlocker> ob_guard;

        if (true == ob_guard.is_unlocked())
        {
            bit::flag::set(&FLASH->PECR, FLASH_PECR_OBL_LAUNCH);
            return false; // we should never get to this point
        }
    }

    return false;
}
bool option_bytes::launch(Milliseconds timeout_a)
{
    const std::uint64_t start = tick_counter<Milliseconds>::get();

    Scoped_guard<internal_flash::unlocker> flash_guard(timeout_a.get() - (tick_counter<Milliseconds>::get() - start));

    if (true == flash_guard.is_unlocked())
    {
        Scoped_guard<option_bytes::unlocker> ob_guard(timeout_a.get() - (tick_counter<Milliseconds>::get() - start));

        if (true == ob_guard.is_unlocked())
        {
            bit::flag::set(&FLASH->PECR, FLASH_PECR_OBL_LAUNCH);
            return false; // we should never get to this point
        }
    }

    return false;
}
} // namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals