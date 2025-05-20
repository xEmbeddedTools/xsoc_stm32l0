#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// std
#include <cstddef>
#include <cstdint>

// externals
#include <stm32l0xx.h>

// xmcu
#include <xmcu/bit.hpp>
#include <xmcu/non_constructible.hpp>

namespace xmcu::soc::st::arm::m0::l0::rm0451::system {
template<std::size_t id> class mcu : private non_constructible
{
};
template<> class mcu<1u> : private non_constructible
{
public:
    enum class Reset_source : std::uint32_t
    {
        low_power = RCC_CSR_LPWRRSTF,
        window_watchdog = RCC_CSR_WWDGRSTF,
        independent_window_watchdog = RCC_CSR_IWDGRSTF,
        software = RCC_CSR_SFTRSTF,
        power_on = RCC_CSR_PORRSTF,
        pin = RCC_CSR_PINRSTF,
        option_byte = RCC_CSR_OBLRSTF
    };

    struct Id
    {
        std::uint8_t serial_number[12] = { 0 };
        std::uint32_t type = 0;
    };

    static void halt()
    {
        __disable_irq();
        __builtin_trap();

        while (true) continue;
    }

    static Id get_id()
    {
        const std::uint8_t* p_id_location = reinterpret_cast<std::uint8_t*>(UID_BASE);

        return { { p_id_location[0],
                   p_id_location[1],
                   p_id_location[2],
                   p_id_location[3],
                   p_id_location[4],
                   p_id_location[5],
                   p_id_location[6],
                   p_id_location[7],
                   p_id_location[8],
                   p_id_location[9],
                   p_id_location[10],
                   p_id_location[11] },

                 DBGMCU->IDCODE };
    }

    static void set_vector_table_address(std::uint32_t a_address)
    {
        SCB->VTOR = a_address;
    }

    static void set_main_stack_pointer(std::uint32_t a_address)
    {
        __set_MSP(a_address);
    }

    static Reset_source get_reset_source()
    {
        return static_cast<Reset_source>(RCC->CSR & 0xFE000000u);
    }

    static std::uint32_t get_unique_device_number()
    {
        return static_cast<std::uint32_t>((*(reinterpret_cast<uint32_t*>(UID_BASE + 0x14))));
    }
};

constexpr mcu<1u>::Reset_source operator~(mcu<1u>::Reset_source a_flag)
{
    return static_cast<mcu<1u>::Reset_source>(~static_cast<std::uint32_t>(a_flag));
}

constexpr mcu<1u>::Reset_source operator|(mcu<1u>::Reset_source left_a, mcu<1u>::Reset_source right_a)
{
    return static_cast<mcu<1u>::Reset_source>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr mcu<1u>::Reset_source operator&(mcu<1u>::Reset_source left_a, mcu<1u>::Reset_source right_a)
{
    return static_cast<mcu<1u>::Reset_source>(static_cast<std::uint32_t>(left_a) & static_cast<std::uint32_t>(right_a));
}

constexpr mcu<1u>::Reset_source& operator&=(mcu<1u>::Reset_source& left_a, mcu<1u>::Reset_source right_a)
{
    left_a = (left_a & right_a);
    return left_a;
}
constexpr mcu<1u>::Reset_source& operator|=(mcu<1u>::Reset_source& left_a, mcu<1u>::Reset_source right_a)
{
    left_a = (left_a | right_a);
    return left_a;
}
} // namespace xmcu::soc::st::arm::m0::l0::rm0451::system