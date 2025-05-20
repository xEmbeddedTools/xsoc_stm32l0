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
#include <rm0451/clocks/sources/hsi16.hpp>
#include <rm0451/clocks/sources/msi.hpp>
#include <rm0451/rcc.hpp>
#include <rm0451/system/mcu/mcu.hpp>
#include <xmcu/bit.hpp>
#include <xmcu/non_constructible.hpp>

namespace xmcu::soc::st::arm::m0::l0::rm0451::system {
template<typename MCU_t> class pwr : private non_constructible
{
};

template<> class pwr<mcu<1u>> : private non_constructible
{
public:
    enum class Voltage_scaling : std::uint32_t
    {
        _1 = PWR_CR_VOS_0,
        _2 = PWR_CR_VOS_1,
        _3 = PWR_CR_VOS
    };

    struct stop_mode : private non_constructible
    {
        enum class Type : std::uint32_t
        {
            stop = 0,
            standby = PWR_CR_PDDS,
        };
        enum class Method : std::uint32_t
        {
            wfi,
            wfe,
            none
        };
        enum class Sleeponexit : std::uint32_t
        {
            disabled = 0x0u,
            enabled = SCB_SCR_SLEEPONEXIT_Msk
        };

        static void enter(Type a_type, Method a_method, Sleeponexit a_sleeponexit);
        static void exit();
    };
    struct backup_domain : private xmcu::non_constructible
    {
        static bool enable(xmcu::Milliseconds timeout_a);
        static bool disable(xmcu::Milliseconds timeout_a);
    };

    static void set_voltage_scaling(Voltage_scaling a_scaling);
    static bool set_voltage_scaling(Voltage_scaling a_scaling, Milliseconds a_timeout);

    static Voltage_scaling get_voltage_scaling();
};
} // namespace xmcu::soc::st::arm::m0::l0::rm0451::system

namespace xmcu::soc::st::arm::m0::l0::rm0451 {
template<> class rcc<system::pwr<system::mcu<1u>>> : private non_constructible
{
public:
    static void enable()
    {
        bit::flag::set(&RCC->APB1ENR, RCC_APB1ENR_PWREN);
    }
    static void disable()
    {
        bit::flag::clear(&RCC->APB1ENR, RCC_APB1ENR_PWREN);
    }
};
} // namespace xmcu::soc::st::arm::m0::l0::rm0451
