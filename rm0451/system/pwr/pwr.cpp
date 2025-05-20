/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// this
#include <rm0451/system/pwr/pwr.hpp>

// xmcu
#include <rm0451/clocks/sysclk.hpp>
#include <rm0451/peripherals/internal_flash/internal_flash.hpp>
#include <rm0451/rcc.hpp>
#include <rm0451/system/mcu/mcu.hpp>
#include <rm0451/utils/wait_until.hpp>

namespace xmcu::soc::st::arm::m0::l0::rm0451::system {
using namespace xmcu;
using namespace xmcu::soc::st::arm::m0::l0::rm0451::clocks;
using namespace xmcu::soc::st::arm::m0::l0::rm0451::clocks::sources;
using namespace xmcu::soc::st::arm::m0::l0::rm0451::system;
using namespace xmcu::soc::st::arm::m0::l0::rm0451::utils;
using namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals;

void pwr<mcu<1u>>::stop_mode::enter(Type a_type, Method a_method, Sleeponexit a_sleeponexit)
{
    bit::flag::set(&PWR->CR, PWR_CR_CWUF | PWR_CR_LPSDSR | PWR_CR_ULP);
    bit::flag::set(&SCB->SCR, static_cast<std::uint32_t>(a_sleeponexit) | SCB_SCR_SLEEPDEEP_Msk);
    switch (a_method)
    {
        case Method::wfe: {
            __SEV();
            __WFE();
            __WFE();
        }
        break;

        case Method::wfi: {
            __WFI();
        }
        break;

        case Method::none: {
        }
        break;
    }
}

void pwr<mcu<1u>>::stop_mode::exit()
{
    if (0 == bit::flag::is(RCC->CFGR, RCC_CFGR_STOPWUCK))
    {
        sysclk<1u>::set_source<msi>();
    }
    else
    {
        sysclk<1u>::set_source<hsi16>();
    }
}

void pwr<mcu<1u>>::set_voltage_scaling(Voltage_scaling a_scaling)
{
    bit::flag::set(&(PWR->CR), PWR_CR_VOS, static_cast<std::uint32_t>(a_scaling));
    wait_until::all_bits_are_cleared(PWR->CSR, PWR_CSR_VOSF);
}

bool pwr<mcu<1u>>::set_voltage_scaling(Voltage_scaling a_scaling, Milliseconds a_timeout)
{
    bit::flag::set(&(PWR->CR), PWR_CR_VOS, static_cast<std::uint32_t>(a_scaling));
    return wait_until::all_bits_are_cleared(PWR->CSR, PWR_CSR_VOSF, a_timeout);
}

pwr<mcu<1u>>::Voltage_scaling pwr<mcu<1u>>::get_voltage_scaling()
{
    return static_cast<Voltage_scaling>(bit::flag::get(PWR->CR, PWR_CR_VOS));
}

bool pwr<mcu<1u>>::backup_domain::enable(Milliseconds timeout_a)
{
    bit::flag::set(&(PWR->CR), PWR_CR_DBP);
    return wait_until::all_bits_are_cleared(PWR->CR, PWR_CR_DBP, timeout_a);
}

bool pwr<mcu<1u>>::backup_domain::disable(Milliseconds timeout_a)
{
    bit::flag::clear(&(PWR->CR), PWR_CR_DBP);
    return wait_until::all_bits_are_set(PWR->CR, PWR_CR_DBP, timeout_a);
}
} // namespace xmcu::soc::st::arm::m0::l0::rm0451::system