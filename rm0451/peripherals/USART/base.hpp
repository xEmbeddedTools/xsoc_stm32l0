#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// soc
#include <rm0451/config.hpp>

// xmcu
#include <xmcu/non_constructible.hpp>

namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals {
#if defined(XMCU_SOC_MODEL_STM32L010F4P6)
struct usart_base : private non_constructible
{
    enum class _2 : std::uint32_t;
};
#endif
#if defined(XMCU_SOC_MODEL_STM32L010C6T6)
struct usart_base : private non_constructible
{
    enum class _2 : std::uint32_t;
};
#endif
} // namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals