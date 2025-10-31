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
#include <rm0451/config.hpp>
#include <rm0451/defs.hpp>
#include <rm0451/rcc.hpp>
#include <xmcu/Non_copyable.hpp>
#include <xmcu/bit.hpp>
#include <xmcu/non_constructible.hpp>
#include <xmcu/various.hpp>

namespace xmcu::soc::st::arm::m0::l0::rm0451 {

template<typename Perihperal_t = void*> class DMA : private xmcu::Non_copyable
{
public:
#if defined(XMCU_DMA1_PRESENT)
    enum class _1;
#endif

    enum class Priority : std::uint32_t
    {
        very_high = DMA_CCR_PL_0 | DMA_CCR_PL_1,
        high = DMA_CCR_PL_1,
        medium = DMA_CCR_PL_0,
        low = 0x0u
    };

    enum class Mode : std::uint32_t
    {
        single = 0x0u,
        circular = DMA_CCR_CIRC
    };

    enum class Channel : std::uint32_t
    {
        _1,
        _2,
        _3,
        _4,
        _5,
#if STM32L010XX_DMA_CHANNEL_COUNT > 5
        _6,
        _7
#endif
    };

    enum class Event_flag : std::uint32_t
    {
        none = 0x0u,
        full_transfer_complete = 0x1u,
        half_transfer_complete = 0x2u,
        transfer_error = 0x4u
    };

    // Hardcoded values from cube - no info about specific numbers in RM/Datasheet
    // TODO: Are they different between l010 family members?
    enum class Request
    {
        usart2 = 4,
        lpuart1 = 5,
    };

    struct Result
    {
        Event_flag event = various::get_enum_incorrect_value<Event_flag>();
        std::size_t data_length_in_words = 0;
    };
    struct Callback
    {
        using Function = void (*)(Event_flag a_event, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    static constexpr std::uint32_t channel_count = STM32L010XX_DMA_CHANNEL_COUNT;
};

constexpr DMA<>::Event_flag operator|(DMA<>::Event_flag a_f1, DMA<>::Event_flag a_f2)
{
    return static_cast<DMA<>::Event_flag>(static_cast<std::uint32_t>(a_f1) | static_cast<std::uint32_t>(a_f2));
}

constexpr DMA<>::Event_flag operator&(DMA<>::Event_flag a_f1, DMA<>::Event_flag a_f2)
{
    return static_cast<DMA<>::Event_flag>(static_cast<std::uint32_t>(a_f1) & static_cast<std::uint32_t>(a_f2));
}

constexpr DMA<>::Event_flag operator|=(DMA<>::Event_flag& a_f1, DMA<>::Event_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

template<> class rcc<DMA<>, DMA<>::_1> : private xmcu::non_constructible
{
public:
    static void enable()
    {
        bit::flag::set(&(RCC->AHBENR), RCC_AHBENR_DMA1EN);
    }
    static void disable()
    {
        bit::flag::clear(&(RCC->AHBENR), RCC_AHBENR_DMA1EN);
    }
};

} // namespace xmcu::soc::st::arm::m0::l0::rm0451