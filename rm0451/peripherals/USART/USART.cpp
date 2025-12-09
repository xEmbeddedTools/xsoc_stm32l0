/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// this
#include <rm0451/peripherals/USART/LPUART.hpp>
#include <rm0451/peripherals/USART/USART.hpp>

// xmcu
#include <rm0451/utils/tick_counter.hpp>
#include <rm0451/utils/wait_until.hpp>
#include <soc/Scoped_guard.hpp>
#include <soc/st/arm/m0/nvic.hpp>

// debug
#include <xmcu/assertion.hpp>

namespace {
using namespace xmcu;
using namespace xmcu::soc::st::arm::m0;
using namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals;
using namespace xmcu::soc::st::arm::m0::l0::rm0451::utils;

constexpr std::uint32_t clock_prescaler_lut[] = { 1u, 2u, 4u, 6u, 8u, 10u, 12u, 16u, 32u, 64u, 128u, 256u };

bool is_rx_error(ll::usart::Registers* a_p_registers)
{
    return bit::is_any(a_p_registers->isr,
                       ll::usart::ISR::pe | ll::usart::ISR::fe | ll::usart::ISR::ore | ll::usart::ISR::ne);
}

bool is_tx_error(ll::usart::Registers* a_p_registers)
{
    return bit::is_any(a_p_registers->isr, ll::usart::ISR::pe | ll::usart::ISR::ne);
}

USART::Event_flag get_Event_flag_and_clear(ll::usart::ICR* a_p_icr, ll::usart::ISR a_isr)
{
    USART::Event_flag pending_events = USART::Event_flag::none;
    ll::usart::ICR::Flag clear_mask = ll::usart::ICR::none;

    if (true == bit::flag::is(a_isr, ll::usart::ISR::idle))
    {
        clear_mask |= ll::usart::ICR::idlecf;
        pending_events |= USART::Event_flag::idle;
    }
    if (true == bit::flag::is(a_isr, ll::usart::ISR::tc))
    {
        clear_mask |= ll::usart::ICR::tccf;
        pending_events |= USART::Event_flag::transfer_complete;
    }

    if (true == bit::flag::is(a_isr, ll::usart::ISR::wuf))
    {
        clear_mask |= ll::usart::ICR::wucf;
        pending_events |= USART::Event_flag::wakeup_from_stop;
    }
    if (true == bit::flag::is(a_isr, ll::usart::ISR::cmf))
    {
        clear_mask |= ll::usart::ICR::cmcf;
        pending_events |= USART::Event_flag::character_matched;
    }

    if (true == bit::is_any(a_isr, ll::usart::ISR::pe | ll::usart::ISR::fe | ll::usart::ISR::ore | ll::usart::ISR::ne))
    {
        if (true == bit::flag::is(a_isr, ll::usart::ISR::pe))
        {
            clear_mask |= ll::usart::ICR::pecf;
            pending_events |= USART::Event_flag::parity_error;
        }
        if (true == bit::flag::is(a_isr, ll::usart::ISR::fe))
        {
            clear_mask |= ll::usart::ICR::fecf;
            pending_events |= USART::Event_flag::framing_error;
        }
        if (true == bit::flag::is(a_isr, ll::usart::ISR::ore))
        {
            clear_mask |= ll::usart::ICR::orecf;
            pending_events |= USART::Event_flag::overrun;
        }
        if (true == bit::flag::is(a_isr, ll::usart::ISR::ne))
        {
            clear_mask |= ll::usart::ICR::ncf;
            pending_events |= USART::Event_flag::noise_detected;
        }
    }

    bit::flag::set(a_p_icr, clear_mask);
    return pending_events;
}

USART::Event_flag get_pending_events(ll::usart::ISR::Data a_isr)
{
    USART::Event_flag pending_events = USART::Event_flag::none;

    if (true == bit::flag::is(a_isr, ll::usart::ISR::idle))
    {
        pending_events |= USART::Event_flag::idle;
    }
    if (true == bit::flag::is(a_isr, ll::usart::ISR::tc))
    {
        pending_events |= USART::Event_flag::transfer_complete;
    }

    if (true == bit::flag::is(a_isr, ll::usart::ISR::wuf))
    {
        pending_events |= USART::Event_flag::wakeup_from_stop;
    }
    if (true == bit::flag::is(a_isr, ll::usart::ISR::cmf))
    {
        pending_events |= USART::Event_flag::character_matched;
    }

    if (true == bit::is_any(a_isr, ll::usart::ISR::pe | ll::usart::ISR::fe | ll::usart::ISR::ore | ll::usart::ISR::ne))
    {
        if (true == bit::flag::is(a_isr, ll::usart::ISR::pe))
        {
            pending_events |= USART::Event_flag::parity_error;
        }
        if (true == bit::flag::is(a_isr, ll::usart::ISR::fe))
        {
            pending_events |= USART::Event_flag::framing_error;
        }
        if (true == bit::flag::is(a_isr, ll::usart::ISR::ore))
        {
            pending_events |= USART::Event_flag::overrun;
        }
        if (true == bit::flag::is(a_isr, ll::usart::ISR::ne))
        {
            pending_events |= USART::Event_flag::noise_detected;
        }
    }

    return pending_events;
}

void clear_events(ll::usart::ICR* a_p_icr, USART::Event_flag a_event_mask)
{
    ll::usart::ICR::Flag clear_mask = ll::usart::ICR::none;

    if (USART::Event_flag::none != (a_event_mask & USART::Event_flag::idle))
    {
        clear_mask |= ll::usart::ICR::idlecf;
    }
    if (USART::Event_flag::none != (a_event_mask & USART::Event_flag::transfer_complete))
    {
        clear_mask |= ll::usart::ICR::tccf;
    }

    if (USART::Event_flag::none != (a_event_mask & USART::Event_flag::wakeup_from_stop))
    {
        clear_mask |= ll::usart::ICR::wucf;
    }
    if (USART::Event_flag::none != (a_event_mask & USART::Event_flag::character_matched))
    {
        clear_mask |= ll::usart::ICR::cmcf;
    }

    if (USART::Event_flag::none != (a_event_mask & (USART::Event_flag::parity_error | USART::Event_flag::framing_error |
                                                    USART::Event_flag::overrun | USART::Event_flag::noise_detected)))
    {
        if (USART::Event_flag::none != (a_event_mask & USART::Event_flag::parity_error))
        {
            clear_mask |= ll::usart::ICR::pecf;
        }
        if (USART::Event_flag::none != (a_event_mask & USART::Event_flag::framing_error))
        {
            clear_mask |= ll::usart::ICR::fecf;
        }
        if (USART::Event_flag::none != (a_event_mask & USART::Event_flag::overrun))
        {
            clear_mask |= ll::usart::ICR::orecf;
        }
        if (USART::Event_flag::none != (a_event_mask & USART::Event_flag::noise_detected))
        {
            clear_mask |= ll::usart::ICR::ncf;
        }
    }

    bit::flag::set(a_p_icr, clear_mask);
}

template<typename Periph_t> typename Periph_t::Frame_format::Word_length get_Word_length(std::uint32_t a_CR1_register)
{
    hkm_assert(false == bit::flag::is(a_CR1_register, ll::usart::CR1::m0) ||
               false == bit::flag::is(a_CR1_register, ll::usart::CR1::m1));

    if (false == bit::flag::is(a_CR1_register, ll::usart::CR1::m0) &&
        false == bit::flag::is(a_CR1_register, ll::usart::CR1::m1))
    {
        return USART::Frame_format::Word_length::_8_bit;
    }

    if (false == bit::flag::is(a_CR1_register, ll::usart::CR1::m0) &&
        true == bit::flag::is(a_CR1_register, ll::usart::CR1::m1))
    {
        return USART::Frame_format::Word_length::_7_bit;
    }

    return USART::Frame_format::Word_length::_9_bit;
}

void enable(ll::usart::Registers* a_p_registers,
            const LPUART::Clock_config& a_clock_config,
            const LPUART::Transceiving_config& a_transceiving_config,
            const LPUART::Frame_format& a_frame_format,
            LPUART::Low_power_wakeup_method a_low_power_wakeup)
{
    a_p_registers->cr1 = ll::usart::CR1::none;
    a_p_registers->cr2 = ll::usart::CR2::none;
    a_p_registers->cr3 = ll::usart::CR3::none;

#if defined(XMCU_ASSERT_ENABLED)
    constexpr std::uint32_t BRR_min = 0x300u;
    constexpr std::uint32_t BRR_max = 0xFFFFFu;

    std::uint32_t clk_freq =
        a_clock_config.freq_Hz / clock_prescaler_lut[static_cast<std::uint32_t>(a_clock_config.prescaler)];

    hkm_assert(clk_freq >= 3u * a_transceiving_config.baud_rate && clk_freq <= a_transceiving_config.baud_rate * 4096u);

    std::uint32_t brr = (clk_freq * 256u + (a_transceiving_config.baud_rate / 2)) / a_transceiving_config.baud_rate;
    hkm_assert(brr >= BRR_min && brr <= BRR_max);
    a_p_registers->brr = brr;
#endif

#if !defined(XMCU_ASSERT_ENABLED)
    a_p_registers->brr =
        ((a_clock_config.freq_Hz / clock_prescaler_lut[static_cast<std::uint32_t>(a_clock_config.prescaler)]) * 256u +
         (a_transceiving_config.baud_rate / 2u)) /
        a_transceiving_config.baud_rate;
#endif

    if (LPUART::Transceiving_config::Flow_control_flag::RS232 ==
        (a_transceiving_config.flow_control & LPUART::Transceiving_config::Flow_control_flag::RS232))
    {
        a_p_registers->cr3 = bit::flag::get(static_cast<ll::usart::CR3::Data>(a_transceiving_config.flow_control),
                                            ll::usart::CR3::rtse & ll::usart::CR3::ctse);
    }

    if (LPUART::Transceiving_config::Flow_control_flag::RS485 ==
        (a_transceiving_config.flow_control & LPUART::Transceiving_config::Flow_control_flag::RS485))
    {
        a_p_registers->cr3 = ll::usart::CR3::dem;
        bit::flag::set(&(a_p_registers->cr1),
                       bit::flag::get(static_cast<ll::usart::CR1::Flag>(a_transceiving_config.flow_control),
                                      (0x1F << ll::usart::CR1::deat) | (0x1F << ll::usart::CR1::dedt)));
    }

    if (LPUART::Transceiving_config::Mute_method::character_matched ==
        static_cast<LPUART::Transceiving_config::Mute_method>(
            static_cast<std::uint32_t>(a_transceiving_config.mute_method) & 0x200u))
    {
        bit::flag::set(&(a_p_registers->cr2),
                       ((bit::flag::get(static_cast<std::uint8_t>(a_transceiving_config.mute_method), 0xFFu))
                        << ll::usart::CR2::add) |
                           ll::usart::CR2::addm7);
        bit::flag::set(&(a_p_registers->cr1), ll::usart::CR1::mme | ll::usart::CR1::wake);
    }
    else if (LPUART::Transceiving_config::Mute_method::idle_line == a_transceiving_config.mute_method)
    {
        bit::flag::set(&(a_p_registers->cr1), ll::usart::CR1::mme);
    }

    if (USART::Low_power_wakeup_method::none != a_low_power_wakeup)
    {
        bit::flag::set(&(a_p_registers->cr1), ll::usart::CR1::uesm);

        bit::flag::clear(&(a_p_registers->cr1), ll::usart::CR1::ue);
        bit::flag::set(
            &(a_p_registers->cr3), 0x3u << ll::usart::CR3::wus, static_cast<ll::usart::CR3::Flag>(a_low_power_wakeup));
    }

    bit::flag::set(&(a_p_registers->cr2), static_cast<ll::usart::CR2::Flag>(a_transceiving_config.stop_bits));
    bit::flag::set(&(a_p_registers->cr1),
                   static_cast<ll::usart::CR1::Flag>(a_transceiving_config.mode) |
                       static_cast<ll::usart::CR1::Flag>(a_frame_format.parity) |
                       static_cast<ll::usart::CR1::Flag>(a_frame_format.word_length) | ll::usart::CR1::ue);
}

void enable(ll::usart::Registers* a_p_registers,
            const USART::Clock_config& a_clock_config,
            const USART::Transceiving_config& a_transceiving_config,
            const USART::Frame_format& a_frame_format,
            USART::Low_power_wakeup_method a_low_power_wakeup)
{
    constexpr std::uint32_t BRR_min = 0x10u;
    constexpr std::uint32_t BRR_max = 0xFFFFu;

    a_p_registers->cr1 = ll::usart::CR1::none;
    a_p_registers->cr2 = ll::usart::CR2::none;
    a_p_registers->cr3 = ll::usart::CR3::none;

    switch (a_transceiving_config.oversampling)
    {
        case USART::Transceiving_config::Oversampling::_16: {
            std::uint32_t brr =
                (a_clock_config.freq_Hz / clock_prescaler_lut[static_cast<std::uint32_t>(a_clock_config.prescaler)]) /
                a_transceiving_config.baud_rate;
            hkm_assert(BRR_min <= brr && brr <= BRR_max);
            a_p_registers->brr = brr;
            bit::flag::clear(&(a_p_registers->cr1), ll::usart::CR1::over8);
        }
        break;

        case USART::Transceiving_config::Oversampling::_8: {
            const std::uint32_t usartdiv =
                (2 *
                 (a_clock_config.freq_Hz / clock_prescaler_lut[static_cast<std::uint32_t>(a_clock_config.prescaler)])) /
                a_transceiving_config.baud_rate;
            std::uint32_t brr = ((usartdiv & 0xFFF0u) | ((usartdiv & 0xFu) >> 1)) & 0xFFFF;
            hkm_assert(BRR_min <= brr && brr <= BRR_max);
            a_p_registers->brr = brr;
            bit::flag::set(&(a_p_registers->cr1), ll::usart::CR1::over8);
        }
        break;
    }

    if (USART::Transceiving_config::Flow_control_flag::RS232 ==
        (a_transceiving_config.flow_control & USART::Transceiving_config::Flow_control_flag::RS232))
    {
        a_p_registers->cr3 = bit::flag::get(static_cast<ll::usart::CR3::Flag>(a_transceiving_config.flow_control),
                                            ll::usart::CR3::rtse | ll::usart::CR3::ctse);
    }

    if (USART::Transceiving_config::Flow_control_flag::RS485 ==
        (a_transceiving_config.flow_control & USART::Transceiving_config::Flow_control_flag::RS485))
    {
        a_p_registers->cr3 = ll::usart::CR3::dem;
        bit::flag::set(&(a_p_registers->cr1),
                       bit::flag::get(static_cast<ll::usart::CR1::Data>(a_transceiving_config.flow_control),
                                      (0x1F << ll::usart::CR1::deat) | (0x1F << ll::usart::CR1::dedt)));
    }

    if (USART::Transceiving_config::Mute_method::character_matched ==
        static_cast<USART::Transceiving_config::Mute_method>(
            static_cast<std::uint32_t>(a_transceiving_config.mute_method) & 0x200u))
    {
        const std::uint32_t address =
            bit::flag::get(static_cast<std::uint32_t>(a_transceiving_config.mute_method), 0xFFu);
        bit::flag::set(&(a_p_registers->cr2), (address << ll::usart::CR2::add) | ll::usart::CR2::addm7);
        bit::flag::set(&(a_p_registers->cr1), ll::usart::CR1::mme | ll::usart::CR1::wake);
    }
    else if (USART::Transceiving_config::Mute_method::idle_line == a_transceiving_config.mute_method)
    {
        bit::flag::set(&(a_p_registers->cr1), ll::usart::CR1::mme);
    }

    if (USART::Low_power_wakeup_method::none != a_low_power_wakeup)
    {
        bit::flag::set(&(a_p_registers->cr1), ll::usart::CR1::uesm);

        bit::flag::clear(&(a_p_registers->cr1), ll::usart::CR1::ue);
        bit::flag::set(
            &(a_p_registers->cr3), 0x3u << ll::usart::CR3::wus, static_cast<ll::usart::CR3::Flag>(a_low_power_wakeup));
    }

    bit::flag::set(&(a_p_registers->cr2), static_cast<ll::usart::CR2::Flag>(a_transceiving_config.stop_bits));
    bit::flag::set(&(a_p_registers->cr1),
                   static_cast<ll::usart::CR1::Flag>(a_transceiving_config.mode) |
                       static_cast<ll::usart::CR1::Flag>(a_frame_format.parity) |
                       static_cast<ll::usart::CR1::Flag>(a_frame_format.word_length) | ll::usart::CR1::ue);
}

// TODO: sfinae to limit data_t to u8/u16? Not really required because fn is internal
template<typename Data_t> USART::Polling::Result
transmit(ll::usart::Registers* a_p_registers, const Data_t* a_p_data, std::size_t a_data_size_in_words)
{
    hkm_assert(a_data_size_in_words > 0);

    // for proper result from is_tx_error
    bit::flag::set(&(a_p_registers->icr), ll::usart::ICR::tccf | ll::usart::ICR::ncf | ll::usart::ICR::pecf);

    std::size_t words = 0;
    USART::Event_flag events = USART::Event_flag::none;
    do
    {
        if (true == bit::flag::is(a_p_registers->isr, ll::usart::ISR::txe))
        {
            if constexpr (true == std::is_same_v<Data_t, uint16_t>)
            {
                a_p_registers->tdr = a_p_data[words++] & 0x1FFu;
            }
            else if constexpr (true == std::is_same_v<Data_t, uint8_t>)
            {
                a_p_registers->tdr = a_p_data[words++];
            }
            else
            {
                hkm_assert(false);
            }
        }
    } while (words < a_data_size_in_words && false == is_tx_error(a_p_registers));

    if (false == is_tx_error(a_p_registers))
    {
        events |= USART::Event_flag::transfer_complete;
        wait_until::all_bits_are_set(a_p_registers->isr, ll::usart::ISR::tc);
    }

    if (true == bit::is_any(a_p_registers->isr, ll::usart::ISR::pe | ll::usart::ISR::fe | ll::usart::ISR::ne))
    {
        if (true == bit::flag::is(a_p_registers->isr, ll::usart::ISR::pe))
        {
            bit::flag::set(&(a_p_registers->icr), ll::usart::ICR::pecf);
            events |= USART::Event_flag::parity_error;
        }
        if (true == bit::flag::is(a_p_registers->isr, ll::usart::ISR::ne))
        {
            bit::flag::set(&(a_p_registers->icr), ll::usart::ICR::ncf);
            events |= USART::Event_flag::noise_detected;
        }
    }

    return { events, words };
}

template<typename Data_t> USART::Polling::Result transmit(ll::usart::Registers* a_p_registers,
                                                          const Data_t* a_p_data,
                                                          std::size_t a_data_size_in_words,
                                                          Milliseconds a_timeout)
{
    hkm_assert(a_data_size_in_words > 0);
    hkm_assert(a_timeout > 0_ms);

    const std::uint64_t start = tick_counter<Milliseconds>::get();

    bit::flag::set(&(a_p_registers->icr), ll::usart::ICR::tccf);

    std::size_t words = 0;
    USART::Event_flag events = USART::Event_flag::none;
    do
    {
        if (true == bit::flag::is(a_p_registers->isr, ll::usart::ISR::txe))
        {
            a_p_registers->tdr = a_p_data[words++];
        }
    } while (words < a_data_size_in_words && false == is_tx_error(a_p_registers) &&
             a_timeout.get() > tick_counter<Milliseconds>::get() - start);

    if (false == is_tx_error(a_p_registers))
    {
        if (true == wait_until::all_bits_are_set(a_p_registers->isr,
                                                 ll::usart::ISR::tc,
                                                 a_timeout.get() - (tick_counter<Milliseconds>::get() - start)))
        {
            events |= USART::Event_flag::transfer_complete;
        }
    }

    if (true == bit::is_any(a_p_registers->isr, ll::usart::ISR::pe | ll::usart::ISR::fe | ll::usart::ISR::ne))
    {
        if (true == bit::flag::is(a_p_registers->isr, ll::usart::ISR::pe))
        {
            bit::flag::set(&(a_p_registers->icr), ll::usart::ICR::pecf);
            events |= USART::Event_flag::parity_error;
        }
        if (true == bit::flag::is(a_p_registers->isr, ll::usart::ISR::ne))
        {
            bit::flag::set(&(a_p_registers->icr), ll::usart::ICR::ncf);
            events |= USART::Event_flag::noise_detected;
        }
    }

    return { events, words };
}

template<typename t_Data>
USART::Polling::Result receive(ll::usart::Registers* a_p_registers, t_Data* a_p_data, std::size_t a_data_size_in_words)
{
    // hkm_assert(a_data_size_in_words > 0);

    bit::flag::set(&(a_p_registers->icr), ll::usart::ICR::idlecf);

    std::size_t words = 0;

    while (false ==
               bit::flag::is(a_p_registers->isr, ll::usart::ISR::idle) && /*false == is_rx_error(a_p_registers) &&*/
           words < a_data_size_in_words)
    {
        if (true == bit::flag::is(a_p_registers->isr, ll::usart::ISR::rxne))
        {
            a_p_data[words++] = static_cast<t_Data>(a_p_registers->rdr);
        }
    }

    if (true == bit::flag::is(a_p_registers->isr, ll::usart::ISR::cmf))
    {
        bit::flag::set(&(a_p_registers->icr), ll::usart::ICR::cmcf);
    }

    return { get_Event_flag_and_clear(&(a_p_registers->icr), a_p_registers->isr), words };
}

template<typename t_Data> USART::Polling::Result
receive(ll::usart::Registers* a_p_registers, t_Data* a_p_data, std::size_t a_data_size_in_words, Milliseconds a_timeout)
{
    // hkm_assert(a_data_size_in_words > 0);
    // hkm_assert(a_timeout > 0_ms);

    const std::uint64_t start = tick_counter<Milliseconds>::get();

    bit::flag::set(&(a_p_registers->icr), ll::usart::ICR::idlecf);

    std::size_t words = 0;

    while (false == bit::flag::is(a_p_registers->isr, ll::usart::ISR::idle) && false == is_rx_error(a_p_registers) &&
           words < a_data_size_in_words && a_timeout.get() > tick_counter<Milliseconds>::get() - start)
    {
        if (true == bit::flag::is(a_p_registers->isr, ll::usart::ISR::rxne))
        {
            a_p_data[words++] = static_cast<t_Data>(a_p_registers->rdr);
        }
    }

    if (true == bit::flag::is(a_p_registers->isr, ll::usart::ISR::cmf))
    {
        bit::flag::set(&(a_p_registers->icr), ll::usart::ICR::cmcf);
    }

    return { get_Event_flag_and_clear(&(a_p_registers->icr), a_p_registers->isr), words };
}

void transmit_start(ll::usart::Registers* a_p_registers)
{
    hkm_assert(true == bit::flag::is(a_p_registers->cr1, ll::usart::CR1::te));

    bit::flag::set(&(a_p_registers->icr), ll::usart::ICR::tccf);
    bit::flag::set(&(a_p_registers->cr1), ll::usart::CR1::txeie | ll::usart::CR1::tcie);
}

void receive_start(ll::usart::Registers* a_p_registers)
{
    hkm_assert(true == bit::flag::is(a_p_registers->cr1, ll::usart::CR1::re));

    bit::flag::set(&(a_p_registers->cr1), ll::usart::CR1::rxneie);
}

void event_listening_start(ll::usart::Registers* a_p_registers, USART::Event_flag events)
{
    clear_events(&a_p_registers->icr, events);
    if (USART::Event_flag::none != (events & USART::Event_flag::parity_error))
    {
        bit::flag::set(&(a_p_registers->cr1), ll::usart::CR1::peie);
    }
    if (USART::Event_flag::none != (events & USART::Event_flag::idle))
    {
        bit::flag::set(&(a_p_registers->cr1), ll::usart::CR1::idleie);
    }
    if (USART::Event_flag::none != (events & USART::Event_flag::transfer_complete))
    {
        bit::flag::set(&(a_p_registers->cr1), ll::usart::CR1::tcie);
    }
    if (USART::Event_flag::none != (events & USART::Event_flag::character_matched))
    {
        bit::flag::set(&(a_p_registers->cr1), ll::usart::CR1::cmie);
    }
    if (USART::Event_flag::none != (events & USART::Event_flag::parity_error))
    {
        bit::flag::set(&(a_p_registers->cr1), ll::usart::CR1::peie);
    }
    if (USART::Event_flag::none !=
        (events & (USART::Event_flag::framing_error | USART::Event_flag::overrun | USART::Event_flag::noise_detected)))
    {
        bit::flag::set(&(a_p_registers->cr3), ll::usart::CR3::eie);
    }
}

void USART_interrupt_handler(ll::usart::Registers* a_p_registers,
                             USART::Interrupt::Transmit_callback* a_p_transmit_callback,
                             USART::Interrupt::Receive_callback* a_p_receive_callback,
                             USART::Interrupt::Event_callback* a_p_event_callback)
{
    hkm_assert(nullptr != a_p_registers);
    hkm_assert(nullptr != a_p_transmit_callback);
    hkm_assert(nullptr != a_p_receive_callback);
    hkm_assert(nullptr != a_p_event_callback);

    const ll::usart::ISR::Data isr = a_p_registers->isr;

    if (nullptr != a_p_event_callback->function)
    {
        USART::Event_flag pending_events = get_pending_events(isr) & a_p_event_callback->events;
        if (USART::Event_flag::none != pending_events)
        {
            a_p_event_callback->function(pending_events, a_p_event_callback->p_user_data);
            clear_events(&a_p_registers->icr, pending_events);
        }
    }

    if (nullptr != a_p_transmit_callback->function && true == bit::flag::is(isr, ll::usart::ISR::txe))
    {
        a_p_transmit_callback->function(&(a_p_registers->tdr), a_p_transmit_callback->p_user_data);
    }

    if (true == bit::flag::is(isr, ll::usart::ISR::rxne))
    {
        const std::uint32_t rx_data = a_p_registers->rdr;
        if (nullptr != a_p_receive_callback->function)
        {
            a_p_receive_callback->function(rx_data, a_p_receive_callback->p_user_data);
        }
    }
    // This prevents retriggering interrupt when overrun is detected, even if CR3.EIE is
    // disabledhttps://dev.azure.com/heavykinematic/SOLOFirmware/_artifacts NOTE: This might have something to do with
    // non-multibuffer communication and might be also needed for NF/FE
    bit::flag::set(&a_p_registers->icr, ll::usart::ICR::orecf | ll::usart::ICR::fecf | ll::usart::ICR::ncf);
}

USART* USART_irq_context[1] = { nullptr };
LPUART* LPUART_irq_context[1] = { nullptr };
} // namespace

extern "C" {
void USART2_IRQHandler()
{
    hkm_assert(nullptr != USART_irq_context[0]);

    USART_interrupt_handler(USART_irq_context[0]);
}
void LPUART1_IRQHandler()
{
    hkm_assert(nullptr != LPUART_irq_context[0]);

    LPUART_interrupt_handler(LPUART_irq_context[0]);
}
}

namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals {
using namespace xmcu;
using namespace utils;

void USART_interrupt_handler(USART* a_p_this)
{
    hkm_assert(nullptr != a_p_this);

    ::USART_interrupt_handler(a_p_this->p_registers,
                              &(a_p_this->transmit_callback),
                              &(a_p_this->receive_callback),
                              &(a_p_this->event_callback));
}

void LPUART_interrupt_handler(LPUART* a_p_this)
{
    hkm_assert(nullptr != a_p_this);

    ::USART_interrupt_handler(a_p_this->p_registers,
                              &(a_p_this->transmit_callback),
                              &(a_p_this->receive_callback),
                              &(a_p_this->event_callback));
}

void USART::enable(const Clock_config& a_clock_config,
                   const Transceiving_config& a_transceiving_config,
                   const Frame_format& a_frame_format,
                   Low_power_wakeup_method a_low_power_wakeup)
{
    hkm_assert(0 != a_clock_config.freq_Hz);
    hkm_assert(various::get_enum_incorrect_value<Clock_config::Prescaler>() != a_clock_config.prescaler);

    hkm_assert(0 != a_transceiving_config.baud_rate);
    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Oversampling>() !=
               a_transceiving_config.oversampling);
    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Stop_bits>() != a_transceiving_config.stop_bits);

    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Flow_control_flag>() !=
               a_transceiving_config.flow_control);
    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Sampling_method>() !=
               a_transceiving_config.sampling_method);
    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Mode_flag>() != a_transceiving_config.mode);
    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Mute_method>() !=
               a_transceiving_config.mute_method);

    hkm_assert(various::get_enum_incorrect_value<Frame_format::Parity>() != a_frame_format.parity);
    hkm_assert(various::get_enum_incorrect_value<Frame_format::Word_length>() != a_frame_format.word_length);

    ::enable(this->p_registers, a_clock_config, a_transceiving_config, a_frame_format, a_low_power_wakeup);

    const ll::usart::ISR::Data flags_to_wait =
        (true == bit::flag::is(this->p_registers->cr1, ll::usart::CR1::re) ? ll::usart::ISR::reack :
                                                                             ll::usart::ISR::none) |
        (true == bit::flag::is(this->p_registers->cr1, ll::usart::CR1::te) ? ll::usart::ISR::teack :
                                                                             ll::usart::ISR::none);

    wait_until::all_bits_are_set(this->p_registers->isr, flags_to_wait);
}
bool USART::enable(const Clock_config& a_clock_config,
                   const Transceiving_config& a_transceiving_config,
                   const Frame_format& a_frame_format,
                   Low_power_wakeup_method a_low_power_wakeup,
                   Milliseconds a_timeout)
{
    hkm_assert(0 != a_clock_config.freq_Hz);
    hkm_assert(various::get_enum_incorrect_value<Clock_config::Prescaler>() != a_clock_config.prescaler);

    hkm_assert(0 != a_transceiving_config.baud_rate);
    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Oversampling>() !=
               a_transceiving_config.oversampling);
    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Stop_bits>() != a_transceiving_config.stop_bits);

    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Flow_control_flag>() !=
               a_transceiving_config.flow_control);
    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Sampling_method>() !=
               a_transceiving_config.sampling_method);
    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Mode_flag>() != a_transceiving_config.mode);
    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Mute_method>() !=
               a_transceiving_config.mute_method);

    hkm_assert(various::get_enum_incorrect_value<Frame_format::Parity>() != a_frame_format.parity);
    hkm_assert(various::get_enum_incorrect_value<Frame_format::Word_length>() != a_frame_format.word_length);

    hkm_assert(a_timeout > 0_ms);

    const std::uint64_t start = tick_counter<Milliseconds>::get();

    ::enable(this->p_registers, a_clock_config, a_transceiving_config, a_frame_format, a_low_power_wakeup);

    const ll::usart::ISR::Data flags_to_wait =
        (true == bit::flag::is(this->p_registers->cr1, ll::usart::CR1::re) ? ll::usart::ISR::reack :
                                                                             ll::usart::ISR::none) |
        (true == bit::flag::is(this->p_registers->cr1, ll::usart::CR1::te) ? ll::usart::ISR::teack :
                                                                             ll::usart::ISR::none);

    return wait_until::all_bits_are_set(
        this->p_registers->isr, flags_to_wait, a_timeout.get() - (tick_counter<Milliseconds>::get() - start));
}

void USART::disable()
{
    if (true == this->interrupt.is_enabled())
    {
        this->interrupt.disable();
    }

    this->p_registers->cr1 = ll::usart::CR1::none;
    this->p_registers->cr2 = ll::usart::CR2::none;
    this->p_registers->cr3 = ll::usart::CR3::none;
}

USART::Polling::Result USART::Polling::transmit(Not_null<const std::uint8_t*> a_p_data,
                                                std::size_t a_data_size_in_words)
{
    return ::transmit<std::uint8_t>(this->p_USART->p_registers, a_p_data, a_data_size_in_words);
}
USART::Polling::Result USART::Polling::transmit(Not_null<const std::uint16_t*> a_p_data,
                                                std::size_t a_data_size_in_words)
{
    return ::transmit<std::uint16_t>(this->p_USART->p_registers, a_p_data, a_data_size_in_words);
}

USART::Polling::Result USART::Polling::transmit(Not_null<const std::uint8_t*> a_p_data,
                                                std::size_t a_data_size_in_words,
                                                Milliseconds a_timeout)
{
    return ::transmit<std::uint8_t>(this->p_USART->p_registers, a_p_data, a_data_size_in_words, a_timeout);
}
USART::Polling::Result USART::Polling::transmit(Not_null<const std::uint16_t*> a_p_data,
                                                std::size_t a_data_size_in_words,
                                                Milliseconds a_timeout)
{
    return ::transmit<std::uint16_t>(this->p_USART->p_registers, a_p_data, a_data_size_in_words, a_timeout);
}

USART::Polling::Result USART::Polling::receive(Not_null<std::uint8_t*> a_p_data, std::size_t a_data_size_in_words)
{
    return ::receive<std::uint8_t>(this->p_USART->p_registers, a_p_data, a_data_size_in_words);
}
USART::Polling::Result USART::Polling::receive(Not_null<std::uint16_t*> a_p_data, std::size_t a_data_size_in_words)
{
    return ::receive<std::uint16_t>(this->p_USART->p_registers, a_p_data, a_data_size_in_words);
}

USART::Polling::Result
USART::Polling::receive(Not_null<std::uint8_t*> a_p_data, std::size_t a_data_size_in_words, Milliseconds a_timeout)
{
    return ::receive<std::uint8_t>(this->p_USART->p_registers, a_p_data, a_data_size_in_words, a_timeout);
}
USART::Polling::Result
USART::Polling::receive(Not_null<std::uint16_t*> a_p_data, std::size_t a_data_size_in_words, Milliseconds a_timeout)
{
    return ::receive<std::uint16_t>(this->p_USART->p_registers, a_p_data, a_data_size_in_words, a_timeout);
}

bool USART::Polling::is_transfer_complete()
{
    Event_flag pending_events = get_pending_events(this->p_USART->p_registers->isr);
    if (Event_flag::none != (pending_events & Event_flag::transfer_complete))
    {
        clear_events(&this->p_USART->p_registers->icr, Event_flag::transfer_complete);
        return true;
    }
    return false;
}

void USART::Interrupt::enable(const IRQ_config& a_config)
{
    hkm_assert(nullptr == USART_irq_context[this->p_USART->idx]);

    USART_irq_context[this->p_USART->idx] = this->p_USART;

    NVIC_SetPriority(this->p_USART->irqn,
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_config.preempt_priority, a_config.sub_priority));
    NVIC_EnableIRQ(this->p_USART->irqn);
}

void USART::Interrupt::disable()
{
    hkm_assert(nullptr != USART_irq_context[this->p_USART->idx]);

    this->transmit_stop();
    this->receive_stop();
    this->event_listening_stop();

    NVIC_DisableIRQ(this->p_USART->irqn);

    bit::flag::clear(&(this->p_USART->p_registers->cr1), ll::usart::CR1::uesm);
    bit::flag::clear(&(this->p_USART->p_registers->cr3), 0x3u << ll::usart::CR3::wus);

    USART_irq_context[this->p_USART->idx] = nullptr;
}

void USART::Interrupt::transmit_start(const Transmit_callback& a_callback)
{
    hkm_assert(nullptr != a_callback.function);

    Scoped_guard<nvic> guard;
    this->p_USART->transmit_callback = a_callback;

    ::transmit_start(this->p_USART->p_registers);
}

void USART::Interrupt::receive_start(const Receive_callback& a_callback)
{
    hkm_assert(nullptr != a_callback.function);

    Scoped_guard<nvic> guard;
    this->p_USART->receive_callback = a_callback;

    ::receive_start(this->p_USART->p_registers);
}

void USART::Interrupt::event_listening_start(const Event_callback& a_callback)
{
    hkm_assert(nullptr != a_callback.function);

    Scoped_guard<nvic> guard;

    this->p_USART->event_callback = a_callback;

    ::event_listening_start(this->p_USART->p_registers, a_callback.events);
}

void USART::Interrupt::transmit_stop()
{
    Scoped_guard<nvic> guard;

    bit::flag::clear(&(this->p_USART->p_registers->cr1), ll::usart::CR1::txeie | ll::usart::CR1::tcie);

    this->p_USART->transmit_callback = { .function = nullptr, .p_user_data = nullptr };
}

void USART::Interrupt::receive_stop()
{
    Scoped_guard<nvic> guard;

    bit::flag::clear(&(this->p_USART->p_registers->cr1), ll::usart::CR1::rxneie | ll::usart::CR1::idleie);

    this->p_USART->receive_callback = { .function = nullptr, .p_user_data = nullptr };
}

void USART::Interrupt::event_listening_stop()
{
    Scoped_guard<nvic> guard;

    bit::flag::clear(&(this->p_USART->p_registers->cr1), ll::usart::CR1::peie | ll::usart::CR1::idleie);
    bit::flag::clear(&(this->p_USART->p_registers->cr3), ll::usart::CR3::eie);

    this->p_USART->event_callback = { .function = nullptr, .p_user_data = nullptr };
}

void LPUART::enable(const Clock_config& a_clock_config,
                    const Transceiving_config& a_transceiving_config,
                    const Frame_format& a_frame_format,
                    Low_power_wakeup_method a_low_power_wakeup)
{
    hkm_assert(0 != a_clock_config.freq_Hz);
    hkm_assert(various::get_enum_incorrect_value<Clock_config::Prescaler>() != a_clock_config.prescaler);

    hkm_assert(0 != a_transceiving_config.baud_rate);
    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Stop_bits>() != a_transceiving_config.stop_bits);

    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Flow_control_flag>() !=
               a_transceiving_config.flow_control);
    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Sampling_method>() !=
               a_transceiving_config.sampling_method);
    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Mode_flag>() != a_transceiving_config.mode);
    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Mute_method>() !=
               a_transceiving_config.mute_method);

    hkm_assert(various::get_enum_incorrect_value<USART::Frame_format::Parity>() != a_frame_format.parity);
    hkm_assert(various::get_enum_incorrect_value<USART::Frame_format::Word_length>() != a_frame_format.word_length);

    hkm_assert(((a_clock_config.freq_Hz / clock_prescaler_lut[static_cast<std::uint32_t>(a_clock_config.prescaler)]) >=
                (3u * a_transceiving_config.baud_rate)) ||
               ((a_clock_config.freq_Hz / clock_prescaler_lut[static_cast<std::uint32_t>(a_clock_config.prescaler)]) <=
                (4096u * a_transceiving_config.baud_rate)));

    ::enable(this->p_registers, a_clock_config, a_transceiving_config, a_frame_format, a_low_power_wakeup);

    const ll::usart::ISR::Data flags_to_wait =
        (true == bit::flag::is(this->p_registers->cr1, ll::usart::CR1::re) ? ll::usart::ISR::reack :
                                                                             ll::usart::ISR::none) |
        (true == bit::flag::is(this->p_registers->cr1, ll::usart::CR1::te) ? ll::usart::ISR::teack :
                                                                             ll::usart::ISR::none);

    wait_until::all_bits_are_set(this->p_registers->isr, flags_to_wait);
}

bool LPUART::enable(const Clock_config& a_clock_config,
                    const Transceiving_config& a_transceiving_config,
                    const Frame_format& a_frame_format,
                    Low_power_wakeup_method a_low_power_wakeup,
                    Milliseconds a_timeout)
{
    hkm_assert(0 != a_clock_config.freq_Hz);
    hkm_assert(various::get_enum_incorrect_value<Clock_config::Prescaler>() != a_clock_config.prescaler);

    hkm_assert(0 != a_transceiving_config.baud_rate);
    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Stop_bits>() != a_transceiving_config.stop_bits);

    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Flow_control_flag>() !=
               a_transceiving_config.flow_control);
    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Sampling_method>() !=
               a_transceiving_config.sampling_method);
    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Mode_flag>() != a_transceiving_config.mode);
    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Mute_method>() !=
               a_transceiving_config.mute_method);

    hkm_assert(various::get_enum_incorrect_value<USART::Frame_format::Parity>() != a_frame_format.parity);
    hkm_assert(various::get_enum_incorrect_value<USART::Frame_format::Word_length>() != a_frame_format.word_length);

    hkm_assert(a_timeout > 0_ms);

    hkm_assert(((a_clock_config.freq_Hz / clock_prescaler_lut[static_cast<std::uint32_t>(a_clock_config.prescaler)]) >=
                (3u * a_transceiving_config.baud_rate)) ||
               ((a_clock_config.freq_Hz / clock_prescaler_lut[static_cast<std::uint32_t>(a_clock_config.prescaler)]) <=
                (4096u * a_transceiving_config.baud_rate)));

    const std::uint64_t start = tick_counter<Milliseconds>::get();

    ::enable(this->p_registers, a_clock_config, a_transceiving_config, a_frame_format, a_low_power_wakeup);

    const ll::usart::ISR::Data flags_to_wait =
        (true == bit::flag::is(this->p_registers->cr1, ll::usart::CR1::re) ? ll::usart::ISR::reack :
                                                                             ll::usart::ISR::none) |
        (true == bit::flag::is(this->p_registers->cr1, ll::usart::CR1::te) ? ll::usart::ISR::teack :
                                                                             ll::usart::ISR::none);

    return wait_until::all_bits_are_set(
        this->p_registers->isr, flags_to_wait, a_timeout.get() - (tick_counter<Milliseconds>::get() - start));
}

void LPUART::disable()
{
    if (true == this->interrupt.is_enabled())
    {
        this->interrupt.disable();
    }

    this->p_registers->cr1 = ll::usart::CR1::none;
    this->p_registers->cr2 = ll::usart::CR2::none;
    this->p_registers->cr3 = ll::usart::CR3::none;
}

LPUART::Polling::Result LPUART::Polling::transmit(Not_null<const std::uint8_t*> a_p_data,
                                                  std::size_t a_data_size_in_words)
{
    return ::transmit<std::uint8_t>(this->p_LPUART->p_registers, a_p_data, a_data_size_in_words);
}
LPUART::Polling::Result LPUART::Polling::transmit(Not_null<const std::uint16_t*> a_p_data,
                                                  std::size_t a_data_size_in_words)
{
    return ::transmit<std::uint16_t>(this->p_LPUART->p_registers, a_p_data, a_data_size_in_words);
}

LPUART::Polling::Result LPUART::Polling::transmit(Not_null<const std::uint8_t*> a_p_data,
                                                  std::size_t a_data_size_in_words,
                                                  Milliseconds a_timeout)
{
    return ::transmit<std::uint8_t>(this->p_LPUART->p_registers, a_p_data, a_data_size_in_words, a_timeout);
}
LPUART::Polling::Result LPUART::Polling::transmit(Not_null<const std::uint16_t*> a_p_data,
                                                  std::size_t a_data_size_in_words,
                                                  Milliseconds a_timeout)
{
    return ::transmit<std::uint16_t>(this->p_LPUART->p_registers, a_p_data, a_data_size_in_words, a_timeout);
}

LPUART::Polling::Result LPUART::Polling::receive(Not_null<std::uint8_t*> a_p_data, std::size_t a_data_size_in_words)
{
    return ::receive<std::uint8_t>(this->p_LPUART->p_registers, a_p_data, a_data_size_in_words);
}
LPUART::Polling::Result LPUART::Polling::receive(Not_null<std::uint16_t*> a_p_data, std::size_t a_data_size_in_words)
{
    return ::receive<std::uint16_t>(this->p_LPUART->p_registers, a_p_data, a_data_size_in_words);
}

LPUART::Polling::Result
LPUART::Polling::receive(Not_null<std::uint8_t*> a_p_data, std::size_t a_data_size_in_words, Milliseconds a_timeout)
{
    return ::receive<std::uint8_t>(this->p_LPUART->p_registers, a_p_data, a_data_size_in_words, a_timeout);
}
LPUART::Polling::Result
LPUART::Polling::receive(Not_null<std::uint16_t*> a_p_data, std::size_t a_data_size_in_words, Milliseconds a_timeout)
{
    return ::receive<std::uint16_t>(this->p_LPUART->p_registers, a_p_data, a_data_size_in_words, a_timeout);
}

void LPUART::Interrupt::enable(const IRQ_config& a_config)
{
    hkm_assert(nullptr == LPUART_irq_context[this->p_LPUART->idx]);

    LPUART_irq_context[this->p_LPUART->idx] = this->p_LPUART;

    NVIC_SetPriority(this->p_LPUART->irqn,
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_config.preempt_priority, a_config.sub_priority));
    NVIC_EnableIRQ(this->p_LPUART->irqn);
}

void LPUART::Interrupt::disable()
{
    hkm_assert(nullptr != LPUART_irq_context[this->p_LPUART->idx]);

    this->transmit_stop();
    this->receive_stop();
    this->event_listening_stop();

    NVIC_DisableIRQ(this->p_LPUART->irqn);

    LPUART_irq_context[this->p_LPUART->idx] = nullptr;
}

void LPUART::Interrupt::transmit_start(const Transmit_callback& a_callback)
{
    hkm_assert(nullptr != a_callback.function);

    Scoped_guard<nvic> guard;
    this->p_LPUART->transmit_callback = a_callback;

    ::transmit_start(this->p_LPUART->p_registers);
}

void LPUART::Interrupt::receive_start(const Receive_callback& a_callback)
{
    hkm_assert(nullptr != a_callback.function);

    Scoped_guard<nvic> guard;
    this->p_LPUART->receive_callback = a_callback;

    ::receive_start(this->p_LPUART->p_registers);
}

void LPUART::Interrupt::event_listening_start(const Event_callback& a_callback)
{
    hkm_assert(nullptr != a_callback.function);

    Scoped_guard<nvic> guard;

    this->p_LPUART->event_callback = a_callback;

    ::event_listening_start(this->p_LPUART->p_registers, a_callback.events);
}

void LPUART::Interrupt::transmit_stop()
{
    Scoped_guard<nvic> guard;

    bit::flag::clear(&(static_cast<ll::usart::Registers*>(*(this->p_LPUART))->cr1),
                     ll::usart::CR1::txeie | ll::usart::CR1::tcie);

    this->p_LPUART->transmit_callback = { .function = nullptr, .p_user_data = nullptr };
}

void LPUART::Interrupt::receive_stop()
{
    Scoped_guard<nvic> guard;

    bit::flag::clear(&(static_cast<ll::usart::Registers*>(*(this->p_LPUART))->cr1),
                     ll::usart::CR1::rxneie | ll::usart::CR1::idleie);

    this->p_LPUART->receive_callback = { .function = nullptr, .p_user_data = nullptr };
}

void LPUART::Interrupt::event_listening_stop()
{
    Scoped_guard<nvic> guard;

    bit::flag::clear(&(this->p_LPUART->p_registers->cr1), ll::usart::CR1::peie | ll::usart::CR1::idleie);
    bit::flag::clear(&(this->p_LPUART->p_registers->cr3), ll::usart::CR3::eie);

    this->p_LPUART->event_callback = { .events = USART::Event_flag::none, .function = nullptr, .p_user_data = nullptr };
}
} // namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals

namespace xmcu::soc::st::arm::m0::l0::rm0451 {
using namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals;
using namespace xmcu::soc::st::arm::m0::l0::rm0451::clocks;
using namespace xmcu::soc::st::arm::m0::l0::rm0451::clocks::sources;
using namespace xmcu::soc::st::arm::m0::l0::rm0451::system;

#if defined(XMCU_USART2_PRESENT)
template<> void rcc<USART, USART::_2>::enable<pclk<2u>>(bool a_enable_in_lp)
{
    bit::flag::clear(&(RCC->CCIPR), RCC_CCIPR_USART2SEL);
    bit::flag::set(&(RCC->APB1ENR), RCC_APB1ENR_USART2EN);

    if (true == a_enable_in_lp)
    {
        bit::flag::set(&(RCC->APB1SMENR), RCC_APB1SMENR_USART2SMEN);
    }
    else
    {
        bit::flag::clear(&(RCC->APB1SMENR), RCC_APB1SMENR_USART2SMEN);
    }
}
template<> void rcc<USART, USART::_2>::enable<rcc<mcu<1u>>>(bool a_enable_in_lp)
{
    bit::flag::set(&(RCC->CCIPR), RCC_CCIPR_USART2SEL, RCC_CCIPR_USART2SEL_0);
    bit::flag::set(&(RCC->APB1ENR), RCC_APB1ENR_USART2EN);

    if (true == a_enable_in_lp)
    {
        bit::flag::set(&(RCC->APB1SMENR), RCC_APB1SMENR_USART2SMEN);
    }
    else
    {
        bit::flag::clear(&(RCC->APB1SMENR), RCC_APB1SMENR_USART2SMEN);
    }
}
template<> void rcc<USART, USART::_2>::enable<hsi16>(bool a_enable_in_lp)
{
    bit::flag::set(&(RCC->CCIPR), RCC_CCIPR_USART2SEL, RCC_CCIPR_USART2SEL_1);
    bit::flag::set(&(RCC->APB1ENR), RCC_APB1ENR_USART2EN);

    if (true == a_enable_in_lp)
    {
        bit::flag::set(&(RCC->APB1SMENR), RCC_APB1SMENR_USART2SMEN);
    }
    else
    {
        bit::flag::clear(&(RCC->APB1SMENR), RCC_APB1SMENR_USART2SMEN);
    }
}

void rcc<USART, USART::_2>::disable()
{
    bit::flag::clear(&(RCC->CCIPR), RCC_CCIPR_USART2SEL);
    bit::flag::clear(&(RCC->APB1ENR), RCC_APB1ENR_USART2EN);
}
#endif

#if defined(XMCU_LPUART1_PRESENT)
template<> void rcc<LPUART, LPUART::_1>::enable<pclk<1u>>(bool a_enable_in_lp)
{
    bit::flag::clear(&(RCC->CCIPR), RCC_CCIPR_LPUART1SEL);
    bit::flag::set(&(RCC->APB1ENR), RCC_APB1ENR_LPUART1EN);

    if (true == a_enable_in_lp)
    {
        bit::flag::set(&(RCC->APB1SMENR), RCC_APB1SMENR_LPUART1SMEN);
    }
    else
    {
        bit::flag::clear(&(RCC->APB1SMENR), RCC_APB1SMENR_LPUART1SMEN);
    }
}
template<> void rcc<LPUART, LPUART::_1>::enable<rcc<mcu<1u>>>(bool a_enable_in_lp)
{
    bit::flag::set(&(RCC->CCIPR), RCC_CCIPR_LPUART1SEL, RCC_CCIPR_LPUART1SEL_0);
    bit::flag::set(&(RCC->APB1ENR), RCC_APB1ENR_LPUART1EN);

    if (true == a_enable_in_lp)
    {
        bit::flag::set(&(RCC->APB1SMENR), RCC_APB1SMENR_LPUART1SMEN);
    }
    else
    {
        bit::flag::clear(&(RCC->APB1SMENR), RCC_APB1SMENR_LPUART1SMEN);
    }
}
template<> void rcc<LPUART, LPUART::_1>::enable<hsi16>(bool a_enable_in_lp)
{
    bit::flag::set(&(RCC->CCIPR), RCC_CCIPR_LPUART1SEL, RCC_CCIPR_LPUART1SEL_1);
    bit::flag::set(&(RCC->APB1ENR), RCC_APB1ENR_LPUART1EN);

    if (true == a_enable_in_lp)
    {
        bit::flag::set(&(RCC->APB1SMENR), RCC_APB1SMENR_LPUART1SMEN);
    }
    else
    {
        bit::flag::clear(&(RCC->APB1SMENR), RCC_APB1SMENR_LPUART1SMEN);
    }
}

void rcc<peripherals::LPUART, LPUART::_1>::disable()
{
    bit::flag::clear(&(RCC->CCIPR), RCC_CCIPR_LPUART1SEL);
    bit::flag::clear(&(RCC->APB1ENR), RCC_APB1ENR_LPUART1EN);
}
#endif
} // namespace xmcu::soc::st::arm::m0::l0::rm0451
