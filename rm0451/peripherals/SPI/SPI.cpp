/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// this
#include <rm0451/peripherals/SPI/SPI.hpp>

// xmcu
#include <rm0451/utils/wait_until.hpp>
#include <xmcu/bit.hpp>

namespace {
using namespace xmcu;
using namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals;
using namespace xmcu::soc::st::arm::m0::l0::rm0451::utils;

void enable(SPI_TypeDef* a_p_registers)
{
    a_p_registers->CR1 = 0;
    a_p_registers->CR2 = 0;

    bit::flag::set(&a_p_registers->CR1,
                   SPI_CR1_BR | SPI_CR1_CPOL | SPI_CR1_CPHA,
                   SPI_CR1_BR_0 | SPI_CR1_BR_1);                               // f_pclk/2, SPI mode 0
    bit::flag::set(&a_p_registers->CR1, SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE, 0); // full duplex
    bit::flag::set(&a_p_registers->CR1, SPI_CR1_MSTR);                         // SPI Master mode
    bit::flag::set(&a_p_registers->CR1, SPI_CR1_LSBFIRST | SPI_CR1_DFF, 0);    // 8-bit transfers, MSBit first

    bit::flag::set(&a_p_registers->CR2, SPI_CR2_SSOE, 0);           // hw CS signal disabled - multi-CS or other pin
    bit::flag::set(&a_p_registers->CR1, SPI_CR1_SSM | SPI_CR1_SSI); // Set to 1 when not using hardware CS to avoid MODF

    bit::flag::set(&a_p_registers->CR1, SPI_CR1_SPE);
}

void disable(SPI_TypeDef* a_p_registers)
{
    a_p_registers->CR1 = 0;
    a_p_registers->CR2 = 0;
}

template<typename data_t>
SPI::Polling::Result transmit(SPI_TypeDef* a_p_registers, const data_t* a_p_data, std::size_t a_data_size_in_words)
{
    std::uint16_t words_sent = 0;
    while (words_sent < a_data_size_in_words)
    {
        a_p_registers->DR = *(a_p_data++);

        wait_until::all_bits_are_set(a_p_registers->SR, SPI_SR_TXE);
        ++words_sent;
    }
    wait_until::all_bits_are_cleared(a_p_registers->SR, SPI_SR_BSY);

    return SPI::Polling::Result { .event = SPI::Event_flag::none, .data_length_in_words = words_sent };
}

template<typename data_t>
SPI::Polling::Result receive(SPI_TypeDef* a_p_registers, data_t* a_p_data, std::size_t a_data_size_in_words)
{
    // dummy read of already stored stuff in DR
    *a_p_data = a_p_registers->DR;

    std::uint16_t words_received = 0;
    while (words_received < a_data_size_in_words)
    {
        a_p_registers->DR = 0; // fill MOSI with anything to receive via MISO
        wait_until::all_bits_are_set(a_p_registers->SR, SPI_SR_RXNE);
        *(a_p_data++) = a_p_registers->DR;
        ++words_received;
    }
    wait_until::all_bits_are_cleared(a_p_registers->SR, SPI_SR_BSY);

    return SPI::Polling::Result { .event = SPI::Event_flag::none, .data_length_in_words = words_received };
}
} // namespace

namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals {
void SPI::enable()
{
    ::enable(this->p_registers);
}

void SPI::disable()
{
    ::disable(this->p_registers);
}

SPI::Polling::Result SPI::Polling::transmit(Not_null<const std::uint8_t*> a_p_data, std::size_t a_data_size_in_words)
{
    return ::transmit<std::uint8_t>(this->p_SPI->p_registers, a_p_data, a_data_size_in_words);
}

SPI::Polling::Result SPI::Polling::receive(Not_null<std::uint8_t*> a_p_data, std::size_t a_data_size_in_words)
{
    return ::receive<std::uint8_t>(this->p_SPI->p_registers, a_p_data, a_data_size_in_words);
}
} // namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals

namespace xmcu::soc::st::arm::m0::l0::rm0451 {
using namespace xmcu;
using namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals;
using namespace xmcu::soc::st::arm::m0::l0::rm0451::clocks::sources;

#if defined(XMCU_SPI1_PRESENT)
template<> void rcc<SPI, SPI::_1>::enable<hsi16>(bool a_enable_in_lp)
{
    bit::flag::set(&(RCC->APB2ENR), RCC_APB2ENR_SPI1EN);

    if (true == a_enable_in_lp)
    {
        bit::flag::set(&(RCC->APB2SMENR), RCC_APB2SMENR_SPI1SMEN);
    }
    else
    {
        bit::flag::clear(&(RCC->APB2SMENR), RCC_APB2SMENR_SPI1SMEN);
    }
}
void rcc<peripherals::SPI, SPI::_1>::disable()
{
    bit::flag::clear(&(RCC->APB2ENR), RCC_APB2ENR_SPI1EN);
}
#endif
} // namespace xmcu::soc::st::arm::m0::l0::rm0451