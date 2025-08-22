#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// std
#include <cstddef>

// external
#include <stm32l0xx.h>

// xmcu
#include <rm0451/clocks/pclk.hpp>
#include <rm0451/clocks/sources/hsi16.hpp>
#include <rm0451/peripherals/GPIO/GPIO.hpp>
#include <rm0451/peripherals/USART/usart_ll.hpp>
#include <rm0451/rcc.hpp>
#include <rm0451/system/mcu/mcu.hpp>
#include <rm0451/utils/tick_counter.hpp>
#include <rm0451/utils/wait_until.hpp>
#include <soc/peripheral.hpp>
#include <soc/st/arm/IRQ_config.hpp>
#include <xmcu/Duration.hpp>
#include <xmcu/Non_copyable.hpp>
#include <xmcu/Not_null.hpp>
#include <xmcu/bit.hpp>
#include <xmcu/various.hpp>

namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals {
class USART : private Non_copyable
{
public:
    enum class Event_flag : std::uint32_t
    {
        none = 0x0u,
        framing_error = 0x1u,
        parity_error = 0x2u,
        overrun = 0x4u,
        noise_detected = 0x8u,
        idle = 0x10u,
        transfer_complete = 0x20u,
        character_matched = 0x40u,
        wakeup_from_stop = 0x80u
    };
    enum class Low_power_wakeup_method : std::uint32_t
    {
        none = 0x1u,
        character_matched = 0x0u,
        start_bit = static_cast<std::uint32_t>(0x2u << ll::usart::CR3::wus),
        rx_not_empty = static_cast<std::uint32_t>((0x1u << ll::usart::CR3::wus) | (0x2u << ll::usart::CR3::wus))
    };

    struct Clock_config
    {
        enum class Prescaler : std::uint32_t
        {
            _1 = 0x0,
        };

        std::uint32_t freq_Hz = 0;
        Prescaler prescaler = various::get_enum_incorrect_value<Prescaler>();
    };
    struct Transceiving_config
    {
        enum class Oversampling : std::uint32_t
        {
            _8,
            _16,
        };
        enum class Stop_bits : std::uint32_t
        {
            _0_5 = static_cast<std::uint32_t>(0x1u << ll::usart::CR2::stop),
            _1 = 0x0u,
            _1_5 = static_cast<std::uint32_t>((0x1u << ll::usart::CR2::stop) | (0x2u << ll::usart::CR2::stop)),
            _2 = static_cast<std::uint32_t>(0x2u << ll::usart::CR2::stop),
        };
        enum class Flow_control_flag : std::uint32_t
        {
            none = 0x0u,
            RS232 = 0x1u,
            RS485 = 0x2u,
        };
        enum Sampling_method : std::uint32_t
        {
            three_sample_bit = 0,
            one_sample_bit = static_cast<std::uint32_t>(ll::usart::CR3::onebit),
        };
        enum class Mode_flag : std::uint32_t
        {
            tx = static_cast<std::uint32_t>(ll::usart::CR1::te),
            rx = static_cast<std::uint32_t>(ll::usart::CR1::re)
        };
        enum class Mute_method : std::uint32_t
        {
            none = 0x0u,
            idle_line = 0x100u,
            character_matched = 0x200u,
        };
        enum class RS232_flow_control_flag : std::uint32_t
        {
            request_to_send = static_cast<std::uint32_t>(ll::usart::CR3::rtse),
            clear_to_send = static_cast<std::uint32_t>(ll::usart::CR3::rtse),
        };

        struct RS485_flow_control_config
        {
            std::uint8_t assertion_time = 0x0u;
            std::uint8_t deassertion_time = 0x0u;
        };

        std::uint32_t baud_rate = 0;
        Oversampling oversampling = various::get_enum_incorrect_value<Oversampling>();
        Stop_bits stop_bits = various::get_enum_incorrect_value<Stop_bits>();
        Flow_control_flag flow_control = various::get_enum_incorrect_value<Flow_control_flag>();
        Sampling_method sampling_method = various::get_enum_incorrect_value<Sampling_method>();
        Mode_flag mode = various::get_enum_incorrect_value<Mode_flag>();
        Mute_method mute_method = various::get_enum_incorrect_value<Mute_method>();
    };
    struct Frame_format
    {
        enum class Word_length : std::uint32_t
        {
            _7_bit = static_cast<std::uint32_t>(ll::usart::CR1::m1),
            _8_bit = 0x0u,
            _9_bit = static_cast<std::uint32_t>(ll::usart::CR1::m0),
        };
        enum class Parity : std::uint32_t
        {
            none = 0x0u,
            even = static_cast<std::uint32_t>(ll::usart::CR1::pce),
            odd = static_cast<std::uint32_t>(ll::usart::CR1::pce | ll::usart::CR1::ps)
        };

        Word_length word_length = various::get_enum_incorrect_value<Word_length>();
        Parity parity = various::get_enum_incorrect_value<Parity>();
    };

    class Polling : private Non_copyable
    {
    public:
        struct Result
        {
            Event_flag event = various::get_enum_incorrect_value<Event_flag>();
            std::size_t data_length_in_words = 0;
        };

        Result transmit(Not_null<const std::uint8_t*> a_p_data, std::size_t a_data_size_in_words);
        Result transmit(Not_null<const std::uint16_t*> a_p_data, std::size_t a_data_size_in_words);
        Result
        transmit(Not_null<const std::uint8_t*> a_p_data, std::size_t a_data_size_in_words, Milliseconds a_timeout);
        Result transmit(Not_null<const uint16_t*> a_p_data, std::size_t a_data_size_in_words, Milliseconds a_timeout);

        Result receive(Not_null<std::uint8_t*> a_p_data, std::size_t a_data_size_in_words);
        Result receive(Not_null<std::uint16_t*> a_p_data, std::size_t a_data_size_in_words);
        Result receive(Not_null<std::uint8_t*> a_p_data, std::size_t a_data_size_in_words, Milliseconds a_timeout);
        Result receive(Not_null<std::uint16_t*> a_p_data, std::size_t a_data_size_in_words, Milliseconds a_timeout);

        template<typename t_Type> std::uint32_t transmit_2(const t_Type& a_data)
        {
            const auto itr_begin = std::begin(a_data);
            const auto itr_end = std::end(a_data);
            auto itr = itr_begin;

            bit::flag::set(&(this->p_USART->p_registers->icr),
                           ll::usart::ICR::tccf | ll::usart::ICR::pecf | ll::usart::ICR::ncf);

            while (itr != itr_end &&
                   false == bit::is_any(this->p_USART->p_registers->isr, ll::usart::ISR::pe | ll::usart::ISR::ne))
            {
                if (true == bit::flag::is(this->p_USART->p_registers->isr, ll::usart::ISR::txe))
                {
                    this->p_USART->p_registers->tdr = *itr;
                    itr++;
                }
            }

            if (false == bit::is_any(this->p_USART->p_registers->isr, ll::usart::ISR::pe | ll::usart::ISR::ne))
            {
                utils::wait_until::all_bits_are_set(this->p_USART->p_registers->isr, ll::usart::ISR::tc);
            }

            return static_cast<std::uint32_t>(itr - itr_begin);
        }
        template<typename t_First, typename... t_Tail>
        std::uint32_t transmit_2(const t_First& a_first, const t_Tail&... a_tail)
        {
            const auto itr_begin = std::begin(a_first);
            const auto itr_end = std::end(a_first);
            auto itr = itr_begin;

            bit::flag::set(&(this->p_USART->p_registers->icr),
                           ll::usart::ICR::tccf | ll::usart::ICR::pecf | ll::usart::ICR::ncf);

            while (itr != itr_end &&
                   false == bit::is_any(this->p_USART->p_registers->isr, ll::usart::ISR::pe | ll::usart::ISR::ne))
            {
                if (true == bit::flag::is(this->p_USART->p_registers->isr, ll::usart::ISR::txe))
                {
                    this->p_USART->p_registers->tdr = *itr;
                    itr++;
                }
            }

            if (false == bit::is_any(this->p_USART->p_registers->isr, ll::usart::ISR::pe | ll::usart::ISR::ne))
            {
                return this->transmit_2(a_tail...) + static_cast<std::uint32_t>(itr - itr_begin);
            }

            return static_cast<std::uint32_t>(itr - itr_begin);
        }

        template<typename t_Type> std::uint32_t transmit_2(Milliseconds a_timeout, const t_Type& a_data)
        {
            const std::uint64_t timeout_end_timestamp = utils::tick_counter<Milliseconds>::get() + a_timeout.get();

            const auto itr_begin = std::begin(a_data);
            const auto itr_end = std::end(a_data);
            auto itr = itr_begin;

            bit::flag::set(&(this->p_USART->p_registers->icr),
                           ll::usart::ICR::tccf | ll::usart::ICR::pecf | ll::usart::ICR::ncf);

            while (itr != itr_end &&
                   false == bit::is_any(this->p_USART->p_registers->isr, ll::usart::ISR::pe | ll::usart::ISR::ne) &&
                   utils::tick_counter<Milliseconds>::get() <= timeout_end_timestamp)
            {
                if (true == bit::flag::is(this->p_USART->p_registers->isr, ll::usart::ISR::txe))
                {
                    this->p_USART->p_registers->tdr = *itr;
                    itr++;
                }
            }
            if (false == bit::is_any(this->p_USART->p_registers->isr, ll::usart::ISR::pe | ll::usart::ISR::ne))
            {
                utils::wait_until::all_bits_are_set(this->p_USART->p_registers->isr, ll::usart::ISR::tc);
            }

            return static_cast<std::uint32_t>(itr - itr_begin);
        }
        template<typename t_First, typename... t_Tail>
        std::uint32_t transmit_2(Milliseconds a_timeout, const t_First& a_first, const t_Tail&... a_tail)
        {
            const std::uint64_t timeout_start_timestamp = utils::tick_counter<Milliseconds>::get();
            const std::uint64_t timeout_end_timestamp = timeout_start_timestamp + a_timeout.get();

            const auto itr_begin = std::begin(a_first);
            const auto itr_end = std::end(a_first);
            auto itr = itr_begin;

            bit::flag::set(&(this->p_USART->p_registers->icr),
                           ll::usart::ICR::tccf | ll::usart::ICR::pecf | ll::usart::ICR::ncf);

            while (itr != itr_end &&
                   false == bit::is_any(this->p_USART->p_registers->isr, ll::usart::ISR::pe | ll::usart::ISR::ne) &&
                   utils::tick_counter<Milliseconds>::get() <= timeout_end_timestamp)
            {
                if (true == bit::flag::is(this->p_USART->p_registers->isr, ll::usart::ISR::txe))
                {
                    this->p_USART->p_registers->tdr = *itr;
                    itr++;
                }
            }

            if (false == bit::is_any(this->p_USART->p_registers->isr, ll::usart::ISR::pe | ll::usart::ISR::ne) &&
                utils::tick_counter<Milliseconds>::get() <= timeout_end_timestamp)
            {
                const std::uint64_t new_timeout =
                    a_timeout.get() - (utils::tick_counter<Milliseconds>::get() - timeout_start_timestamp);
                return this->transmit_2(Milliseconds(new_timeout), a_tail...) +
                       static_cast<std::uint32_t>(itr - itr_begin);
            }

            return static_cast<std::uint32_t>(itr - itr_begin);
        }

        bool is_transfer_complete();

    private:
        USART* p_USART = nullptr;
        friend class USART;
    };
    class Interrupt : private Non_copyable
    {
    public:
        struct Transmit_callback
        {
            using Function = void (*)(ll::usart::TDR* a_p_data, void* a_p_user_data);

            Function function = nullptr;
            void* p_user_data = nullptr;
        };
        struct Receive_callback
        {
            using Function = void (*)(std::uint32_t a_data, void* a_p_user_data);

            Function function = nullptr;
            void* p_user_data = nullptr;
        };
        struct Event_callback
        {
            using Function = void (*)(Event_flag a_event, void* a_p_user_data);

            Event_flag events = Event_flag::none;
            Function function = nullptr;
            void* p_user_data = nullptr;
        };

        ~Interrupt()
        {
            if (true == this->is_enabled())
            {
                this->disable();
            }
        }

        void enable(const IRQ_config& a_config);
        void disable();

        void transmit_start(const Transmit_callback& a_callback);
        void transmit_stop();

        void receive_start(const Receive_callback& a_callback);
        void receive_stop();

        void event_listening_start(const Event_callback& a_callback);
        void event_listening_stop();

        bool is_enabled() const
        {
            return 0 != NVIC_GetEnableIRQ(this->p_USART->irqn);
        }

    private:
        USART* p_USART = nullptr;
        friend class USART;
    };

    USART(USART&&) = default;
    USART& operator=(USART&&) = default;

    USART()
        : idx(std::numeric_limits<decltype(this->idx)>::max())
        , p_registers(nullptr)
        , irqn(static_cast<IRQn_Type>(std::numeric_limits<std::uint32_t>::max()))
    {
        this->polling.p_USART = nullptr;
        this->interrupt.p_USART = nullptr;
    }
    ~USART()
    {
        if (true == this->is_enabled())
        {
            this->disable();
        }
    }

    void enable(const Clock_config& a_clock_config,
                const Transceiving_config& a_transceiving_config,
                const Frame_format& a_frame_format,
                Low_power_wakeup_method a_low_power_wakeup);
    bool enable(const Clock_config& a_clock_config,
                const Transceiving_config& a_transceiving_config,
                const Frame_format& a_frame_format,
                Low_power_wakeup_method a_low_power_wakeup,
                Milliseconds a_timeout);

    void disable();

    bool is_enabled() const
    {
        return true == bit::is_any(this->p_registers->isr, ll::usart::ISR::reack | ll::usart::ISR::teack) &&
               true == bit::flag::is(this->p_registers->cr1, ll::usart::CR1::ue);
    }

    Transceiving_config get_transceiving_config() const
    {
        return {};
    }

    Frame_format get_frame_format() const
    {
        return {};
    }

    bool request_mute_mode(Milliseconds a_timeout) const
    {
        this->p_registers->rqr = ll::usart::RQR::mmrq;
        return utils::wait_until::all_bits_are_set(this->p_registers->isr, ll::usart::ISR::rwu, a_timeout);
    }

    bool is_mute_mode_requested() const
    {
        return bit::is(this->p_registers->rqr, ll::usart::RQR::mmrq);
    }

    operator ll::usart::Registers*()
    {
        return this->p_registers;
    }

    operator const ll::usart::Registers*() const
    {
        return this->p_registers;
    }

    Polling polling;
    Interrupt interrupt;

private:
    USART(std::uint32_t a_idx, ll::usart::Registers* a_p_registers, IRQn_Type a_irqn)
        : idx(a_idx)
        , p_registers(a_p_registers)
        , irqn(a_irqn)
    {
        this->polling.p_USART = this;
        this->interrupt.p_USART = this;
    }

    std::uint32_t idx;
    ll::usart::Registers* p_registers;

    IRQn_Type irqn;
    Interrupt::Transmit_callback transmit_callback;
    Interrupt::Receive_callback receive_callback;
    Interrupt::Event_callback event_callback;

    template<typename Periph_t, std::uint32_t id> friend class xmcu::soc::peripheral;
    friend void USART_interrupt_handler(USART* a_p_this);
};
void USART_interrupt_handler(USART* a_p_this);

constexpr USART::Transceiving_config::Flow_control_flag
operator|(USART::Transceiving_config::Flow_control_flag a_f1, USART::Transceiving_config::RS232_flow_control_flag a_f2)
{
    hkm_assert(USART::Transceiving_config::Flow_control_flag::RS232 == a_f1);

    return static_cast<USART::Transceiving_config::Flow_control_flag>(
        static_cast<std::uint32_t>(USART::Transceiving_config::Flow_control_flag::RS232) |
        static_cast<std::uint32_t>(a_f2));
}

constexpr USART::Transceiving_config::Flow_control_flag
operator|(USART::Transceiving_config::Flow_control_flag a_f1,
          const USART::Transceiving_config::RS485_flow_control_config& a_f2)
{
    hkm_assert(USART::Transceiving_config::Flow_control_flag::RS485 == a_f1);

    return static_cast<USART::Transceiving_config::Flow_control_flag>(
        static_cast<std::uint32_t>(USART::Transceiving_config::Flow_control_flag::RS485) |
        (a_f2.assertion_time << USART_CR1_DEAT_Pos) | (a_f2.deassertion_time << USART_CR1_DEDT_Pos));
}

constexpr USART::Transceiving_config::Flow_control_flag operator&(USART::Transceiving_config::Flow_control_flag a_f1,
                                                                  USART::Transceiving_config::Flow_control_flag a_f2)
{
    return static_cast<USART::Transceiving_config::Flow_control_flag>((static_cast<std::uint32_t>(a_f1) & 0x3u) &
                                                                      (static_cast<std::uint32_t>(a_f2) & 0x3u));
}

constexpr USART::Transceiving_config::Mode_flag operator|(USART::Transceiving_config::Mode_flag a_f1,
                                                          USART::Transceiving_config::Mode_flag a_f2)
{
    return static_cast<USART::Transceiving_config::Mode_flag>(static_cast<std::uint32_t>(a_f1) |
                                                              static_cast<std::uint32_t>(a_f2));
}

constexpr USART::Transceiving_config::Mode_flag operator&(USART::Transceiving_config::Mode_flag a_f1,
                                                          USART::Transceiving_config::Mode_flag a_f2)
{
    return static_cast<USART::Transceiving_config::Mode_flag>(static_cast<std::uint32_t>(a_f1) &
                                                              static_cast<std::uint32_t>(a_f2));
}

constexpr USART::Transceiving_config::Mode_flag operator|=(USART::Transceiving_config::Mode_flag& a_f1,
                                                           USART::Transceiving_config::Mode_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

constexpr USART::Transceiving_config::Mute_method operator|(USART::Transceiving_config::Mute_method a_f1,
                                                            std::uint8_t a_f2)
{
    hkm_assert(USART::Transceiving_config::Mute_method::character_matched == a_f1);
    return static_cast<USART::Transceiving_config::Mute_method>(static_cast<std::uint32_t>(a_f1) | a_f2);
}

constexpr USART::Transceiving_config::RS232_flow_control_flag
operator|(USART::Transceiving_config::RS232_flow_control_flag a_f1,
          USART::Transceiving_config::RS232_flow_control_flag a_f2)
{
    return static_cast<USART::Transceiving_config::RS232_flow_control_flag>(static_cast<std::uint32_t>(a_f1) |
                                                                            static_cast<std::uint32_t>(a_f2));
}

constexpr USART::Event_flag operator|(USART::Event_flag a_f1, USART::Event_flag a_f2)
{
    return static_cast<USART::Event_flag>(static_cast<std::uint32_t>(a_f1) | static_cast<std::uint32_t>(a_f2));
}

constexpr USART::Event_flag operator&(USART::Event_flag a_f1, USART::Event_flag a_f2)
{
    return static_cast<USART::Event_flag>(static_cast<std::uint32_t>(a_f1) & static_cast<std::uint32_t>(a_f2));
}

constexpr USART::Event_flag operator|=(USART::Event_flag& a_f1, USART::Event_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

} // namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals

namespace xmcu::soc::st::arm::m0::l0::rm0451 {
template<std::uint32_t id> class rcc<peripherals::USART, id> : private non_constructible
{
public:
    template<typename Source_t> static void enable(bool a_enable_in_lp) = delete;
    static void disable() = delete;
};
template<> template<> void rcc<peripherals::USART, 2u>::enable<clocks::pclk<2u>>(bool a_enable_in_lp);
template<> template<> void rcc<peripherals::USART, 2u>::enable<rcc<system::mcu<1u>>>(bool a_enable_in_lp);
template<> template<> void rcc<peripherals::USART, 2u>::enable<clocks::sources::hsi16>(bool a_enable_in_lp);
// template<> template<> void rcc<peripherals::USART, 2u>::enable<sources::lse>(bool a_enable_in_lp);
template<> void rcc<peripherals::USART, 2u>::disable();

template<>
inline void peripherals::GPIO::Alternate_function::enable<peripherals::USART, 2>(Limited<std::uint32_t, 0, 15> a_id,
                                                                                 const Enable_config& a_config,
                                                                                 Pin* a_p_pin)
{
    // Check if GPIO is eligible for RX/TX pin
    std::uint8_t alternate_function_index;
#if defined(XMCU_SOC_MODEL_STM32L010F4P6)
    // DS12323 table 11
    hkm_assert(
        (0u == this->p_port->idx && (0u == a_id || 2u == a_id || 3u == a_id || 9u == a_id || 10u == a_id || // PORTA
                                     14u == a_id || 15u == a_id || 1u == a_id || 12u == a_id)) ||           // PORTA
        (1u == this->p_port->idx && (6u == a_id || 7u == a_id || 0u == a_id)));                             // PORTB

    // Be careful when enabling CTS functionality, as pin PA0 is shared with RX functionality.
    alternate_function_index =
        ((0u == this->p_port->idx && a_id > 0) || (1u == this->p_port->idx && 0u == a_id)) ? 4u : 0u;
#elif defined(XMCU_SOC_MODEL_STM32L010C6T6)
    hkm_assert(
        (0u == this->p_port->idx && (2u == a_id || 3u == a_id || 9u == a_id || 10u == a_id || 14u == a_id || // PORTA
                                     15u == a_id || 1u == a_id || 12u == a_id)) ||                           // PORTA
        (1u == this->p_port->idx && (6u == a_id || 7u == a_id || 0u == a_id)));                              // PORTB

    alternate_function_index = ((0u == this->p_port->idx) || (1u == this->p_port->idx && 0u == a_id)) ? 4u : 0u;
#else
#error "Undefined pin constraints for uC - add missing model refering to its *datasheet*"
#endif

    this->enable(a_id, a_config, alternate_function_index, a_p_pin);
}

} // namespace xmcu::soc::st::arm::m0::l0::rm0451

namespace xmcu::soc {
template<> class peripheral<st::arm::m0::l0::rm0451::peripherals::USART, 2u> : private non_constructible
{
public:
    static st::arm::m0::l0::rm0451::peripherals::USART create()
    {
        return st::arm::m0::l0::rm0451::peripherals::USART(
            0u,
            reinterpret_cast<st::arm::m0::l0::rm0451::peripherals::ll::usart::Registers*>(USART2),
            IRQn_Type::USART2_IRQn);
    }
};
} // namespace xmcu::soc
