/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// this
#include <rm0451/peripherals/GPIO/GPIO.hpp>

// xmcu
#include <soc/Scoped_guard.hpp>
#include <soc/st/arm/IRQ_config.hpp>
#include <soc/st/arm/m0/nvic.hpp>
#include <xmcu/bit.hpp>

// debug
#include <xmcu/assertion.hpp>

namespace {
using namespace xmcu;
using namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals;

GPIO::Interrupt::Callback callbacks[3u];

void shared_exti_int_handler(std::uint32_t a_start, std::uint32_t a_end, std::uint32_t a_handler_index)
{
    for (std::uint32_t i = a_start; i <= a_end; ++i)
    {
        std::uint32_t bit_index = i - 1;
        if (true == bit::is(EXTI->PR, bit_index))
        {
            hkm_assert(nullptr != callbacks[a_handler_index].function);
            callbacks[a_handler_index].function(bit_index, callbacks[a_handler_index].p_user_data);
            bit::set(&(EXTI->PR), bit_index);
        }
    }
}
} // namespace

extern "C" {

void EXTI0_1_IRQHandler()
{
    shared_exti_int_handler(0u, 1u, 0u);
}

void EXTI2_3_IRQHandler()
{
    shared_exti_int_handler(2u, 3u, 1u);
}

void EXTI4_15_IRQHandler()
{
    shared_exti_int_handler(4u, 15u, 2u);
}
}

namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals {
using namespace xmcu;
using namespace xmcu::soc::st::arm::m0::l0::rm0451::clocks::sources;
using namespace xmcu::soc::st::arm::m0::l0::rm0451::system;

void GPIO::In::Pin::set_pull(Pull a_pull)
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    bit::flag::set(&(this->p_port->p_registers->pupdr),
                   ll::gpio::PUPDR::mask << this->id,
                   static_cast<ll::gpio::PUPDR::Flag>(a_pull) << this->id);
}

GPIO::Level GPIO::In::Pin::get_level() const
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    return static_cast<GPIO::Level>(bit::is(this->p_port->p_registers->idr, Limited<std::uint32_t, 0u, 15u>(this->id)));
}

GPIO::Pull GPIO::In::Pin::get_pull() const
{
    hkm_assert(nullptr != this->p_port && 0xFF != this->id);

    return static_cast<Pull>(bit::flag::get(this->p_port->p_registers->pupdr, ll::gpio::PUPDR::mask << this->id) >>
                             this->id);
}

void GPIO::Out::Pin::set_level(Level a_level)
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    xmcu::bit::flag::set(&(this->p_port->p_registers->odr),
                         ll::gpio::ODR::mask << this->id,
                         static_cast<ll::gpio::ODR::Flag>(a_level) << this->id);
}

void GPIO::Out::Pin::toggle_level()
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    bit::toggle(&(this->p_port->p_registers->odr), this->id);
}

void GPIO::Out::Pin::set_type(Type a_type)
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    bit::flag::set(&(this->p_port->p_registers->otyper),
                   ll::gpio::OTYPER::mask << this->id,
                   static_cast<ll::gpio::OTYPER::Flag>(a_type) << this->id);
}

void GPIO::Out::Pin::set_pull(Pull a_pull)
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    bit::flag::set(&(this->p_port->p_registers->pupdr),
                   ll::gpio::PUPDR::mask << this->id,
                   static_cast<ll::gpio::PUPDR::Flag>(a_pull) << this->id);
}

void GPIO::Out::Pin::set_speed(Speed a_speed)
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    bit::flag::set(&(this->p_port->p_registers->ospeedr),
                   ll::gpio::OSPEEDR::mask << this->id,
                   static_cast<ll::gpio::OSPEEDR::Flag>(a_speed) << this->id);
}

GPIO::Level GPIO::Out::Pin::get_level() const
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    return static_cast<GPIO::Level>(bit::is(this->p_port->p_registers->idr, Limited<std::uint32_t, 0u, 15u>(this->id)));
}

GPIO::Type GPIO::Out::Pin::get_type() const
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    return static_cast<Type>(bit::flag::get(this->p_port->p_registers->otyper, ll::gpio::OTYPER::mask << this->id) >>
                             this->id);
}

GPIO::Pull GPIO::Out::Pin::get_pull() const
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    return static_cast<Pull>(bit::flag::get(this->p_port->p_registers->pupdr, ll::gpio::PUPDR::mask << this->id) >>
                             this->id);
}

GPIO::Speed GPIO::Out::Pin::get_speed() const
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    return static_cast<Speed>(bit::flag::get(this->p_port->p_registers->ospeedr, ll::gpio::OSPEEDR::mask << this->id) >>
                              this->id);
}

void GPIO::Out::Bus::set_value(std::uint32_t a_value)
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id_start && 0xFFu != this->id_end);

    const std::uint32_t bus_width = 1 + this->id_end - this->id_start;
    std::uint32_t mask_from_lower_bit = (1 << bus_width) - 1;

    hkm_assert(0 == (mask_from_lower_bit & a_value));
    std::uint32_t mask = mask_from_lower_bit << this->id_start;

    std::uint32_t sr_val = mask << 16 | a_value << this->id_start;
    this->p_port->p_registers->bsrr = static_cast<ll::gpio::BSRR::Data>(sr_val);
}

// TODO: parallel write to configuration registers

void GPIO::Out::Bus::set_type(Type a_type)
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id_start && 0xFFu != this->id_end);

    ll::gpio::OTYPER temp = this->p_port->p_registers->otyper;

    for (std::int32_t id = this->id_start; id <= this->id_end; ++id)
    {
        bit::flag::set(&temp, ll::gpio::OTYPER::mask << id, static_cast<ll::gpio::OTYPER::Flag>(a_type) << id);
    }

    this->p_port->p_registers->otyper = temp;
}

void GPIO::Out::Bus::set_pull(Pull a_pull)
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id_start && 0xFFu != this->id_end);

    ll::gpio::PUPDR temp = this->p_port->p_registers->pupdr;

    for (std::int32_t id = this->id_start; id <= this->id_end; ++id)
    {
        bit::flag::set(&temp, ll::gpio::PUPDR::mask << id, static_cast<ll::gpio::PUPDR::Flag>(a_pull) << id);
    }

    this->p_port->p_registers->pupdr = temp;
}

void GPIO::Out::Bus::set_speed(Speed a_speed)
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id_start && 0xFFu != this->id_end);

    ll::gpio::OSPEEDR temp = this->p_port->p_registers->ospeedr;

    for (std::int32_t id = this->id_start; id <= this->id_end; ++id)
    {
        bit::flag::set(&temp, ll::gpio::OSPEEDR::mask << id, static_cast<ll::gpio::OSPEEDR::Flag>(a_speed) << id);
    }

    this->p_port->p_registers->ospeedr = temp;
}

// LookUpTableBus

void GPIO::Analog::Pin::set_pull(Pull a_pull)
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    bit::flag::set(&(this->p_port->p_registers->pupdr),
                   ll::gpio::PUPDR::mask << this->id,
                   static_cast<ll::gpio::PUPDR::Flag>(a_pull) << this->id);
}

GPIO::Pull GPIO::Analog::Pin::get_pull() const
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    return static_cast<Pull>(bit::flag::get(this->p_port->p_registers->pupdr, ll::gpio::PUPDR::mask << this->id) >>
                             this->id);
}

void GPIO::Alternate_function::Pin::set_type(Type a_type)
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    bit::flag::set(&(this->p_port->p_registers->otyper),
                   ll::gpio::OTYPER::mask << this->id,
                   static_cast<ll::gpio::OTYPER::Flag>(a_type) << this->id);
}

void GPIO::Alternate_function::Pin::set_pull(Pull a_pull)
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    bit::flag::set(&(this->p_port->p_registers->pupdr),
                   ll::gpio::PUPDR::mask << this->id,
                   static_cast<ll::gpio::PUPDR::Flag>(a_pull) << this->id);
}

void GPIO::Alternate_function::Pin::set_speed(Speed a_speed)
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    bit::flag::set(&(this->p_port->p_registers->ospeedr),
                   ll::gpio::OSPEEDR::mask << this->id,
                   static_cast<ll::gpio::OSPEEDR::Flag>(a_speed) << this->id);
}

GPIO::Type GPIO::Alternate_function::Pin::get_type() const
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    return static_cast<Type>(bit::flag::get(this->p_port->p_registers->otyper, ll::gpio::OTYPER::mask << this->id) >>
                             this->id);
}

GPIO::Pull GPIO::Alternate_function::Pin::get_pull() const
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    return static_cast<Pull>(bit::flag::get(this->p_port->p_registers->pupdr, ll::gpio::PUPDR::mask << this->id) >>
                             this->id);
}

GPIO::Speed GPIO::Alternate_function::Pin::get_speed() const
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    return static_cast<Speed>(bit::flag::get(this->p_port->p_registers->ospeedr, ll::gpio::OSPEEDR::mask << this->id) >>
                              this->id);
}

void GPIO::In::enable(Limited<std::uint32_t, 0, 15> a_id, Pull a_pull, Pin* a_p_pin)
{
    hkm_assert(false == this->p_port->is_pin_taken(a_id));

    ll::gpio::Registers* p_port = this->p_port->p_registers;

    bit::flag::set(&(p_port->pupdr), ll::gpio::PUPDR::mask << a_id, static_cast<ll::gpio::PUPDR::Flag>(a_pull) << a_id);
    bit::flag::clear(&(p_port->moder), ll::gpio::MODER::mask << a_id);

    this->p_port->take_pin(a_id);

    if (nullptr != a_p_pin)
    {
        a_p_pin->id = a_id;
        a_p_pin->p_port = this->p_port;
    }
}

void GPIO::In::disable(Limited<std::uint32_t, 0, 15> a_id)
{
    hkm_assert(false == this->p_port->is_pin_taken(a_id));

    ll::gpio::Registers* p_port = this->p_port->p_registers;

    bit::flag::set(&(p_port->moder), ll::gpio::MODER::mask << a_id);
    bit::flag::clear(&(p_port->pupdr), ll::gpio::PUPDR::mask << a_id);

    this->p_port->give_pin(a_id);
}
void GPIO::In::disable(Pin* p_pin)
{
    this->disable(p_pin->get_id());

    p_pin->p_port = nullptr;
    p_pin->id = 0xFFu;
}

void GPIO::Out::enable(Limited<std::uint32_t, 0, 15> a_id, const Enable_config& a_config, Pin* a_p_pin)
{
    hkm_assert(various::get_enum_incorrect_value<Pull>() != a_config.pull);
    hkm_assert(various::get_enum_incorrect_value<Speed>() != a_config.speed);
    hkm_assert(various::get_enum_incorrect_value<Type>() != a_config.type);

    hkm_assert(false == this->p_port->is_pin_taken(a_id));

    ll::gpio::Registers* p_port = this->p_port->p_registers;

    bit::flag::set(&(p_port->ospeedr),
                   ll::gpio::OSPEEDR::mask << a_id,
                   static_cast<ll::gpio::OSPEEDR::Flag>(a_config.speed) << a_id);
    bit::flag::set(
        &(p_port->pupdr), ll::gpio::PUPDR::mask << a_id, static_cast<ll::gpio::PUPDR::Flag>(a_config.pull) << a_id);
    bit::flag::set(&(p_port->moder), ll::gpio::MODER::mask << a_id, ll::gpio::MODER::output << a_id);
    bit::flag::set(
        &(p_port->otyper), ll::gpio::OTYPER::mask << a_id, static_cast<ll::gpio::OTYPER::Flag>(a_config.type) << a_id);

    this->p_port->take_pin(a_id);

    if (nullptr != a_p_pin)
    {
        a_p_pin->id = a_id;
        a_p_pin->p_port = this->p_port;
    }
}

void GPIO::Out::enable(Limited<std::uint32_t, 0, 15> a_id, const Enable_config& a_config, Bus* a_p_bus)
{
    hkm_assert(nullptr != a_p_bus);

    hkm_assert(various::get_enum_incorrect_value<Pull>() != a_config.pull);
    hkm_assert(various::get_enum_incorrect_value<Speed>() != a_config.speed);
    hkm_assert(various::get_enum_incorrect_value<Type>() != a_config.type);

    hkm_assert(false == this->p_port->is_pin_taken(a_id));
    if (0xFF != a_p_bus->id_start)
    {
        hkm_assert((a_p_bus->id_end + 1u) == a_id);
        hkm_assert(a_p_bus->p_port == this->p_port);

        a_p_bus->id_end = a_id;
    }
    else
    {
        a_p_bus->id_start = a_p_bus->id_end = a_id;
        a_p_bus->p_port = this->p_port;
    }

    this->enable(a_id, a_config);
}

void GPIO::Out::disable(Limited<std::uint32_t, 0, 15> a_id)
{
    hkm_assert(false == this->p_port->is_pin_taken(a_id));

    ll::gpio::Registers* p_port = this->p_port->p_registers;

    bit::flag::set(&(p_port->moder), ll::gpio::MODER::mask << a_id);
    bit::flag::clear(&(p_port->ospeedr), ll::gpio::OSPEEDR::mask << a_id);
    bit::flag::clear(&(p_port->pupdr), ll::gpio::PUPDR::mask << a_id);

    this->p_port->give_pin(a_id);
}
void GPIO::Out::disable(Pin* p_pin)
{
    this->disable(p_pin->get_id());

    p_pin->p_port = nullptr;
    p_pin->id = 0xFFu;
}

void GPIO::Analog::enable(Limited<std::uint32_t, 0, 15> a_id, Pull a_pull, Pin* a_p_out_pin)
{
    hkm_assert(false == this->p_port->is_pin_taken(a_id));

    ll::gpio::Registers* p_port = this->p_port->p_registers;

    bit::flag::set(&(p_port->pupdr), ll::gpio::PUPDR::mask << a_id, static_cast<ll::gpio::PUPDR::Flag>(a_pull) << a_id);
    bit::flag::set(&(p_port->moder), ll::gpio::MODER::mask << a_id, ll::gpio::MODER::analog << a_id);

    this->p_port->take_pin(a_id);

    if (nullptr != a_p_out_pin)
    {
        a_p_out_pin->id = a_id;
        a_p_out_pin->p_port = this->p_port;
    }
}

void GPIO::Analog::disable(Limited<std::uint32_t, 0, 15> a_id)
{
    bit::flag::clear(&(this->p_port->p_registers->pupdr), (ll::gpio::PUPDR::mask << a_id));

    this->p_port->give_pin(a_id);
}
void GPIO::Analog::disable(Pin* p_pin)
{
    this->disable(p_pin->get_id());

    p_pin->p_port = nullptr;
    p_pin->id = 0xFFu;
}

void GPIO::Alternate_function::disable(Limited<std::uint32_t, 0, 15> a_id)
{
    ll::gpio::Registers* p_port = this->p_port->p_registers;

    bit::flag::set(&(p_port->moder), ll::gpio::MODER::mask << a_id);
    bit::flag::clear(&(p_port->ospeedr), ll::gpio::OSPEEDR::mask << a_id);
    bit::flag::clear(&(p_port->pupdr), ll::gpio::PUPDR::mask << a_id);

    this->p_port->give_pin(a_id);
}
void GPIO::Alternate_function::disable(Pin* p_pin)
{
    this->disable(p_pin->get_id());

    p_pin->p_port = nullptr;
    p_pin->id = 0xFFu;
}

void GPIO::Alternate_function::enable(Limited<std::uint32_t, 0, 15> a_id,
                                      const Enable_config& a_config,
                                      std::uint32_t a_function,
                                      Pin* a_p_pin)
{
    hkm_assert(false == this->p_port->is_pin_taken(a_id));

    hkm_assert(various::get_enum_incorrect_value<Pull>() != a_config.pull);
    hkm_assert(various::get_enum_incorrect_value<Speed>() != a_config.speed);
    hkm_assert(various::get_enum_incorrect_value<Type>() != a_config.type);

    ll::gpio::Registers* p_port = this->p_port->p_registers;

    bit::flag::set(&(p_port->ospeedr),
                   ll::gpio::OSPEEDR::mask << a_id,
                   static_cast<ll::gpio::OSPEEDR::Flag>(a_config.speed) << a_id);
    bit::flag::set(
        &(p_port->pupdr), ll::gpio::PUPDR::mask << a_id, static_cast<ll::gpio::PUPDR::Flag>(a_config.pull) << a_id);
    bit::flag::set(
        &(p_port->otyper), ll::gpio::OTYPER::mask << a_id, static_cast<ll::gpio::OTYPER::Flag>(a_config.type) << a_id);

    const std::uint32_t index = a_id >> 3u;
    const std::uint32_t shift = a_id - (index * 8u);

    hkm_assert(various::countof(p_port->afr) > index);

    bit::flag::set(
        &(p_port->afr[index]), ll::gpio::AFR::mask << shift, static_cast<ll::gpio::AFR::Flag>(a_function) << shift);
    bit::flag::set(&(p_port->moder), ll::gpio::MODER::mask << a_id, ll::gpio::MODER::af << a_id);

    this->p_port->take_pin(a_id);

    if (nullptr != a_p_pin)
    {
        a_p_pin->id = a_id;
        a_p_pin->p_port = this->p_port;
        a_p_pin->function = a_function;
    }
}

void GPIO::Interrupt::enable(const Callback& a_callback, const IRQ_config& a_irq_config)
{
    hkm_assert(std::numeric_limits<decltype(this->idx)>::max() != this->idx);

    NVIC_SetPriority(
        this->irqn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(this->irqn);

    callbacks[this->idx] = a_callback;
}

void GPIO::Interrupt::disable()
{
    hkm_assert(std::numeric_limits<decltype(this->idx)>::max() != this->idx);

    NVIC_DisableIRQ(this->irqn);

    callbacks[this->idx] = { nullptr, nullptr };
}

void GPIO::Interrupt::attach(const GPIO& a_port, std::uint32_t a_pin, Trigger_flag a_trigger, Type a_type)
{
    hkm_assert(std::numeric_limits<decltype(this->idx)>::max() != this->idx);

    volatile std::uint32_t* p_register = &(SYSCFG->EXTICR[a_pin / 4u]);
    std::uint32_t pos = ((static_cast<std::uint32_t>(a_pin) % 4u) * 4u);

#if defined(XMCU_ASSERT_ENABLED)
    const bool f = bit::flag::is(*p_register, (a_port.idx) << pos);
    hkm_assert((0u == a_port.idx && true == f) || (0u != a_port.idx && false == f));
    hkm_assert((0u == this->idx && 0u == a_pin) || (1u == this->idx && 1u == a_pin) ||
               (2u == this->idx && 2u == a_pin) || (3u == this->idx && 3u == a_pin) ||
               (4u == this->idx && 4u == a_pin) || (5u == this->idx && (a_pin >= 5u && a_pin <= 9u)) ||
               (6u == this->idx && (a_pin >= 10u && a_pin <= 15u)));
#endif

    Scoped_guard<nvic> guard;

    bit::flag::set(p_register, 0x3u << pos, a_port.idx << pos);

    bit::clear(&(EXTI->RTSR), a_pin);
    bit::clear(&(EXTI->FTSR), a_pin);

    switch (a_type)
    {
        case Type::event: {
            bit::set(&(EXTI->EMR), a_pin);
        }
        break;

        case Type::interrupt: {
            bit::set(&(EXTI->IMR), a_pin);
        }
        break;
    }

    switch (a_trigger)
    {
        case Trigger_flag::rising: {
            bit::set(&(EXTI->RTSR), a_pin);
        }
        break;

        case Trigger_flag::falling: {
            bit::set(&(EXTI->FTSR), a_pin);
        }
        break;

        default: {
            if ((Trigger_flag::rising | Trigger_flag::falling) == a_trigger)
            {
                bit::set(&(EXTI->RTSR), a_pin);
                bit::set(&(EXTI->FTSR), a_pin);
            }
        }
    }
}
void GPIO::Interrupt::attach(const GPIO::In::Pin& a_pin, Trigger_flag a_trigger, Type a_type)
{
    this->attach(*(a_pin.get_port()), a_pin.get_id(), a_trigger, a_type);
}
void GPIO::Interrupt::attach(const GPIO::Out::Pin& a_pin, Trigger_flag a_trigger, Type a_type)
{
    this->attach(*(a_pin.get_port()), a_pin.get_id(), a_trigger, a_type);
}
void GPIO::Interrupt::attach(const GPIO::Alternate_function::Pin& a_pin, Trigger_flag a_trigger, Type a_type)
{
    this->attach(*(a_pin.get_port()), a_pin.get_id(), a_trigger, a_type);
}

void GPIO::Interrupt::deattach(const GPIO& a_port, std::uint32_t a_pin)
{
    hkm_assert(std::numeric_limits<decltype(this->idx)>::max() != this->idx);

    Scoped_guard<nvic> guard;

    bit::clear(&(EXTI->RTSR), a_pin);
    bit::clear(&(EXTI->FTSR), a_pin);

    bit::clear(&(EXTI->EMR), a_pin);
    bit::clear(&(EXTI->IMR), a_pin);

    bit::flag::clear(&(SYSCFG->EXTICR[a_pin / 4u]), a_port.idx << ((static_cast<std::uint32_t>(a_pin) % 4u) * 4u));

    callbacks[this->idx] = { nullptr, nullptr };
}
void GPIO::Interrupt::deattach(const GPIO::In::Pin& a_pin)
{
    this->deattach(*(a_pin.get_port()), a_pin.get_id());
}
void GPIO::Interrupt::deattach(const GPIO::Out::Pin& a_pin)
{
    this->deattach(*(a_pin.get_port()), a_pin.get_id());
}
void GPIO::Interrupt::deattach(const GPIO::Alternate_function::Pin& a_pin)
{
    this->deattach(*(a_pin.get_port()), a_pin.get_id());
}

template<> void GPIO::mco::enable<hse>(Divider a_divider)
{
    bit::flag::set(&(RCC->CFGR), static_cast<std::uint32_t>(a_divider));
    bit::flag::set(&(RCC->CFGR), RCC_CFGR_MCOSEL, RCC_CFGR_MCOSEL_2);
}

template<> void GPIO::mco::enable<hsi16>(Divider a_divider)
{
    bit::flag::set(&(RCC->CFGR), static_cast<std::uint32_t>(a_divider));
    bit::flag::set(&(RCC->CFGR), RCC_CFGR_MCOSEL, RCC_CFGR_MCOSEL_0 | RCC_CFGR_MCOSEL_1);
}

template<> void GPIO::mco::enable<rcc<mcu<1u>>>(Divider a_divider)
{
    bit::flag::set(&(RCC->CFGR), static_cast<std::uint32_t>(a_divider));
    bit::flag::set(&(RCC->CFGR), RCC_CFGR_MCOSEL, RCC_CFGR_MCOSEL_0);
}

template<> void GPIO::mco::enable<msi>(Divider a_divider)
{
    bit::flag::set(&(RCC->CFGR), static_cast<std::uint32_t>(a_divider));
    bit::flag::set(&(RCC->CFGR), RCC_CFGR_MCOSEL, RCC_CFGR_MCOSEL_1);
}

void GPIO::mco::disable()
{
    bit::flag::clear(&(RCC->CFGR), RCC_CFGR_MCOSEL);
    bit::flag::clear(&(RCC->CFGR), RCC_CFGR_MCOPRE);
}

} // namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals

namespace xmcu::soc::st::arm::m0::l0::rm0451 {
using namespace xmcu::soc::st::arm::m0::l0::rm0451::system;

#if defined(XMCU_GPIOA_PRESENT)
void rcc<GPIO, GPIO::A>::enable(bool a_enable_in_lp)
{
    bit::flag::set(&(RCC->IOPENR), RCC_IOPENR_GPIOAEN);

    if (true == a_enable_in_lp)
    {
        bit::flag::set(&(RCC->IOPSMENR), RCC_IOPSMENR_GPIOASMEN);
    }
    else
    {
        bit::flag::clear(&(RCC->IOPSMENR), RCC_IOPSMENR_GPIOASMEN);
    }
}
void rcc<GPIO, GPIO::A>::disable()
{
    bit::flag::clear(&(RCC->IOPENR), RCC_IOPENR_GPIOAEN);
}
#endif

#if defined(XMCU_GPIOB_PRESENT)
void rcc<GPIO, GPIO::B>::enable(bool a_enable_in_lp)
{
    bit::flag::set(&(RCC->IOPENR), RCC_IOPENR_GPIOBEN);

    if (true == a_enable_in_lp)
    {
        bit::flag::set(&(RCC->IOPSMENR), RCC_IOPSMENR_GPIOBSMEN);
    }
    else
    {
        bit::flag::clear(&(RCC->IOPSMENR), RCC_IOPSMENR_GPIOBSMEN);
    }
}
void rcc<GPIO, GPIO::B>::disable()
{
    bit::flag::clear(&(RCC->IOPENR), RCC_IOPENR_GPIOBEN);
}
#endif

#if defined(XMCU_GPIOC_PRESENT)
void rcc<GPIO, GPIO::C>::enable(bool a_enable_in_lp)
{
    bit::flag::set(&(RCC->IOPENR), RCC_IOPENR_GPIOCEN);

    if (true == a_enable_in_lp)
    {
        bit::flag::set(&(RCC->IOPSMENR), RCC_IOPSMENR_GPIOCSMEN);
    }
    else
    {
        bit::flag::clear(&(RCC->IOPSMENR), RCC_IOPSMENR_GPIOCSMEN);
    }
}
void rcc<GPIO, GPIO::C>::disable()
{
    bit::flag::clear(&(RCC->IOPENR), RCC_IOPENR_GPIOCEN);
}
#endif

template<> void GPIO::Alternate_function::enable<GPIO::mco>(Limited<std::uint32_t, 0, 15> a_id,
                                                            const Enable_config& a_config,
                                                            Pin* a_p_pin)
{
    hkm_assert((0u == this->p_port->idx && 8u == a_id) || (1u == this->p_port->idx && 6u == a_id));

    this->enable(a_id, a_config, 0x0u, a_p_pin);
}
} // namespace xmcu::soc::st::arm::m0::l0::rm0451