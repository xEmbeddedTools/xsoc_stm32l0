#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// std
#include <array>
#include <cstdint>
#include <type_traits>
// #include <ranges>

// externals
#include <stm32l0xx.h>

// xmcu
#include <rm0451/clocks/sources/hse.hpp>
#include <rm0451/clocks/sources/hsi16.hpp>
#include <rm0451/clocks/sources/msi.hpp>
#include <rm0451/peripherals/GPIO/gpio_ll.hpp>
#include <rm0451/rcc.hpp>
#include <rm0451/system/mcu/mcu.hpp>
#include <soc/peripheral.hpp>
#include <soc/st/arm/IRQ_config.hpp>
#include <xmcu/Limited.hpp>
#include <xmcu/Non_copyable.hpp>
#include <xmcu/bit.hpp>
#include <xmcu/non_constructible.hpp>
#include <xmcu/various.hpp>

// debug
#include <xmcu/assertion.hpp>

namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals {

    namespace multi_output_look_up_table
    {        
        // TODO could be declared one level up inside GPIO
        // look to be done.
        // TODO could be declared a few levels up (common?)
        struct Description_entry // is it smart enough?
        {
            uint8_t word_idx;
            uint8_t bit_idx;
            consteval Description_entry(uint8_t a_word_idx, uint8_t a_bit_idx)
                : word_idx(a_word_idx)
                , bit_idx(a_bit_idx)
            {
            }
            
            consteval Description_entry()
                : word_idx(0)
                , bit_idx(0)
            {
            }
        };
        template<size_t N> struct Log2_Very_Smart
        {
        };

        template<> struct Log2_Very_Smart<16>
        {
            static constexpr size_t value = 4;
        };
        template<> struct Log2_Very_Smart<2>
        {
            static constexpr size_t value = 1;
        };

        using LUT_entry = std::array<unsigned, 2>; // could be LUT_row?

        template<size_t N> consteval auto compose_table(const std::array<Description_entry, N>& a_description) 
        {
            LUT_entry common_mask {};

            // move to dedicated function creating mask
            size_t common_mask_shift = 16; // TODO export (import) as parameter?
            size_t common_mask_shift_base = 1u<<common_mask_shift;

            for (auto& input : a_description)
            {
                common_mask[input.word_idx] |= common_mask_shift_base << input.bit_idx;
            }
            // [end] move to dedicated function creating mask

            std::array<LUT_entry, 1 << N> result_array;
            result_array.fill(LUT_entry {});
            result_array.fill(common_mask);
            for (size_t channel = 0; channel < result_array.size(); ++channel)
            {
                for (size_t i = 0; i < a_description.size(); ++i)
                {
                    size_t bit = 1u << i;

                    if (0 == ((bit) & channel))
                    {
                        continue;
                    }
                    auto input = a_description[i];
                    auto& channel_variable = result_array.at(channel);
                    channel_variable[input.word_idx] |= 1u << input.bit_idx;
                }
            }
            return result_array;
        }

        template<size_t N, size_t X>
        // N parameter is same as result array size. It is differnet than `compose_table` template parameter
        consteval auto combine_tables(const std::array<std::array<LUT_entry, N>, X>& a_tables)
        {
            std::array<LUT_entry, N> result_array;
            result_array.fill(LUT_entry{}); // zeroing results or copy and drop a_tables[0]
            // std::array<LUT_entry, N> result_array = a_tables[0];
            // std::array ranges_drop_workaround { a_tables.begin() + 1, a_tables.end() };
            // TODO test #include <ranges> : std::ranges::views::drop(1)
            for (auto& table : a_tables) 
            {
                for (size_t i = 0; i < result_array.size(); ++i)
                {
                    // result_array[i] |= table[i];
                    result_array[i][0] |= table[i][0];
                    result_array[i][1] |= table[i][1]; // ninja xD
                }
            }
            return result_array;
        }
        template<size_t N, size_t X>
            requires((X == 2))
        // N parameter is same as result array size. It is differnet than `compose_table` template parameter
        consteval auto combine_tables_combined_args(const std::array<std::array<LUT_entry, N>, X>& a_tables)
        {
            std::array<LUT_entry, N*N> result_array; // TODO: is silent hardcoded to X=2
            result_array.fill(LUT_entry{});

            for (size_t i = 0; i < result_array.size(); ++i)
            {
                // TODO: is silent hardoced...
                LUT_entry& row = result_array[i];
                // TODO N=16;
                auto shift = Log2_Very_Smart<N>::value;
                auto mask = N - 1;
                // auto& r0 = a_tables[0][i & 0x01];
                // auto& r1 = a_tables[1][i >> 1 & 0x01];
                auto& r0 = a_tables[0][i & mask];
                auto& r1 = a_tables[1][i >> shift & mask];
                row[0] = r0[0] | r1[0];
                row[1] = r0[1] | r1[1];
            }
            return result_array;
        }
    }
class GPIO : private Non_copyable
{
public:
#if defined(XMCU_GPIOA_PRESENT)
    using A = ll::gpio_base::A;
#endif
#if defined(XMCU_GPIOB_PRESENT)
    using B = ll::gpio_base::B;
#endif
#if defined(XMCU_GPIOC_PRESENT)
    using C = ll::gpio_base::C;
#endif
#if defined(XMCU_GPIOH_PRESENT)
    using H = ll::gpio_base::H;
#endif

    enum class Level : std::uint32_t
    {
        low = 0x0u,
        high = 0x1u
    };
    enum class Type : std::uint32_t
    {
        push_pull = static_cast<std::uint32_t>(ll::gpio::OTYPER::push_pull),
        open_drain = static_cast<std::uint32_t>(ll::gpio::OTYPER::open_drain),
    };
    enum class Pull : std::uint32_t
    {
        none = static_cast<std::uint32_t>(ll::gpio::PUPDR::none),
        up = static_cast<std::uint32_t>(ll::gpio::PUPDR::pull_up),
        down = static_cast<std::uint32_t>(ll::gpio::PUPDR::pull_down),
    };
    enum class Speed : std::uint32_t
    {
        low = static_cast<std::uint32_t>(ll::gpio::OSPEEDR::low),
        medium = static_cast<std::uint32_t>(ll::gpio::OSPEEDR::medium),
        high = static_cast<std::uint32_t>(ll::gpio::OSPEEDR::high),
        ultra = static_cast<std::uint32_t>(ll::gpio::OSPEEDR::ultra),
    };

    class Out : private xmcu::Non_copyable
    {
    public:
        struct Enable_config
        {
            Type type = various::get_enum_incorrect_value<Type>();
            Pull pull = various::get_enum_incorrect_value<Pull>();
            Speed speed = various::get_enum_incorrect_value<Speed>();
        };

        class Pin : private xmcu::Non_copyable
        {
        public:
            Pin()
                : p_port(nullptr)
                , id(0xFFu)
            {
            }

            void set_level(Level a_level);
            void toggle_level();

            void set_type(Type a_type);
            void set_pull(Pull a_pull);
            void set_speed(Speed a_speed);

            Level get_level() const;
            Type get_type() const;
            Pull get_pull() const;
            Speed get_speed() const;

            GPIO* get_port() const
            {
                return this->p_port;
            }
            std::uint8_t get_id() const
            {
                return this->id;
            }

        private:
            GPIO* p_port;
            std::uint8_t id;

            friend Out;
        };
        class Bus : private Non_copyable
        {
        public:
            Bus()
                : p_port(nullptr)
                , id_start(0xFFu)
                , id_end(0xFF)
            {
            }

            void set_value(std::uint32_t a_value);

            void set_type(Type a_type);
            void set_pull(Pull a_pull);
            void set_speed(Speed a_speed);

            GPIO* get_port() const
            {
                return this->p_port;
            }

        private:
            GPIO* p_port;
            std::uint8_t id_start, id_end;

            friend Out;
        };

        using Description_entry  = multi_output_look_up_table::Description_entry;
        using LUT_entry = multi_output_look_up_table::LUT_entry;

        template<typename T,size_t N>
        class LookUpTableBasedBus : private Non_copyable // TODO: Someone bring me a better name?
        {
            // Strongly concept code xD
            #define CACHING_BSRR
        public:
            static consteval int test_map(const std::array<T, N>& a_map)
            {
                // assume number of bits^2 == N

                // GPIO_BSRR_BS and GPIO_BSRR_BR should be same in last row
                auto last_row = a_map[N - 1];

                constexpr size_t BSRR_BS_MASK = GPIO_BSRR_BS_15 | (GPIO_BSRR_BS_15 - 1);
                constexpr size_t BSRR_BR_MASK = GPIO_BSRR_BR_15 | (GPIO_BSRR_BR_15 - 1);
                size_t bitcounter = 0;
                for (size_t word_in_row : last_row)
                {
                    const size_t tested_bs = word_in_row & BSRR_BS_MASK;
                    const size_t tested_br = word_in_row & BSRR_BR_MASK;
                    if (tested_bs != tested_br >> 16)
                    {
                        return -1;
                    }

                    for (size_t i = 0; i <= BSRR_BS_MASK; i <<= 1)
                    {
                        if (tested_bs & i)
                        {
                            ++bitcounter;
                        }
                    }
                }
                return bitcounter;
            }
            // We might need to initialize ports. Initialize separately is a bit ugly.
            LookUpTableBasedBus(const std::array<T, N>& a_map, const std::array<GPIO*, 2>& a_ports)
                : lut(a_map)
                , p_ports(a_ports)
#ifdef CACHING_BSRR
                , p_bsr_registers { reinterpret_cast<volatile uint32_t*>(&a_ports[0]->p_registers->bsrr),
                                    reinterpret_cast<volatile uint32_t*>(&a_ports[1]->p_registers->bsrr) }
                //(std::array{})
#endif
            {
                // GPIO* g = a_map.at(0);
                // ll::gpio::BSRR& bssr = g->p_registers->bsrr;
                // decltype(a_map.at(0))
            }

            // void set_value(std::uint32_t a_value);
            void set_value(std::uint32_t a_value)
            {
                // props to change array to some iterator. This can be non-template
                hkm_assert(a_value < lut.size());
                
                [[maybe_unused]]
                // auto row = lut.at(a_value); // require throw 0ut of range frmt
                auto row = lut[a_value];
                for(size_t i = 0; i<row.size();++i)
                {
                    // write where to do nothing is faster, deterministic and might be cheaper than test (a few
                    // instructions analyze) zero value.
#ifdef CACHING_BSRR
                    // TODO test performance of both cases
                    // one shot test:
                    // 4585 SPS
                    *this->p_bsr_registers[i] = row[i]; // static_cast<ll::gpio::BSRR::Data>(row[i]);
#else
                    // 4577 SPS
                    this->p_ports[i]->p_registers->bsrr = static_cast<ll::gpio::BSRR::Data>(row[i]);
#endif
                }
            }
            

        private:
            const std::array<T,N>& lut; // TODO check allocation of this. Expected to place in flash.
            // std::vector might be a bit more flexible, avoid templatization this class.
            
            const std::array<GPIO*  , 2> p_ports;
            // crutial "cache" to maximalize performance
            // TODO provide data types...
#ifdef CACHING_BSRR
            const
             std::array<volatile uint32_t*, 2> p_bsr_registers;
            //  std::array<ll::gpio::BSRR, 2> p_bsr_registers;
#endif
        };

        void enable(Limited<std::uint32_t, 0, 15> a_id, const Enable_config& a_enable_config, Pin* a_p_pin = nullptr);
        void enable(Limited<std::uint32_t, 0, 15> a_id, const Enable_config& a_enable_config, Bus* a_p_bus);
        void disable(Limited<std::uint32_t, 0, 15> a_id);
        void disable(Pin* p_pin);

    private:
        Out(GPIO* a_p_port)
            : p_port(a_p_port)
        {
        }

        GPIO* p_port;
        friend GPIO;
    };
    class In : private xmcu::Non_copyable
    {
    public:
        class Pin : private xmcu::Non_copyable
        {
        public:
            Pin()
                : p_port(nullptr)
                , id(0xFFu)
            {
            }

            void set_pull(Pull a_pull);

            GPIO::Pull get_pull() const;
            GPIO::Level get_level() const;

            GPIO* get_port() const
            {
                return this->p_port;
            }
            std::uint8_t get_id() const
            {
                return this->id;
            }

        private:
            GPIO* p_port;
            std::uint8_t id;

            friend In;
        };

        void enable(Limited<std::uint32_t, 0, 15> a_id, Pull a_pull, Pin* a_p_pin = nullptr);
        void disable(Limited<std::uint32_t, 0, 15> a_id);
        void disable(Pin* p_pin);

    private:
        In(GPIO* a_p_port)
            : p_port(a_p_port)
        {
        }

        GPIO* p_port;

        friend GPIO;
    };
    class Analog : private xmcu::Non_copyable
    {
    public:
        class Pin : private xmcu::Non_copyable
        {
        public:
            Pin()
                : p_port(nullptr)
                , id(0xFFu)
            {
            }

            void set_pull(Pull a_pull);

            GPIO::Pull get_pull() const;

            GPIO* get_port() const
            {
                return this->p_port;
            }
            std::uint8_t get_id() const
            {
                return this->id;
            }

        private:
            GPIO* p_port;
            std::uint8_t id;

            friend Analog;
        };

        void enable(Limited<std::uint32_t, 0, 15>, Pull a_pull, Pin* a_p_pin = nullptr);
        void disable(Limited<std::uint32_t, 0, 15> a_id);
        void disable(Pin* p_pin);

    private:
        Analog(GPIO* a_p_port)
            : p_port(a_p_port)
        {
        }

        GPIO* p_port;

        friend GPIO;
    };
    class Alternate_function : private xmcu::Non_copyable
    {
    public:
        struct Enable_config
        {
            Type type = various::get_enum_incorrect_value<Type>();
            Pull pull = various::get_enum_incorrect_value<Pull>();
            Speed speed = various::get_enum_incorrect_value<Speed>();
        };

        class Pin : private xmcu::Non_copyable
        {
        public:
            Pin()
                : p_port(nullptr)
                , id(0xFFu)
            {
            }

            void set_type(Type a_type);
            void set_pull(Pull a_pull);
            void set_speed(Speed a_speed);

            GPIO::Type get_type() const;
            GPIO::Pull get_pull() const;
            GPIO::Speed get_speed() const;

            std::uint32_t get_function() const
            {
                return this->function;
            }
            GPIO* get_port() const
            {
                return this->p_port;
            }
            std::uint32_t get_id() const
            {
                return this->id;
            }

        private:
            GPIO* p_port;
            std::uint32_t id;

            std::uint32_t function;

            friend Alternate_function;
        };

        template<typename Periph_t, std::uint32_t periph_id = std::numeric_limits<std::uint32_t>::max()> void
        enable(Limited<std::uint32_t, 0, 15>, const Enable_config& a_enable_config, Pin* a_p_pin = nullptr) = delete;
        void disable(Limited<std::uint32_t, 0, 15> a_id);
        void disable(Pin* p_pin);

    private:
        Alternate_function(GPIO* a_p_port)
            : p_port(a_p_port)
        {
        }

        void enable(Limited<std::uint32_t, 0, 15>,
                    const Enable_config& a_enable_config,
                    std::uint32_t a_function,
                    Pin* a_p_pin);

        GPIO* p_port;

        friend GPIO;
    };

    class Interrupt : private xmcu::Non_copyable
    {
    private:
        template<std::uint8_t> struct H
        {
        };

    public:
        enum class Type : std::uint32_t
        {
            interrupt,
            event
        };

        enum class Trigger_flag : std::uint32_t
        {
            rising = 0x1,
            falling = 0x2,
        };

        struct Id
        {
            static H<0> _0_1;
            static H<1> _2_3;
            static H<2> _4_15;
        };

        struct Callback
        {
            using Function = void (*)(std::uint32_t a_pin, void* a_p_user_data);

            Function function = nullptr;
            void* p_user_data = nullptr;
        };

        Interrupt(Interrupt&&) = default;
        Interrupt& operator=(Interrupt&&) = default;

        Interrupt()
            : idx(std::numeric_limits<decltype(this->idx)>::max())
            , irqn(static_cast<IRQn_Type>(std::numeric_limits<int32_t>::max()))
        {
        }

        Interrupt(H<0>)
            : idx(0)
            , irqn(EXTI0_1_IRQn)
        {
        }
        Interrupt(H<1>)
            : idx(1)
            , irqn(EXTI2_3_IRQn)
        {
        }
        Interrupt(H<2>)
            : idx(2)
            , irqn(EXTI4_15_IRQn)
        {
        }

        ~Interrupt()
        {
            if (0x0 != NVIC_GetEnableIRQ(this->irqn))
            {
                this->disable();
            }
        }

        void enable(const Callback& a_callback, const IRQ_config& a_irq_config);
        void disable();

        void attach(const GPIO& a_port, std::uint32_t a_pin, Trigger_flag a_trigger, Type a_type);
        void attach(const GPIO::In::Pin& a_pin, Trigger_flag a_trigger, Type a_type);
        void attach(const GPIO::Out::Pin& a_pin, Trigger_flag a_trigger, Type a_type);
        void attach(const GPIO::Alternate_function::Pin& a_pin, Trigger_flag a_trigger, Type a_type);

        void deattach(const GPIO& a_port, std::uint32_t a_pin);
        void deattach(const GPIO::In::Pin& a_pin);
        void deattach(const GPIO::Out::Pin& a_pin);
        void deattach(const GPIO::Alternate_function::Pin& a_pin);

    private:
        std::uint32_t idx;
        IRQn_Type irqn;

        friend GPIO;
    };

    struct mco : private xmcu::non_constructible
    {
        enum class Divider : std::uint32_t
        {
            _1 = 0x0u,
            _2 = RCC_CFGR_MCOPRE_0,
            _4 = RCC_CFGR_MCOPRE_1,
            _8 = RCC_CFGR_MCOPRE_0 | RCC_CFGR_MCOPRE_1,
            _16 = RCC_CFGR_MCOPRE_2
        };

        template<typename Clock_t, std::uint32_t clock_id = std::numeric_limits<std::uint32_t>::max()>
        static void enable(Divider a_divider) = delete;
        static void disable();
    };

    struct lsco : private xmcu::non_constructible
    {
        template<typename Clock_t> static void enable() = delete;
        static void disable();
    };

    GPIO()
        : out(nullptr)
        , in(nullptr)
        , analog(nullptr)
        , alternate_function(nullptr)
        , idx(std::numeric_limits<decltype(this->idx)>::max())
        , p_registers(nullptr)
        , flags(0u)
    {
    }

    ~GPIO()
    {
        if (true == this->is_enabled())
        {
            this->disable();
        }
    }

    void enable()
    {
        bit::set(&(this->flags), 31u);
    }

    void disable()
    {
        bit::clear(&(this->flags), 31u);
    }

    bool is_pin_taken(std::uint8_t a_id) const
    {
        return bit::is(this->flags, a_id);
    }

    bool is_enabled() const
    {
        return bit::is(this->flags, 31u);
    }

    bool is_created()
    {
        return std::numeric_limits<decltype(this->idx)>::max() != this->idx && nullptr != this->p_registers;
    }

    explicit operator ll::gpio::Registers*()
    {
        return this->p_registers;
    }

    Out out;
    In in;
    Analog analog;
    Alternate_function alternate_function;

private:
    GPIO(std::uint32_t a_idx, ll::gpio::Registers* a_p_registers)
        : out(this)
        , in(this)
        , analog(this)
        , alternate_function(this)
        , idx(a_idx)
        , p_registers(a_p_registers)
        , flags(0u)
    {
    }

    void take_pin(std::uint8_t a_id)
    {
        bit::set(&(this->flags), a_id);
    }

    void give_pin(std::uint8_t a_id)
    {
        bit::clear(&(this->flags), a_id);
    }

    std::uint32_t idx;
    ll::gpio::Registers* p_registers;

    std::uint32_t flags;

    friend Out;
    friend In;
    friend Analog;
    friend Alternate_function;
    template<typename Periph_t, std::uint32_t id> friend class xmcu::soc::peripheral;
};

template<> void GPIO::mco::enable<clocks::sources::hse>(Divider a_divider);
template<> void GPIO::mco::enable<clocks::sources::hsi16>(Divider a_divider);
template<> void GPIO::mco::enable<clocks::sources::msi>(Divider a_divider);
template<> void GPIO::mco::enable<rcc<system::mcu<1>>>(Divider a_divider);

constexpr GPIO::Interrupt::Trigger_flag operator|(GPIO::Interrupt::Trigger_flag a_f1,
                                                  GPIO::Interrupt::Trigger_flag a_f2)
{
    return static_cast<GPIO::Interrupt::Trigger_flag>(static_cast<std::uint32_t>(a_f1) |
                                                      static_cast<std::uint32_t>(a_f2));
}
constexpr GPIO::Interrupt::Trigger_flag operator&(GPIO::Interrupt::Trigger_flag a_f1,
                                                  GPIO::Interrupt::Trigger_flag a_f2)
{
    return static_cast<GPIO::Interrupt::Trigger_flag>(static_cast<std::uint32_t>(a_f1) &
                                                      static_cast<std::uint32_t>(a_f2));
}
constexpr GPIO::Interrupt::Trigger_flag operator|=(GPIO::Interrupt::Trigger_flag& a_f1,
                                                   GPIO::Interrupt::Trigger_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}
} // namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals

namespace xmcu::soc::st::arm::m0::l0::rm0451 {
#if defined(XMCU_GPIOA_PRESENT)
template<> class rcc<peripherals::GPIO, peripherals::GPIO::A>
{
public:
    static void enable(bool a_enable_in_lp);
    static void disable();
};
#endif
#if defined(XMCU_GPIOB_PRESENT)
template<> class rcc<peripherals::GPIO, peripherals::GPIO::B>
{
public:
    static void enable(bool a_enable_in_lp);
    static void disable();
};
#endif
#if defined(XMCU_GPIOC_PRESENT)
template<> class rcc<peripherals::GPIO, peripherals::GPIO::C>
{
public:
    static void enable(bool a_enable_in_lp);
    static void disable();
};
#endif
#if defined(XMCU_GPIOH_PRESENT)
template<> class rcc<peripherals::GPIO, peripherals::GPIO::H>
{
public:
    static void enable(bool a_enable_in_lp);
    static void disable();
};
#endif
template<>
void peripherals::GPIO::Alternate_function::enable<peripherals::GPIO::mco>(Limited<std::uint32_t, 0, 15> a_id,
                                                                           const Enable_config& a_config,
                                                                           Pin* a_p_pin);
template<>
void peripherals::GPIO::Alternate_function::enable<peripherals::GPIO::lsco>(Limited<std::uint32_t, 0, 15> a_id,
                                                                            const Enable_config& a_config,
                                                                            Pin* a_p_pin);
} // namespace xmcu::soc::st::arm::m0::l0::rm0451

namespace xmcu::soc {
#if defined(XMCU_GPIOA_PRESENT)
template<> class peripheral<st::arm::m0::l0::rm0451::peripherals::GPIO, st::arm::m0::l0::rm0451::peripherals::GPIO::A>
    : private xmcu::non_constructible
{
public:
    static st::arm::m0::l0::rm0451::peripherals::GPIO create()
    {
        namespace l0_peripherals = st::arm::m0::l0::rm0451::peripherals;
        return l0_peripherals::GPIO(0u, l0_peripherals::ll::gpio::registers<l0_peripherals::ll::gpio::A>());
    }
};
#endif

#if defined(XMCU_GPIOB_PRESENT)
template<> class peripheral<st::arm::m0::l0::rm0451::peripherals::GPIO, st::arm::m0::l0::rm0451::peripherals::GPIO::B>
    : private xmcu::non_constructible
{
public:
    static st::arm::m0::l0::rm0451::peripherals::GPIO create()
    {
        namespace l0_peripherals = st::arm::m0::l0::rm0451::peripherals;
        return l0_peripherals::GPIO(1u, l0_peripherals::ll::gpio::registers<l0_peripherals::ll::gpio::B>());
    }
};
#endif

#if defined(XMCU_GPIOC_PRESENT)
template<> class peripheral<st::arm::m0::l0::rm0451::peripherals::GPIO, st::arm::m0::l0::rm0451::peripherals::GPIO::C>
    : private xmcu::non_constructible
{
public:
    static st::arm::m0::l0::rm0451::peripherals::GPIO create()
    {
        namespace l0_peripherals = st::arm::m0::l0::rm0451::peripherals;
        return l0_peripherals::GPIO(2u, l0_peripherals::ll::gpio::registers<l0_peripherals::ll::gpio::C>());
    }
};
#endif

#if defined(XMCU_GPIOH_PRESENT)
// not implemented yet
#endif
} // namespace xmcu::soc