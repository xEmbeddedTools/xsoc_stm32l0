#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// xmcu
#include <rm0451/peripherals/GPIO/base.hpp>
#include <xmcu/Limited.hpp>
#include <xmcu/Non_copyable.hpp>

namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll {
struct gpio : public gpio_base
{
private:
    struct moder_descriptor : private xmcu::non_constructible
    {
        static constexpr std::uint32_t mask = 0x3u;
        static constexpr std::uint32_t shift_multiplier = 2u;

        enum class Flag : std::uint32_t
        {
            input = 0x0u,
            output = 0x1u,
            af = 0x2u,
            analog = 0x3u
        };

        enum class Mask : std::uint32_t
        {
            mask = mask
        };
    };
    struct otyper_descriptor : private xmcu::non_constructible
    {
        static constexpr std::uint32_t mask = 0x1u;
        static constexpr std::uint32_t shift_multiplier = 1u;

        enum class Flag : std::uint32_t
        {
            push_pull = 0x0u,
            open_drain = 0x1u,
        };

        enum class Mask : std::uint32_t
        {
            mask = mask
        };
    };
    struct ospeedr_descriptor : private xmcu::non_constructible
    {
        static constexpr std::uint32_t mask = 0x3u;
        static constexpr std::uint32_t shift_multiplier = 2u;

        enum class Flag : std::uint32_t
        {
            low = 0x0u,
            medium = 0x1u,
            high = 0x2u,
            ultra = 0x3u
        };

        enum class Mask : std::uint32_t
        {
            mask = mask
        };
    };
    struct pupdr_descriptor : private xmcu::non_constructible
    {
        static constexpr std::uint32_t mask = 0x3u;
        static constexpr std::uint32_t shift_multiplier = 2u;

        enum class Flag : std::uint32_t
        {
            none = 0x0u,
            pull_up = 0x1u,
            pull_down = 0x2u,
        };

        enum class Mask : std::uint32_t
        {
            mask = mask
        };
    };
    struct idr_descriptor : private xmcu::non_constructible
    {
        static constexpr std::uint32_t shift_multiplier = 1u;

        enum class Flag : std::uint32_t
        {
            low,
            high
        };
    };
    struct odr_descriptor : private xmcu::non_constructible
    {
        static constexpr std::uint32_t mask = 0x1u;
        static constexpr std::uint32_t shift_multiplier = 1u;

        enum class Flag : std::uint32_t
        {
            low,
            high
        };

        enum class Mask : std::uint32_t
        {
            mask = mask
        };
    };
    struct bsrr_descriptor : private xmcu::non_constructible
    {
        enum class Flag : std::uint32_t
        {
            high = 0x1u,
            low = 0x2u
        };
    };
    struct lckr_descriptor : private xmcu::non_constructible
    {
        static constexpr std::uint32_t mask = 0x1u;
        static constexpr std::uint32_t shift_multiplier = 1u;

        enum class Flag : std::uint32_t
        {
            unlocked = 0x0u,
            lock = 0x01u,
            key = 0x10u
        };

        enum class Mask : std::uint32_t
        {
            mask = mask
        };
    };
    struct afr_descriptor : private xmcu::non_constructible
    {
        static constexpr std::uint32_t mask = 0xFu;
        static constexpr std::uint32_t shift_multiplier = 4u;

        enum class Flag : std::uint32_t
        {
            af0,
            af1,
            af2,
            af3,
            af4,
            af5,
            af6,
            af7,
            af8,
            af9,
            af10,
            af11,
            af12,
            af13,
            af14
        };

        enum class Mask : std::uint32_t
        {
            mask = mask
        };
    };
    struct brr_descriptor : private xmcu::non_constructible
    {
        static constexpr std::uint32_t shift_multiplier = 1u;

        enum class Flag : std::uint32_t
        {
            low = 0x0u,
        };
    };

    /*
     * W - write, R - read, C - clear
     */
    template<typename desc_t> struct Reg_wrc
    {
        enum class Data;

        Reg_wrc(const volatile Reg_wrc& other_a)
            : v(other_a.v)
        {
        }

        void zero()
        {
            this->v = static_cast<Data>(0x0u);
        }

        operator Data() const
        {
            return this->v;
        }

    protected:
        volatile Data v;
    };
    template<typename desc_t> struct Reg_r
    {
        enum class Data;

        Reg_r(const volatile Reg_r& other_a)
            : v(other_a.v)
        {
        }

        operator Data() const
        {
            return this->v;
        }

    protected:
        const volatile Data v;
    };
    template<typename desc_t> struct Reg_wc
    {
        enum class Data;

        Reg_wc(const volatile Reg_wc& other_a)
            : v(other_a.v)
        {
        }

        void zero()
        {
            this->v = 0x0u;
        }

        operator Data() const
        {
            return this->v;
        }

    protected:
        volatile Data v;
    };
    template<typename desc_t> struct Reg_wr
    {
        enum class Data;

        operator Data() const
        {
            return this->v;
        }

    protected:
        volatile Data v;
    };

public:
    struct MODER : public Reg_wrc<moder_descriptor>
    {
        using Flag = moder_descriptor::Flag;
        using Mask = moder_descriptor::Mask;

        using enum Flag;
        using enum Mask;

        using Data = Reg_wrc<moder_descriptor>::Data;

        MODER& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }
    };
    struct OTYPER : public Reg_wrc<otyper_descriptor>
    {
        using Flag = otyper_descriptor::Flag;
        using Mask = otyper_descriptor::Mask;

        using enum Flag;
        using enum Mask;

        using Data = Reg_wrc<otyper_descriptor>::Data;

        OTYPER& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }
    };
    struct OSPEEDR : public Reg_wrc<ospeedr_descriptor>
    {
        using Flag = ospeedr_descriptor::Flag;
        using Mask = ospeedr_descriptor::Mask;

        using enum Flag;
        using enum Mask;

        using Data = Reg_wrc<ospeedr_descriptor>::Data;

        OSPEEDR& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }
    };
    struct PUPDR : public Reg_wrc<pupdr_descriptor>
    {
        using Flag = pupdr_descriptor::Flag;
        using Mask = pupdr_descriptor::Mask;

        using enum Flag;
        using enum Mask;

        using Data = Reg_wrc<pupdr_descriptor>::Data;

        PUPDR& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }
    };
    struct IDR : public Reg_r<idr_descriptor>
    {
        using Flag = idr_descriptor::Flag;

        using enum Flag;

        using Data = Reg_r<idr_descriptor>::Data;
    };
    struct ODR : public Reg_wc<odr_descriptor>
    {
        using Flag = odr_descriptor::Flag;
        using Mask = odr_descriptor::Mask;

        using enum Flag;
        using enum Mask;

        using Data = Reg_wc<odr_descriptor>::Data;

        ODR& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }
    };
    struct BSRR : public Reg_wrc<bsrr_descriptor>
    {
        using Flag = bsrr_descriptor::Flag;

        using enum Flag;

        using Data = Reg_wrc<bsrr_descriptor>::Data;

        BSRR& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }
    };
    struct LCKR : public Reg_wr<lckr_descriptor>
    {
        using Flag = lckr_descriptor::Flag;

        using enum Flag;

        using Data = Reg_wr<lckr_descriptor>::Data;
    };
    struct AFR : public Reg_wrc<afr_descriptor>
    {
        using Flag = afr_descriptor::Flag;
        using Mask = afr_descriptor::Mask;

        using enum Flag;
        using enum Mask;

        using Data = Reg_wrc<afr_descriptor>::Data;

        AFR& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }
    };
    struct BRR : public Reg_wrc<brr_descriptor>
    {
        using Flag = brr_descriptor::Flag;

        using enum Flag;

        using Data = Reg_wrc<brr_descriptor>::Data;

        BRR& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }
    };

    struct Registers : private Non_copyable
    {
        MODER moder;     // port mode register
        OTYPER otyper;   // port output type register
        OSPEEDR ospeedr; // port output speed register
        PUPDR pupdr;     // port pull-up/pull-down register
        IDR idr;         // port input data register
        ODR odr;         // port output data register
        BSRR bsrr;       // port bit set/reset  register
        LCKR lckr;       // port configuration lock register
        AFR afr[2];      // alternate function registers
        BRR brr;         // bit reset register
    };

    template<typename Port_t> [[nodiscard]] static constexpr Registers* registers() = delete;

    // MODER
    friend constexpr gpio::MODER::Data operator<<(gpio::MODER::Flag left_a,
                                                  xmcu::Limited<std::uint32_t, 0u, 15u> pin_a);
    friend constexpr gpio::MODER::Data operator<<(gpio::MODER::Mask mask_a,
                                                  xmcu::Limited<std::uint32_t, 0u, 15u> pin_a);
    friend constexpr gpio::MODER::Flag operator>>(gpio::MODER::Data mask_a,
                                                  xmcu::Limited<std::uint32_t, 0u, 15u> pin_a);

    // OTYPER
    friend constexpr gpio::OTYPER::Data operator<<(gpio::OTYPER::Flag left_a,
                                                   xmcu::Limited<std::uint32_t, 0u, 15u> pin_a);
    friend constexpr gpio::OTYPER::Data operator<<(gpio::OTYPER::Mask left_a,
                                                   xmcu::Limited<std::uint32_t, 0u, 15u> pin_a);
    friend constexpr gpio::OTYPER::Flag operator>>(gpio::OTYPER::Data left_a,
                                                   xmcu::Limited<std::uint32_t, 0u, 15u> pin_a);

    // OSPEEDR
    friend constexpr gpio::OSPEEDR::Data operator<<(gpio::OSPEEDR::Flag left_a,
                                                    xmcu::Limited<std::uint32_t, 0u, 15u> pin_a);
    friend constexpr gpio::OSPEEDR::Data operator<<(gpio::OSPEEDR::Mask left_a,
                                                    xmcu::Limited<std::uint32_t, 0u, 15u> pin_a);
    friend constexpr gpio::OSPEEDR::Flag operator>>(gpio::OSPEEDR::Data left_a,
                                                    xmcu::Limited<std::uint32_t, 0u, 15u> pin_a);

    // PUPDR
    friend constexpr gpio::PUPDR::Data operator<<(gpio::PUPDR::Flag left_a,
                                                  xmcu::Limited<std::uint32_t, 0u, 15u> pin_a);
    friend constexpr gpio::PUPDR::Data operator<<(gpio::PUPDR::Mask left_a,
                                                  xmcu::Limited<std::uint32_t, 0u, 15u> pin_a);
    friend constexpr gpio::PUPDR::Flag operator>>(gpio::PUPDR::Data left_a,
                                                  xmcu::Limited<std::uint32_t, 0u, 15u> pin_a);

    // IDR
    friend constexpr gpio::IDR::Data operator<<(gpio::IDR::Flag left_a, xmcu::Limited<std::uint32_t, 0u, 15u> pin_a);
    friend constexpr gpio::IDR::Flag operator>>(gpio::IDR::Data left_a, xmcu::Limited<std::uint32_t, 0u, 15u> pin_a);

    // ODR
    friend constexpr gpio::ODR::Data operator<<(gpio::ODR::Flag left_a, xmcu::Limited<std::uint32_t, 0u, 15u> pin_a);
    friend constexpr gpio::ODR::Data operator<<(gpio::ODR::Mask left_a, xmcu::Limited<std::uint32_t, 0u, 15u> pin_a);

    // LCKR
    friend constexpr gpio::LCKR::Data operator<<(gpio::LCKR::Flag left_a, xmcu::Limited<std::uint32_t, 0u, 15u> pin_a);

    // AFR
    friend constexpr gpio::AFR::Data operator<<(gpio::AFR::Flag left_a, xmcu::Limited<std::uint32_t, 0u, 15u> pin_a);
    friend constexpr gpio::AFR::Data operator<<(gpio::AFR::Mask left_a, xmcu::Limited<std::uint32_t, 0u, 15u> pin_a);
    friend constexpr gpio::AFR::Flag operator>>(gpio::AFR::Data left_a, xmcu::Limited<std::uint32_t, 0u, 15u> pin_a);

    // BRR
    friend constexpr gpio::BRR::Data operator<<(gpio::BRR::Flag left_a, xmcu::Limited<std::uint32_t, 0u, 15u> pin_a);

#if defined XMCU_GPIOA_PRESENT
    // MODER
    friend constexpr gpio::MODER::Data operator<<(gpio::MODER::Flag left_a, gpio::A pin_a);
    friend constexpr gpio::MODER::Data operator<<(gpio::MODER::Mask left_a, gpio::A pin_a);
    friend constexpr gpio::MODER::Flag operator>>(gpio::MODER::Data left_a, gpio::A pin_a);

    // OTYPER
    friend constexpr gpio::OTYPER::Data operator<<(gpio::OTYPER::Flag left_a, gpio::A pin_a);
    friend constexpr gpio::OTYPER::Data operator<<(gpio::OTYPER::Mask left_a, gpio::A pin_a);
    friend constexpr gpio::OTYPER::Flag operator>>(gpio::OTYPER::Data left_a, gpio::A pin_a);

    // OSPEEDR
    friend constexpr gpio::OSPEEDR::Data operator<<(gpio::OSPEEDR::Flag left_a, gpio::A pin_a);
    friend constexpr gpio::OSPEEDR::Data operator<<(gpio::OSPEEDR::Mask left_a, gpio::A pin_a);
    friend constexpr gpio::OSPEEDR::Flag operator>>(gpio::OSPEEDR::Data left_a, gpio::A pin_a);

    // PUPDR
    friend constexpr gpio::PUPDR::Data operator<<(gpio::PUPDR::Flag left_a, gpio::A pin_a);
    friend constexpr gpio::PUPDR::Data operator<<(gpio::PUPDR::Mask left_a, gpio::A pin_a);
    friend constexpr gpio::PUPDR::Flag operator>>(gpio::PUPDR::Data left_a, gpio::A pin_a);

    // IDR
    friend constexpr gpio::IDR::Data operator<<(gpio::IDR::Flag left_a, gpio::A pin_a);
    friend constexpr gpio::IDR::Data operator&(gpio::IDR::Data left_a, gpio::A pin_a);
    friend constexpr gpio::IDR::Flag operator>>(gpio::IDR::Data left_a, gpio::A pin_a);

    // ODR
    friend constexpr gpio::ODR::Data operator<<(gpio::ODR::Flag left_a, gpio::A pin_a);
    friend constexpr gpio::ODR::Data operator<<(gpio::ODR::Mask left_a, gpio::A pin_a);

    // LCKR
    friend constexpr gpio::LCKR::Data operator<<(gpio::LCKR::Flag left_a, gpio::A pin_a);

    // AFR
    friend constexpr gpio::AFR::Data operator<<(gpio::AFR::Flag left_a, gpio::A pin_a);
    friend constexpr gpio::AFR::Data operator<<(gpio::AFR::Mask left_a, gpio::A pin_a);
    friend constexpr gpio::AFR::Flag operator>>(gpio::AFR::Data left_a, gpio::A pin_a);

    // BRR
    friend constexpr gpio::BRR::Data operator<<(gpio::BRR::Flag left_a, gpio::A pin_a);
#endif
#if defined XMCU_GPIOB_PRESENT
    // MODER
    friend constexpr gpio::MODER::Data operator<<(gpio::MODER::Flag left_a, gpio::B pin_a);
    friend constexpr gpio::MODER::Data operator<<(gpio::MODER::Mask left_a, gpio::B pin_a);
    friend constexpr gpio::MODER::Flag operator>>(gpio::MODER::Data left_a, gpio::B pin_a);

    // OTYPER
    friend constexpr gpio::OTYPER::Data operator<<(gpio::OTYPER::Flag left_a, gpio::B pin_a);
    friend constexpr gpio::OTYPER::Data operator<<(gpio::OTYPER::Mask left_a, gpio::B pin_a);
    friend constexpr gpio::OTYPER::Flag operator>>(gpio::OTYPER::Data left_a, gpio::B pin_a);

    // OSPEEDR
    friend constexpr gpio::OSPEEDR::Data operator<<(gpio::OSPEEDR::Flag left_a, gpio::B pin_a);
    friend constexpr gpio::OSPEEDR::Data operator<<(gpio::OSPEEDR::Mask left_a, gpio::B pin_a);
    friend constexpr gpio::OSPEEDR::Flag operator>>(gpio::OSPEEDR::Data left_a, gpio::B pin_a);

    // PUPDR
    friend constexpr gpio::PUPDR::Data operator<<(gpio::PUPDR::Flag left_a, gpio::B pin_a);
    friend constexpr gpio::PUPDR::Data operator<<(gpio::PUPDR::Mask left_a, gpio::B pin_a);
    friend constexpr gpio::PUPDR::Flag operator>>(gpio::PUPDR::Data left_a, gpio::B pin_a);

    // IDR
    friend constexpr gpio::IDR::Data operator<<(gpio::IDR::Flag left_a, gpio::B pin_a);
    friend constexpr gpio::IDR::Data operator&(gpio::IDR::Data left_a, gpio::B pin_a);
    friend constexpr gpio::IDR::Flag operator>>(gpio::IDR::Data left_a, gpio::B pin_a);

    // ODR
    friend constexpr gpio::ODR::Data operator<<(gpio::ODR::Flag left_a, gpio::B pin_a);
    friend constexpr gpio::ODR::Data operator<<(gpio::ODR::Mask left_a, gpio::B pin_a);

    // LCKR
    friend constexpr gpio::LCKR::Data operator<<(gpio::LCKR::Flag left_a, gpio::B pin_a);

    // AFR
    friend constexpr gpio::AFR::Data operator<<(gpio::AFR::Flag left_a, gpio::B pin_a);
    friend constexpr gpio::AFR::Data operator<<(gpio::AFR::Mask left_a, gpio::B pin_a);
    friend constexpr gpio::AFR::Flag operator>>(gpio::AFR::Data left_a, gpio::B pin_a);

    // BRR
    friend constexpr gpio::BRR::Data operator<<(gpio::BRR::Flag left_a, gpio::B pin_a);
#endif
#if defined XMCU_GPIOC_PRESENT
    // MODER
    friend constexpr gpio::MODER::Data operator<<(gpio::MODER::Flag left_a, gpio::C pin_a);
    friend constexpr gpio::MODER::Data operator<<(gpio::MODER::Mask left_a, gpio::C pin_a);
    friend constexpr gpio::MODER::Flag operator>>(gpio::MODER::Data left_a, gpio::C pin_a);

    // OTYPER
    friend constexpr gpio::OTYPER::Data operator<<(gpio::OTYPER::Flag left_a, gpio::C pin_a);
    friend constexpr gpio::OTYPER::Data operator<<(gpio::OTYPER::Mask left_a, gpio::C pin_a);
    friend constexpr gpio::OTYPER::Flag operator>>(gpio::OTYPER::Data left_a, gpio::C pin_a);

    // OSPEEDR
    friend constexpr gpio::OSPEEDR::Data operator<<(gpio::OSPEEDR::Flag left_a, gpio::C pin_a);
    friend constexpr gpio::OSPEEDR::Data operator<<(gpio::OSPEEDR::Mask left_a, gpio::C pin_a);
    friend constexpr gpio::OSPEEDR::Flag operator>>(gpio::OSPEEDR::Data left_a, gpio::C pin_a);

    // PUPDR
    friend constexpr gpio::PUPDR::Data operator<<(gpio::PUPDR::Flag left_a, gpio::C pin_a);
    friend constexpr gpio::PUPDR::Data operator<<(gpio::PUPDR::Mask left_a, gpio::C pin_a);
    friend constexpr gpio::PUPDR::Flag operator>>(gpio::PUPDR::Data left_a, gpio::C pin_a);

    // IDR
    friend constexpr gpio::IDR::Data operator<<(gpio::IDR::Flag left_a, gpio::C pin_a);
    friend constexpr gpio::IDR::Data operator&(gpio::IDR::Data left_a, gpio::C pin_a);
    friend constexpr gpio::IDR::Flag operator>>(gpio::IDR::Data left_a, gpio::C pin_a);

    // ODR
    friend constexpr gpio::ODR::Data operator<<(gpio::ODR::Flag left_a, gpio::C pin_a);
    friend constexpr gpio::ODR::Data operator<<(gpio::ODR::Mask left_a, gpio::C pin_a);

    // LCKR
    friend constexpr gpio::LCKR::Data operator<<(gpio::LCKR::Flag left_a, gpio::C pin_a);

    // AFR
    friend constexpr gpio::AFR::Data operator<<(gpio::AFR::Flag left_a, gpio::C pin_a);
    friend constexpr gpio::AFR::Data operator<<(gpio::AFR::Mask left_a, gpio::C pin_a);
    friend constexpr gpio::AFR::Flag operator>>(gpio::AFR::Data left_a, gpio::C pin_a);

    // BRR
    friend constexpr gpio::BRR::Data operator<<(gpio::BRR::Flag left_a, gpio::C pin_a);
#endif
};

constexpr bool operator==(std::uint32_t left_a, gpio::MODER::Data right_a)
{
    return left_a == static_cast<std::uint32_t>(right_a);
}
constexpr bool operator==(gpio::MODER::Data left_a, std::uint32_t right_a)
{
    return static_cast<std::uint32_t>(left_a) == right_a;
}
constexpr bool operator!=(std::uint32_t left_a, gpio::MODER::Data right_a)
{
    return left_a != static_cast<std::uint32_t>(right_a);
}
constexpr bool operator!=(gpio::MODER::Data left_a, std::uint32_t right_a)
{
    return static_cast<std::uint32_t>(left_a) != right_a;
}

// MODER
constexpr gpio::MODER::Data operator<<(gpio::MODER::Flag left_a, xmcu::Limited<std::uint32_t, 0u, 15u> pin_a)
{
    return static_cast<gpio::MODER::Data>(static_cast<std::uint32_t>(left_a)
                                          << (pin_a * gpio::moder_descriptor::shift_multiplier));
}
constexpr gpio::MODER::Data operator<<(gpio::MODER::Mask mask_a, xmcu::Limited<std::uint32_t, 0u, 15u> pin_a)
{
    return static_cast<gpio::MODER::Data>(static_cast<std::uint32_t>(mask_a)
                                          << (pin_a * gpio::moder_descriptor::shift_multiplier));
}
constexpr gpio::MODER::Flag operator>>(gpio::MODER::Data mask_a, xmcu::Limited<std::uint32_t, 0u, 15u> pin_a)
{
    return static_cast<gpio::MODER::Flag>(
        (static_cast<std::uint32_t>(mask_a) >> (pin_a * gpio::moder_descriptor::shift_multiplier) &
         gpio::moder_descriptor::mask));
}
constexpr gpio::MODER::Data operator|(gpio::MODER::Data left_a, gpio::MODER::Data right_a)
{
    return static_cast<gpio::MODER::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr gpio::MODER::Data operator&(gpio::MODER::Data left_a, gpio::MODER::Data right_a)
{
    return static_cast<gpio::MODER::Data>((static_cast<std::uint32_t>(left_a) & static_cast<std::uint32_t>(right_a)));
}
constexpr gpio::MODER::Flag operator~(gpio::MODER::Flag flag_a)
{
    return static_cast<gpio::MODER::Flag>(~static_cast<std::uint32_t>(flag_a));
}
constexpr gpio::MODER::Mask operator~(gpio::MODER::Mask flag_a)
{
    return static_cast<gpio::MODER::Mask>(~static_cast<std::uint32_t>(flag_a));
}
constexpr gpio::MODER::Data operator~(gpio::MODER::Data flag_a)
{
    return static_cast<gpio::MODER::Data>(~static_cast<std::uint32_t>(flag_a));
}
constexpr gpio::MODER::Data operator^(gpio::MODER::Data left_a, std::uint32_t right_a)
{
    return static_cast<gpio::MODER::Data>(static_cast<std::uint32_t>(left_a) ^ right_a);
}

// OTYPER
constexpr bool operator==(std::uint32_t left_a, gpio::OTYPER::Data right_a)
{
    return left_a == static_cast<std::uint32_t>(right_a);
}
constexpr bool operator==(gpio::OTYPER::Data left_a, std::uint32_t right_a)
{
    return static_cast<std::uint32_t>(left_a) == right_a;
}
constexpr bool operator!=(std::uint32_t left_a, gpio::OTYPER::Data right_a)
{
    return left_a != static_cast<std::uint32_t>(right_a);
}
constexpr bool operator!=(gpio::OTYPER::Data left_a, std::uint32_t right_a)
{
    return static_cast<std::uint32_t>(left_a) != right_a;
}
constexpr gpio::OTYPER::Data operator<<(gpio::OTYPER::Flag left_a, xmcu::Limited<std::uint32_t, 0u, 15u> pin_a)
{
    return static_cast<gpio::OTYPER::Data>(static_cast<std::uint32_t>(left_a)
                                           << (pin_a * gpio::otyper_descriptor::shift_multiplier));
}
constexpr gpio::OTYPER::Data operator<<(gpio::OTYPER::Mask left_a, xmcu::Limited<std::uint32_t, 0u, 15u> pin_a)
{
    return static_cast<gpio::OTYPER::Data>(static_cast<std::uint32_t>(left_a)
                                           << (pin_a * gpio::otyper_descriptor::shift_multiplier));
}
constexpr gpio::OTYPER::Flag operator>>(gpio::OTYPER::Data left_a, xmcu::Limited<std::uint32_t, 0u, 15u> pin_a)
{
    return static_cast<gpio::OTYPER::Flag>(
        (static_cast<std::uint32_t>(left_a) >> (pin_a * gpio::otyper_descriptor::shift_multiplier) &
         gpio::otyper_descriptor::mask));
}
constexpr gpio::OTYPER::Data operator|(gpio::OTYPER::Data left_a, gpio::OTYPER::Data right_a)
{
    return static_cast<gpio::OTYPER::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr gpio::OTYPER::Data operator&(gpio::OTYPER::Data left_a, gpio::OTYPER::Data right_a)
{
    return static_cast<gpio::OTYPER::Data>((static_cast<std::uint32_t>(left_a) & static_cast<std::uint32_t>(right_a)));
}
constexpr gpio::OTYPER::Flag operator~(gpio::OTYPER::Flag flag_a)
{
    return static_cast<gpio::OTYPER::Flag>(~static_cast<std::uint32_t>(flag_a));
}
constexpr gpio::OTYPER::Mask operator~(gpio::OTYPER::Mask flag_a)
{
    return static_cast<gpio::OTYPER::Mask>(~static_cast<std::uint32_t>(flag_a));
}
constexpr gpio::OTYPER::Data operator~(gpio::OTYPER::Data flag_a)
{
    return static_cast<gpio::OTYPER::Data>(~static_cast<std::uint32_t>(flag_a));
}
constexpr gpio::OTYPER::Data operator^(gpio::OTYPER::Data left_a, std::uint32_t right_a)
{
    return static_cast<gpio::OTYPER::Data>(static_cast<std::uint32_t>(left_a) ^ right_a);
}

// OSPEEDR
constexpr bool operator==(std::uint32_t left_a, gpio::OSPEEDR::Data right_a)
{
    return left_a == static_cast<std::uint32_t>(right_a);
}
constexpr bool operator==(gpio::OSPEEDR::Data left_a, std::uint32_t right_a)
{
    return static_cast<std::uint32_t>(left_a) == right_a;
}
constexpr bool operator!=(std::uint32_t left_a, gpio::OSPEEDR::Data right_a)
{
    return left_a != static_cast<std::uint32_t>(right_a);
}
constexpr bool operator!=(gpio::OSPEEDR::Data left_a, std::uint32_t right_a)
{
    return static_cast<std::uint32_t>(left_a) != right_a;
}
constexpr gpio::OSPEEDR::Data operator<<(gpio::OSPEEDR::Flag left_a, xmcu::Limited<std::uint32_t, 0u, 15u> pin_a)
{
    return static_cast<gpio::OSPEEDR::Data>(static_cast<std::uint32_t>(left_a)
                                            << (pin_a * gpio::ospeedr_descriptor::shift_multiplier));
}
constexpr gpio::OSPEEDR::Data operator<<(gpio::OSPEEDR::Mask left_a, xmcu::Limited<std::uint32_t, 0u, 15u> pin_a)
{
    return static_cast<gpio::OSPEEDR::Data>(static_cast<std::uint32_t>(left_a)
                                            << (pin_a * gpio::ospeedr_descriptor::shift_multiplier));
}
constexpr gpio::OSPEEDR::Flag operator>>(gpio::OSPEEDR::Data left_a, xmcu::Limited<std::uint32_t, 0u, 15u> pin_a)
{
    return static_cast<gpio::OSPEEDR::Flag>(
        (static_cast<std::uint32_t>(left_a) >> (pin_a * gpio::ospeedr_descriptor::shift_multiplier) &
         gpio::ospeedr_descriptor::mask));
}
constexpr gpio::OSPEEDR::Data operator|(gpio::OSPEEDR::Data left_a, gpio::OSPEEDR::Data right_a)
{
    return static_cast<gpio::OSPEEDR::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr gpio::OSPEEDR::Data operator&(gpio::OSPEEDR::Data left_a, gpio::OSPEEDR::Data right_a)
{
    return static_cast<gpio::OSPEEDR::Data>((static_cast<std::uint32_t>(left_a) & static_cast<std::uint32_t>(right_a)));
}
constexpr gpio::OSPEEDR::Flag operator~(gpio::OSPEEDR::Flag flag_a)
{
    return static_cast<gpio::OSPEEDR::Flag>(~static_cast<std::uint32_t>(flag_a));
}
constexpr gpio::OSPEEDR::Mask operator~(gpio::OSPEEDR::Mask flag_a)
{
    return static_cast<gpio::OSPEEDR::Mask>(~static_cast<std::uint32_t>(flag_a));
}
constexpr gpio::OSPEEDR::Data operator~(gpio::OSPEEDR::Data flag_a)
{
    return static_cast<gpio::OSPEEDR::Data>(~static_cast<std::uint32_t>(flag_a));
}
constexpr gpio::OSPEEDR::Data operator^(gpio::OSPEEDR::Data left_a, std::uint32_t right_a)
{
    return static_cast<gpio::OSPEEDR::Data>(static_cast<std::uint32_t>(left_a) ^ right_a);
}

// PUPDR
constexpr bool operator==(std::uint32_t left_a, gpio::PUPDR::Data right_a)
{
    return left_a == static_cast<std::uint32_t>(right_a);
}
constexpr bool operator==(gpio::PUPDR::Data left_a, std::uint32_t right_a)
{
    return static_cast<std::uint32_t>(left_a) == right_a;
}
constexpr bool operator!=(std::uint32_t left_a, gpio::PUPDR::Data right_a)
{
    return left_a != static_cast<std::uint32_t>(right_a);
}
constexpr bool operator!=(gpio::PUPDR::Data left_a, std::uint32_t right_a)
{
    return static_cast<std::uint32_t>(left_a) != right_a;
}
constexpr gpio::PUPDR::Data operator<<(gpio::PUPDR::Flag left_a, xmcu::Limited<std::uint32_t, 0u, 15u> pin_a)
{
    return static_cast<gpio::PUPDR::Data>(static_cast<std::uint32_t>(left_a)
                                          << (pin_a * gpio::pupdr_descriptor::shift_multiplier));
}
constexpr gpio::PUPDR::Data operator<<(gpio::PUPDR::Mask left_a, xmcu::Limited<std::uint32_t, 0u, 15u> pin_a)
{
    return static_cast<gpio::PUPDR::Data>(static_cast<std::uint32_t>(left_a)
                                          << (pin_a * gpio::pupdr_descriptor::shift_multiplier));
}
constexpr gpio::PUPDR::Flag operator>>(gpio::PUPDR::Data left_a, xmcu::Limited<std::uint32_t, 0u, 15u> pin_a)
{
    return static_cast<gpio::PUPDR::Flag>(
        (static_cast<std::uint32_t>(left_a) >> (pin_a * gpio::otyper_descriptor::shift_multiplier) &
         gpio::otyper_descriptor::mask));
}
constexpr gpio::PUPDR::Data operator|(gpio::PUPDR::Data left_a, gpio::PUPDR::Data right_a)
{
    return static_cast<gpio::PUPDR::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr gpio::PUPDR::Data operator&(gpio::PUPDR::Data left_a, gpio::PUPDR::Data right_a)
{
    return static_cast<gpio::PUPDR::Data>((static_cast<std::uint32_t>(left_a) & static_cast<std::uint32_t>(right_a)));
}
constexpr gpio::PUPDR::Flag operator~(gpio::PUPDR::Flag flag_a)
{
    return static_cast<gpio::PUPDR::Flag>(~static_cast<std::uint32_t>(flag_a));
}
constexpr gpio::PUPDR::Mask operator~(gpio::PUPDR::Mask flag_a)
{
    return static_cast<gpio::PUPDR::Mask>(~static_cast<std::uint32_t>(flag_a));
}
constexpr gpio::PUPDR::Data operator~(gpio::PUPDR::Data flag_a)
{
    return static_cast<gpio::PUPDR::Data>(~static_cast<std::uint32_t>(flag_a));
}
constexpr gpio::PUPDR::Data operator^(gpio::PUPDR::Data left_a, std::uint32_t right_a)
{
    return static_cast<gpio::PUPDR::Data>(static_cast<std::uint32_t>(left_a) ^ right_a);
}

// IDR
constexpr bool operator==(std::uint32_t left_a, gpio::IDR::Data right_a)
{
    return left_a == static_cast<std::uint32_t>(right_a);
}
constexpr bool operator==(gpio::IDR::Data left_a, std::uint32_t right_a)
{
    return static_cast<std::uint32_t>(left_a) == right_a;
}
constexpr bool operator!=(std::uint32_t left_a, gpio::IDR::Data right_a)
{
    return left_a != static_cast<std::uint32_t>(right_a);
}
constexpr bool operator!=(gpio::IDR::Data left_a, std::uint32_t right_a)
{
    return static_cast<std::uint32_t>(left_a) != right_a;
}
constexpr gpio::IDR::Data operator<<(gpio::IDR::Flag left_a, xmcu::Limited<std::uint32_t, 0u, 15u> pin_a)
{
    return static_cast<gpio::IDR::Data>(static_cast<std::uint32_t>(left_a)
                                        << (pin_a * gpio::idr_descriptor::shift_multiplier));
}
constexpr gpio::IDR::Flag operator>>(gpio::IDR::Data left_a, xmcu::Limited<std::uint32_t, 0u, 15u> pin_a)
{
    return static_cast<gpio::IDR::Flag>(
        (static_cast<std::uint32_t>(left_a) >> (pin_a * gpio::idr_descriptor::shift_multiplier) & 0x1u));
}
constexpr gpio::IDR::Data operator&(gpio::IDR::Data left_a, xmcu::Limited<std::uint32_t, 0u, 0xFFFFu> pins_a)
{
    return static_cast<gpio::IDR::Data>((static_cast<std::uint32_t>(left_a) & (pins_a)));
}
constexpr gpio::IDR::Data operator|(gpio::IDR::Data left_a, gpio::IDR::Data right_a)
{
    return static_cast<gpio::IDR::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr gpio::IDR::Data operator&(gpio::IDR::Data left_a, gpio::IDR::Data right_a)
{
    return static_cast<gpio::IDR::Data>((static_cast<std::uint32_t>(left_a) & static_cast<std::uint32_t>(right_a)));
}
constexpr gpio::IDR::Flag operator~(gpio::IDR::Flag flag_a)
{
    return static_cast<gpio::IDR::Flag>(~static_cast<std::uint32_t>(flag_a));
}
constexpr gpio::IDR::Data operator~(gpio::IDR::Data flag_a)
{
    return static_cast<gpio::IDR::Data>(~static_cast<std::uint32_t>(flag_a));
}
constexpr gpio::IDR::Data operator^(gpio::IDR::Data left_a, std::uint32_t right_a)
{
    return static_cast<gpio::IDR::Data>(static_cast<std::uint32_t>(left_a) ^ right_a);
}

// ODR
constexpr bool operator==(std::uint32_t left_a, gpio::ODR::Data right_a)
{
    return left_a == static_cast<std::uint32_t>(right_a);
}
constexpr bool operator==(gpio::ODR::Data left_a, std::uint32_t right_a)
{
    return static_cast<std::uint32_t>(left_a) == right_a;
}
constexpr bool operator!=(std::uint32_t left_a, gpio::ODR::Data right_a)
{
    return left_a != static_cast<std::uint32_t>(right_a);
}
constexpr bool operator!=(gpio::ODR::Data left_a, std::uint32_t right_a)
{
    return static_cast<std::uint32_t>(left_a) != right_a;
}
constexpr gpio::ODR::Data operator<<(gpio::ODR::Flag left_a, xmcu::Limited<std::uint32_t, 0u, 15u> pin_a)
{
    return static_cast<gpio::ODR::Data>(static_cast<std::uint32_t>(left_a)
                                        << (pin_a * gpio::odr_descriptor::shift_multiplier));
}
constexpr gpio::ODR::Data operator<<(gpio::ODR::Mask left_a, xmcu::Limited<std::uint32_t, 0u, 15u> pin_a)
{
    return static_cast<gpio::ODR::Data>(static_cast<std::uint32_t>(left_a)
                                        << (pin_a * gpio::odr_descriptor::shift_multiplier));
}
constexpr gpio::ODR::Data operator|(gpio::ODR::Data left_a, gpio::ODR::Data right_a)
{
    return static_cast<gpio::ODR::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr gpio::ODR::Data operator&(gpio::ODR::Data left_a, gpio::ODR::Data right_a)
{
    return static_cast<gpio::ODR::Data>((static_cast<std::uint32_t>(left_a) & static_cast<std::uint32_t>(right_a)));
}
constexpr gpio::ODR::Flag operator~(gpio::ODR::Flag flag_a)
{
    return static_cast<gpio::ODR::Flag>(~static_cast<std::uint32_t>(flag_a));
}
constexpr gpio::ODR::Mask operator~(gpio::ODR::Mask flag_a)
{
    return static_cast<gpio::ODR::Mask>(~static_cast<std::uint32_t>(flag_a));
}
constexpr gpio::ODR::Data operator~(gpio::ODR::Data flag_a)
{
    return static_cast<gpio::ODR::Data>(~static_cast<std::uint32_t>(flag_a));
}
constexpr gpio::ODR::Data operator^(gpio::ODR::Data left_a, std::uint32_t right_a)
{
    return static_cast<gpio::ODR::Data>(static_cast<std::uint32_t>(left_a) ^ right_a);
}

// BSRR
constexpr bool operator==(std::uint32_t left_a, gpio::BSRR::Data right_a)
{
    return left_a == static_cast<std::uint32_t>(right_a);
}
constexpr bool operator==(gpio::BSRR::Data left_a, std::uint32_t right_a)
{
    return static_cast<std::uint32_t>(left_a) == right_a;
}
constexpr bool operator!=(std::uint32_t left_a, gpio::BSRR::Data right_a)
{
    return left_a != static_cast<std::uint32_t>(right_a);
}
constexpr bool operator!=(gpio::BSRR::Data left_a, std::uint32_t right_a)
{
    return static_cast<std::uint32_t>(left_a) != right_a;
}
constexpr gpio::BSRR::Data operator<<(gpio::BSRR::Flag left_a, xmcu::Limited<std::uint32_t, 0u, 15u> pin_a)
{
    return static_cast<gpio::BSRR::Data>(0x1u << (pin_a * static_cast<std::uint32_t>(left_a)));
}
constexpr gpio::BSRR::Data operator|(gpio::BSRR::Data left_a, gpio::BSRR::Data right_a)
{
    return static_cast<gpio::BSRR::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr gpio::BSRR::Data operator&(gpio::BSRR::Data left_a, gpio::BSRR::Data right_a)
{
    return static_cast<gpio::BSRR::Data>((static_cast<std::uint32_t>(left_a) & static_cast<std::uint32_t>(right_a)));
}
constexpr gpio::BSRR::Flag operator~(gpio::BSRR::Flag flag_a)
{
    return static_cast<gpio::BSRR::Flag>(~static_cast<std::uint32_t>(flag_a));
}
constexpr gpio::BSRR::Data operator~(gpio::BSRR::Data flag_a)
{
    return static_cast<gpio::BSRR::Data>(~static_cast<std::uint32_t>(flag_a));
}
constexpr gpio::BSRR::Data operator^(gpio::BSRR::Data left_a, std::uint32_t right_a)
{
    return static_cast<gpio::BSRR::Data>(static_cast<std::uint32_t>(left_a) ^ right_a);
}

// LCKR
constexpr bool operator==(std::uint32_t left_a, gpio::LCKR::Data right_a)
{
    return left_a == static_cast<std::uint32_t>(right_a);
}
constexpr bool operator==(gpio::LCKR::Data left_a, std::uint32_t right_a)
{
    return static_cast<std::uint32_t>(left_a) == right_a;
}
constexpr bool operator==(gpio::LCKR::Data left_a, gpio::LCKR::Flag right_a)
{
    return static_cast<std::uint32_t>(left_a) == static_cast<std::uint32_t>(right_a);
}
constexpr bool operator==(gpio::LCKR::Flag left_a, gpio::LCKR::Data right_a)
{
    return static_cast<std::uint32_t>(left_a) == static_cast<std::uint32_t>(right_a);
}
constexpr bool operator!=(std::uint32_t left_a, gpio::LCKR::Data right_a)
{
    return left_a != static_cast<std::uint32_t>(right_a);
}
constexpr bool operator!=(gpio::LCKR::Data left_a, std::uint32_t right_a)
{
    return static_cast<std::uint32_t>(left_a) != right_a;
}
constexpr bool operator!=(gpio::LCKR::Data left_a, gpio::LCKR::Flag right_a)
{
    return static_cast<std::uint32_t>(left_a) != static_cast<std::uint32_t>(right_a);
}
constexpr bool operator!=(gpio::LCKR::Flag left_a, gpio::LCKR::Data right_a)
{
    return static_cast<std::uint32_t>(left_a) != static_cast<std::uint32_t>(right_a);
}
constexpr gpio::LCKR::Data operator<<(gpio::LCKR::Flag left_a, xmcu::Limited<std::uint32_t, 0u, 15u> pin_a)
{
    return static_cast<gpio::LCKR::Data>(static_cast<std::uint32_t>(left_a)
                                         << (pin_a * gpio::lckr_descriptor::shift_multiplier));
}
constexpr gpio::LCKR::Data operator|(gpio::LCKR::Data left_a, gpio::LCKR::Data right_a)
{
    return static_cast<gpio::LCKR::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr gpio::LCKR::Data operator|(gpio::LCKR::Flag left_a, gpio::LCKR::Data right_a)
{
    hkm_assert(left_a == gpio::LCKR::key);
    return static_cast<gpio::LCKR::Data>((static_cast<std::uint32_t>(left_a) & static_cast<std::uint32_t>(right_a)));
}
constexpr gpio::LCKR::Data operator|(gpio::LCKR::Data left_a, gpio::LCKR::Flag right_a)
{
    hkm_assert(left_a == gpio::LCKR::key);
    return static_cast<gpio::LCKR::Data>((static_cast<std::uint32_t>(left_a) & static_cast<std::uint32_t>(right_a)));
}
constexpr gpio::LCKR::Data operator&(gpio::LCKR::Data left_a, gpio::LCKR::Data right_a)
{
    return static_cast<gpio::LCKR::Data>((static_cast<std::uint32_t>(left_a) & static_cast<std::uint32_t>(right_a)));
}
constexpr gpio::LCKR::Flag operator~(gpio::LCKR::Flag flag_a)
{
    return static_cast<gpio::LCKR::Flag>(~static_cast<std::uint32_t>(flag_a));
}
constexpr gpio::LCKR::Data operator~(gpio::LCKR::Data flag_a)
{
    return static_cast<gpio::LCKR::Data>(~static_cast<std::uint32_t>(flag_a));
}

// AFR
constexpr bool operator==(std::uint32_t left_a, gpio::AFR::Data right_a)
{
    return left_a == static_cast<std::uint32_t>(right_a);
}
constexpr bool operator==(gpio::AFR::Data left_a, std::uint32_t right_a)
{
    return static_cast<std::uint32_t>(left_a) == right_a;
}
constexpr bool operator!=(std::uint32_t left_a, gpio::AFR::Data right_a)
{
    return left_a != static_cast<std::uint32_t>(right_a);
}
constexpr bool operator!=(gpio::AFR::Data left_a, std::uint32_t right_a)
{
    return static_cast<std::uint32_t>(left_a) != right_a;
}
constexpr gpio::AFR::Data operator<<(gpio::AFR::Flag left_a, xmcu::Limited<std::uint32_t, 0u, 15u> pin_a)
{
    return static_cast<gpio::AFR::Data>(static_cast<std::uint32_t>(left_a)
                                        << (pin_a * gpio::afr_descriptor::shift_multiplier));
}
constexpr gpio::AFR::Data operator<<(gpio::AFR::Mask left_a, xmcu::Limited<std::uint32_t, 0u, 15u> pin_a)
{
    return static_cast<gpio::AFR::Data>(static_cast<std::uint32_t>(left_a)
                                        << (pin_a * gpio::afr_descriptor::shift_multiplier));
}
constexpr gpio::AFR::Flag operator>>(gpio::AFR::Data left_a, xmcu::Limited<std::uint32_t, 0u, 15u> pin_a)
{
    return static_cast<gpio::AFR::Flag>(
        (static_cast<std::uint32_t>(left_a) >> (pin_a * gpio::afr_descriptor::shift_multiplier) &
         gpio::afr_descriptor::mask));
}
constexpr gpio::AFR::Data operator|(gpio::AFR::Data left_a, gpio::AFR::Data right_a)
{
    return static_cast<gpio::AFR::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr gpio::AFR::Data operator&(gpio::AFR::Data left_a, gpio::AFR::Data right_a)
{
    return static_cast<gpio::AFR::Data>((static_cast<std::uint32_t>(left_a) & static_cast<std::uint32_t>(right_a)));
}
constexpr gpio::AFR::Flag operator~(gpio::AFR::Flag flag_a)
{
    return static_cast<gpio::AFR::Flag>(~static_cast<std::uint32_t>(flag_a));
}
constexpr gpio::AFR::Mask operator~(gpio::AFR::Mask flag_a)
{
    return static_cast<gpio::AFR::Mask>(~static_cast<std::uint32_t>(flag_a));
}
constexpr gpio::AFR::Data operator~(gpio::AFR::Data flag_a)
{
    return static_cast<gpio::AFR::Data>(~static_cast<std::uint32_t>(flag_a));
}
constexpr gpio::AFR::Data operator^(gpio::AFR::Data left_a, std::uint32_t right_a)
{
    return static_cast<gpio::AFR::Data>(static_cast<std::uint32_t>(left_a) ^ right_a);
}

// BRR
constexpr bool operator==(std::uint32_t left_a, gpio::BRR::Data right_a)
{
    return left_a == static_cast<std::uint32_t>(right_a);
}
constexpr bool operator==(gpio::BRR::Data left_a, std::uint32_t right_a)
{
    return static_cast<std::uint32_t>(left_a) == right_a;
}
constexpr bool operator!=(std::uint32_t left_a, gpio::BRR::Data right_a)
{
    return left_a != static_cast<std::uint32_t>(right_a);
}
constexpr bool operator!=(gpio::BRR::Data left_a, std::uint32_t right_a)
{
    return static_cast<std::uint32_t>(left_a) != right_a;
}
constexpr gpio::BRR::Data operator<<(gpio::BRR::Flag left_a, xmcu::Limited<std::uint32_t, 0u, 15u> pin_a)
{
    return static_cast<gpio::BRR::Data>(static_cast<std::uint32_t>(left_a)
                                        << (pin_a * gpio::brr_descriptor::shift_multiplier));
}
constexpr gpio::BRR::Data operator|(gpio::BRR::Data left_a, gpio::BRR::Data right_a)
{
    return static_cast<gpio::BRR::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr gpio::BRR::Data operator&(gpio::BRR::Data left_a, gpio::BRR::Data right_a)
{
    return static_cast<gpio::BRR::Data>((static_cast<std::uint32_t>(left_a) & static_cast<std::uint32_t>(right_a)));
}
constexpr gpio::BRR::Flag operator~(gpio::BRR::Flag flag_a)
{
    return static_cast<gpio::BRR::Flag>(~static_cast<std::uint32_t>(flag_a));
}
constexpr gpio::BRR::Data operator~(gpio::BRR::Data flag_a)
{
    return static_cast<gpio::BRR::Data>(~static_cast<std::uint32_t>(flag_a));
}
constexpr gpio::BRR::Data operator^(gpio::BRR::Data left_a, std::uint32_t right_a)
{
    return static_cast<gpio::BRR::Data>(static_cast<std::uint32_t>(left_a) ^ right_a);
}

#if defined XMCU_GPIOA_PRESENT
template<> [[nodiscard]] constexpr gpio::Registers* gpio::registers<gpio::A>()
{
    return reinterpret_cast<gpio::Registers*>(GPIOA_BASE);
}

// MODER
constexpr gpio::MODER::Data operator<<(gpio::MODER::Flag left_a, gpio::A pin_a)
{
    return static_cast<gpio::MODER::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::moder_descriptor::shift_multiplier)));
}
constexpr gpio::MODER::Data operator<<(gpio::MODER::Mask left_a, gpio::A pin_a)
{
    return static_cast<gpio::MODER::Data>(
        static_cast<std::uint32_t>(left_a)
        << (static_cast<std::uint32_t>(pin_a) * gpio::moder_descriptor::shift_multiplier));
}
constexpr gpio::MODER::Flag operator>>(gpio::MODER::Data left_a, gpio::A pin_a)
{
    return static_cast<gpio::MODER::Flag>(
        (static_cast<std::uint32_t>(left_a) >>
             (static_cast<std::uint32_t>(pin_a) * gpio::moder_descriptor::shift_multiplier) &
         gpio::moder_descriptor::mask));
}

// OTYPER
constexpr gpio::OTYPER::Data operator<<(gpio::OTYPER::Flag left_a, gpio::A pin_a)
{
    return static_cast<gpio::OTYPER::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::otyper_descriptor::shift_multiplier)));
}
constexpr gpio::OTYPER::Data operator<<(gpio::OTYPER::Mask left_a, gpio::A pin_a)
{
    return static_cast<gpio::OTYPER::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::otyper_descriptor::shift_multiplier)));
}
constexpr gpio::OTYPER::Flag operator>>(gpio::OTYPER::Data left_a, gpio::A pin_a)
{
    return static_cast<gpio::OTYPER::Flag>(
        (static_cast<std::uint32_t>(left_a) >>
             (static_cast<std::uint32_t>(pin_a) * gpio::otyper_descriptor::shift_multiplier) &
         gpio::otyper_descriptor::mask));
}

// OSPEEDR
constexpr gpio::OSPEEDR::Data operator<<(gpio::OSPEEDR::Flag left_a, gpio::A pin_a)
{
    return static_cast<gpio::OSPEEDR::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::ospeedr_descriptor::shift_multiplier)));
}
constexpr gpio::OSPEEDR::Data operator<<(gpio::OSPEEDR::Mask left_a, gpio::A pin_a)
{
    return static_cast<gpio::OSPEEDR::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::ospeedr_descriptor::shift_multiplier)));
}
constexpr gpio::OSPEEDR::Flag operator>>(gpio::OSPEEDR::Data left_a, gpio::A pin_a)
{
    return static_cast<gpio::OSPEEDR::Flag>(
        (static_cast<std::uint32_t>(left_a) >>
             (static_cast<std::uint32_t>(pin_a) * gpio::ospeedr_descriptor::shift_multiplier) &
         gpio::ospeedr_descriptor::mask));
}

// PUPDR
constexpr gpio::PUPDR::Data operator<<(gpio::PUPDR::Flag left_a, gpio::A pin_a)
{
    return static_cast<gpio::PUPDR::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::pupdr_descriptor::shift_multiplier)));
}
constexpr gpio::PUPDR::Data operator<<(gpio::PUPDR::Mask left_a, gpio::A pin_a)
{
    return static_cast<gpio::PUPDR::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::pupdr_descriptor::shift_multiplier)));
}
constexpr gpio::PUPDR::Flag operator>>(gpio::PUPDR::Data left_a, gpio::A pin_a)
{
    return static_cast<gpio::PUPDR::Flag>(
        (static_cast<std::uint32_t>(left_a) >>
             (static_cast<std::uint32_t>(pin_a) * gpio::pupdr_descriptor::shift_multiplier) &
         gpio::pupdr_descriptor::mask));
}

// IDR
constexpr gpio::IDR::Data operator<<(gpio::IDR::Flag left_a, gpio::A pin_a)
{
    return static_cast<gpio::IDR::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::idr_descriptor::shift_multiplier)));
}
constexpr gpio::IDR::Data operator&(gpio::IDR::Data left_a, gpio::A pin_a)
{
    return left_a & (0x1u << (static_cast<std::uint32_t>(pin_a) * gpio::idr_descriptor::shift_multiplier));
}
constexpr gpio::IDR::Flag operator>>(gpio::IDR::Data left_a, gpio::A pin_a)
{
    return static_cast<gpio::IDR::Flag>(
        (static_cast<std::uint32_t>(left_a) >>
             (static_cast<std::uint32_t>(pin_a) * gpio::idr_descriptor::shift_multiplier) &
         0x1u));
}

// ODR
constexpr gpio::ODR::Data operator<<(gpio::ODR::Flag left_a, gpio::A pin_a)
{
    return static_cast<gpio::ODR::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::odr_descriptor::shift_multiplier)));
}
constexpr gpio::ODR::Data operator<<(gpio::ODR::Mask left_a, gpio::A pin_a)
{
    return static_cast<gpio::ODR::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::odr_descriptor::shift_multiplier)));
}

// BSRR
constexpr gpio::BSRR::Data operator<<(gpio::BSRR::Flag left_a, gpio::A pin_a)
{
    return static_cast<gpio::BSRR::Data>(0x1u
                                         << (static_cast<std::uint32_t>(pin_a) * static_cast<std::uint32_t>(left_a)));
}

// LCKR
constexpr gpio::LCKR::Data operator<<(gpio::LCKR::Flag left_a, gpio::A pin_a)
{
    return static_cast<gpio::LCKR::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::lckr_descriptor::shift_multiplier)));
}

// AFR
constexpr gpio::AFR::Data operator<<(gpio::AFR::Flag left_a, gpio::A pin_a)
{
    return static_cast<gpio::AFR::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::afr_descriptor::shift_multiplier)));
}
constexpr gpio::AFR::Data operator<<(gpio::AFR::Mask left_a, gpio::A pin_a)
{
    return static_cast<gpio::AFR::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::afr_descriptor::shift_multiplier)));
}
constexpr gpio::AFR::Flag operator>>(gpio::AFR::Data left_a, gpio::A pin_a)
{
    return static_cast<gpio::AFR::Flag>(
        (static_cast<std::uint32_t>(left_a) >>
             (static_cast<std::uint32_t>(pin_a) * gpio::afr_descriptor::shift_multiplier) &
         gpio::afr_descriptor::mask));
}

// BRR
constexpr gpio::BRR::Data operator<<(gpio::BRR::Flag left_a, gpio::A pin_a)
{
    return static_cast<gpio::BRR::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::afr_descriptor::shift_multiplier)));
}
#endif
#if defined XMCU_GPIOB_PRESENT
template<> [[nodiscard]] constexpr gpio::Registers* gpio::registers<gpio::B>()
{
    return reinterpret_cast<gpio::Registers*>(GPIOB_BASE);
}

// MODER
constexpr gpio::MODER::Data operator<<(gpio::MODER::Flag left_a, gpio::B pin_a)
{
    return static_cast<gpio::MODER::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::moder_descriptor::shift_multiplier)));
}
constexpr gpio::MODER::Data operator<<(gpio::MODER::Mask left_a, gpio::B pin_a)
{
    return static_cast<gpio::MODER::Data>(
        static_cast<std::uint32_t>(left_a)
        << (static_cast<std::uint32_t>(pin_a) * gpio::moder_descriptor::shift_multiplier));
}
constexpr gpio::MODER::Flag operator>>(gpio::MODER::Data left_a, gpio::B pin_a)
{
    return static_cast<gpio::MODER::Flag>(
        (static_cast<std::uint32_t>(left_a) >>
             (static_cast<std::uint32_t>(pin_a) * gpio::moder_descriptor::shift_multiplier) &
         gpio::moder_descriptor::mask));
}

// OTYPER
constexpr gpio::OTYPER::Data operator<<(gpio::OTYPER::Flag left_a, gpio::B pin_a)
{
    return static_cast<gpio::OTYPER::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::otyper_descriptor::shift_multiplier)));
}
constexpr gpio::OTYPER::Data operator<<(gpio::OTYPER::Mask left_a, gpio::B pin_a)
{
    return static_cast<gpio::OTYPER::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::otyper_descriptor::shift_multiplier)));
}
constexpr gpio::OTYPER::Flag operator>>(gpio::OTYPER::Data left_a, gpio::B pin_a)
{
    return static_cast<gpio::OTYPER::Flag>(
        (static_cast<std::uint32_t>(left_a) >>
             (static_cast<std::uint32_t>(pin_a) * gpio::otyper_descriptor::shift_multiplier) &
         gpio::otyper_descriptor::mask));
}

// OSPEEDR
constexpr gpio::OSPEEDR::Data operator<<(gpio::OSPEEDR::Flag left_a, gpio::B pin_a)
{
    return static_cast<gpio::OSPEEDR::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::ospeedr_descriptor::shift_multiplier)));
}
constexpr gpio::OSPEEDR::Data operator<<(gpio::OSPEEDR::Mask left_a, gpio::B pin_a)
{
    return static_cast<gpio::OSPEEDR::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::ospeedr_descriptor::shift_multiplier)));
}
constexpr gpio::OSPEEDR::Flag operator>>(gpio::OSPEEDR::Data left_a, gpio::B pin_a)
{
    return static_cast<gpio::OSPEEDR::Flag>(
        (static_cast<std::uint32_t>(left_a) >>
             (static_cast<std::uint32_t>(pin_a) * gpio::ospeedr_descriptor::shift_multiplier) &
         gpio::ospeedr_descriptor::mask));
}

// PUPDR
constexpr gpio::PUPDR::Data operator<<(gpio::PUPDR::Flag left_a, gpio::B pin_a)
{
    return static_cast<gpio::PUPDR::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::pupdr_descriptor::shift_multiplier)));
}
constexpr gpio::PUPDR::Data operator<<(gpio::PUPDR::Mask left_a, gpio::B pin_a)
{
    return static_cast<gpio::PUPDR::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::pupdr_descriptor::shift_multiplier)));
}
constexpr gpio::PUPDR::Flag operator>>(gpio::PUPDR::Data left_a, gpio::B pin_a)
{
    return static_cast<gpio::PUPDR::Flag>(
        (static_cast<std::uint32_t>(left_a) >>
             (static_cast<std::uint32_t>(pin_a) * gpio::pupdr_descriptor::shift_multiplier) &
         gpio::pupdr_descriptor::mask));
}

// IDR
constexpr gpio::IDR::Data operator<<(gpio::IDR::Flag left_a, gpio::B pin_a)
{
    return static_cast<gpio::IDR::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::idr_descriptor::shift_multiplier)));
}
constexpr gpio::IDR::Data operator&(gpio::IDR::Data left_a, gpio::B pin_a)
{
    return left_a & (0x1u << (static_cast<std::uint32_t>(pin_a) * gpio::idr_descriptor::shift_multiplier));
}
constexpr gpio::IDR::Flag operator>>(gpio::IDR::Data left_a, gpio::B pin_a)
{
    return static_cast<gpio::IDR::Flag>(
        (static_cast<std::uint32_t>(left_a) >>
             (static_cast<std::uint32_t>(pin_a) * gpio::idr_descriptor::shift_multiplier) &
         0x1u));
}

// ODR
constexpr gpio::ODR::Data operator<<(gpio::ODR::Flag left_a, gpio::B pin_a)
{
    return static_cast<gpio::ODR::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::odr_descriptor::shift_multiplier)));
}
constexpr gpio::ODR::Data operator<<(gpio::ODR::Mask left_a, gpio::B pin_a)
{
    return static_cast<gpio::ODR::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::odr_descriptor::shift_multiplier)));
}

// BSRR
constexpr gpio::BSRR::Data operator<<(gpio::BSRR::Flag left_a, gpio::B pin_a)
{
    return static_cast<gpio::BSRR::Data>(0x1u
                                         << (static_cast<std::uint32_t>(pin_a) * static_cast<std::uint32_t>(left_a)));
}

// LCKR
constexpr gpio::LCKR::Data operator<<(gpio::LCKR::Flag left_a, gpio::B pin_a)
{
    return static_cast<gpio::LCKR::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::lckr_descriptor::shift_multiplier)));
}

// AFR
constexpr gpio::AFR::Data operator<<(gpio::AFR::Flag left_a, gpio::B pin_a)
{
    return static_cast<gpio::AFR::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::afr_descriptor::shift_multiplier)));
}
constexpr gpio::AFR::Data operator<<(gpio::AFR::Mask left_a, gpio::B pin_a)
{
    return static_cast<gpio::AFR::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::afr_descriptor::shift_multiplier)));
}
constexpr gpio::AFR::Flag operator>>(gpio::AFR::Data left_a, gpio::B pin_a)
{
    return static_cast<gpio::AFR::Flag>(
        (static_cast<std::uint32_t>(left_a) >>
             (static_cast<std::uint32_t>(pin_a) * gpio::afr_descriptor::shift_multiplier) &
         gpio::afr_descriptor::mask));
}

// BRR
constexpr gpio::BRR::Data operator<<(gpio::BRR::Flag left_a, gpio::B pin_a)
{
    return static_cast<gpio::BRR::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::afr_descriptor::shift_multiplier)));
}
#endif
#if defined XMCU_GPIOC_PRESENT
template<> [[nodiscard]] constexpr gpio::Registers* gpio::registers<gpio::C>()
{
    return reinterpret_cast<gpio::Registers*>(GPIOC_BASE);
}

// MODER
constexpr gpio::MODER::Data operator<<(gpio::MODER::Flag left_a, gpio::C pin_a)
{
    return static_cast<gpio::MODER::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::moder_descriptor::shift_multiplier)));
}
constexpr gpio::MODER::Data operator<<(gpio::MODER::Mask left_a, gpio::C pin_a)
{
    return static_cast<gpio::MODER::Data>(
        static_cast<std::uint32_t>(left_a)
        << (static_cast<std::uint32_t>(pin_a) * gpio::moder_descriptor::shift_multiplier));
}
constexpr gpio::MODER::Flag operator>>(gpio::MODER::Data left_a, gpio::C pin_a)
{
    return static_cast<gpio::MODER::Flag>(
        (static_cast<std::uint32_t>(left_a) >>
             (static_cast<std::uint32_t>(pin_a) * gpio::moder_descriptor::shift_multiplier) &
         gpio::moder_descriptor::mask));
}

// OTYPER
constexpr gpio::OTYPER::Data operator<<(gpio::OTYPER::Flag left_a, gpio::C pin_a)
{
    return static_cast<gpio::OTYPER::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::otyper_descriptor::shift_multiplier)));
}
constexpr gpio::OTYPER::Data operator<<(gpio::OTYPER::Mask left_a, gpio::C pin_a)
{
    return static_cast<gpio::OTYPER::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::otyper_descriptor::shift_multiplier)));
}
constexpr gpio::OTYPER::Flag operator>>(gpio::OTYPER::Data left_a, gpio::C pin_a)
{
    return static_cast<gpio::OTYPER::Flag>(
        (static_cast<std::uint32_t>(left_a) >>
             (static_cast<std::uint32_t>(pin_a) * gpio::otyper_descriptor::shift_multiplier) &
         gpio::otyper_descriptor::mask));
}

// OSPEEDR
constexpr gpio::OSPEEDR::Data operator<<(gpio::OSPEEDR::Flag left_a, gpio::C pin_a)
{
    return static_cast<gpio::OSPEEDR::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::ospeedr_descriptor::shift_multiplier)));
}
constexpr gpio::OSPEEDR::Data operator<<(gpio::OSPEEDR::Mask left_a, gpio::C pin_a)
{
    return static_cast<gpio::OSPEEDR::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::ospeedr_descriptor::shift_multiplier)));
}
constexpr gpio::OSPEEDR::Flag operator>>(gpio::OSPEEDR::Data left_a, gpio::C pin_a)
{
    return static_cast<gpio::OSPEEDR::Flag>(
        (static_cast<std::uint32_t>(left_a) >>
             (static_cast<std::uint32_t>(pin_a) * gpio::ospeedr_descriptor::shift_multiplier) &
         gpio::ospeedr_descriptor::mask));
}

// PUPDR
constexpr gpio::PUPDR::Data operator<<(gpio::PUPDR::Flag left_a, gpio::C pin_a)
{
    return static_cast<gpio::PUPDR::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::pupdr_descriptor::shift_multiplier)));
}
constexpr gpio::PUPDR::Data operator<<(gpio::PUPDR::Mask left_a, gpio::C pin_a)
{
    return static_cast<gpio::PUPDR::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::pupdr_descriptor::shift_multiplier)));
}
constexpr gpio::PUPDR::Flag operator>>(gpio::PUPDR::Data left_a, gpio::C pin_a)
{
    return static_cast<gpio::PUPDR::Flag>(
        (static_cast<std::uint32_t>(left_a) >>
             (static_cast<std::uint32_t>(pin_a) * gpio::pupdr_descriptor::shift_multiplier) &
         gpio::pupdr_descriptor::mask));
}

// IDR
constexpr gpio::IDR::Data operator<<(gpio::IDR::Flag left_a, gpio::C pin_a)
{
    return static_cast<gpio::IDR::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::idr_descriptor::shift_multiplier)));
}
constexpr gpio::IDR::Data operator&(gpio::IDR::Data left_a, gpio::C pin_a)
{
    return left_a & (0x1u << (static_cast<std::uint32_t>(pin_a) * gpio::idr_descriptor::shift_multiplier));
}
constexpr gpio::IDR::Flag operator>>(gpio::IDR::Data left_a, gpio::C pin_a)
{
    return static_cast<gpio::IDR::Flag>(
        (static_cast<std::uint32_t>(left_a) >>
             (static_cast<std::uint32_t>(pin_a) * gpio::idr_descriptor::shift_multiplier) &
         0x1u));
}

// ODR
constexpr gpio::ODR::Data operator<<(gpio::ODR::Flag left_a, gpio::C pin_a)
{
    return static_cast<gpio::ODR::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::odr_descriptor::shift_multiplier)));
}
constexpr gpio::ODR::Data operator<<(gpio::ODR::Mask left_a, gpio::C pin_a)
{
    return static_cast<gpio::ODR::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::odr_descriptor::shift_multiplier)));
}

// BSRR
constexpr gpio::BSRR::Data operator<<(gpio::BSRR::Flag left_a, gpio::C pin_a)
{
    return static_cast<gpio::BSRR::Data>(0x1u
                                         << (static_cast<std::uint32_t>(pin_a) * static_cast<std::uint32_t>(left_a)));
}

// LCKR
constexpr gpio::LCKR::Data operator<<(gpio::LCKR::Flag left_a, gpio::C pin_a)
{
    return static_cast<gpio::LCKR::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::lckr_descriptor::shift_multiplier)));
}

// AFR
constexpr gpio::AFR::Data operator<<(gpio::AFR::Flag left_a, gpio::C pin_a)
{
    return static_cast<gpio::AFR::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::afr_descriptor::shift_multiplier)));
}
constexpr gpio::AFR::Data operator<<(gpio::AFR::Mask left_a, gpio::C pin_a)
{
    return static_cast<gpio::AFR::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::afr_descriptor::shift_multiplier)));
}
constexpr gpio::AFR::Flag operator>>(gpio::AFR::Data left_a, gpio::C pin_a)
{
    return static_cast<gpio::AFR::Flag>(
        (static_cast<std::uint32_t>(left_a) >>
             (static_cast<std::uint32_t>(pin_a) * gpio::afr_descriptor::shift_multiplier) &
         gpio::afr_descriptor::mask));
}

// BRR
constexpr gpio::BRR::Data operator<<(gpio::BRR::Flag left_a, gpio::C pin_a)
{
    return static_cast<gpio::BRR::Data>(
        (static_cast<std::uint32_t>(left_a)
         << (static_cast<std::uint32_t>(pin_a) * gpio::afr_descriptor::shift_multiplier)));
}
#endif
} // namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll

namespace xmcu {
template<> [[nodiscard]] constexpr bool
bit::is<soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::IDR, xmcu::Limited<std::uint32_t, 0u, 15u>>(
    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::IDR a_register,
    xmcu::Limited<std::uint32_t, 0u, 15u> a_index)
{
    using ll_gpio = soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio;

    const std::uint32_t flag = 0x1u << static_cast<std::uint32_t>(a_index);
    return flag == (static_cast<ll_gpio::IDR::Data>(a_register) & flag);
}

template<> [[nodiscard]] constexpr bool
bit::is_any<soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::IDR, xmcu::Limited<std::uint32_t, 0x0u, 0xFFFFu>>(
    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::IDR a_register,
    xmcu::Limited<std::uint32_t, 0x0u, 0xFFFFu> a_mask)
{
    using ll_gpio = soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio;

    return 0u != static_cast<std::uint32_t>(static_cast<ll_gpio::IDR::Data>(a_register) & a_mask);
}

template<> constexpr void
bit::set<soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::ODR, xmcu::Limited<std::uint32_t, 0u, 15u>>(
    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::ODR* a_p_register,
    xmcu::Limited<std::uint32_t, 0u, 15u> a_index)
{
    using ll_gpio = soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio;

    (*a_p_register) = (*a_p_register) | static_cast<ll_gpio::ODR::Data>(0x1u << static_cast<std::uint32_t>(a_index));
}

template<> constexpr void
bit::clear<soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::ODR, xmcu::Limited<std::uint32_t, 0u, 15u>>(
    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::ODR* a_p_register,
    xmcu::Limited<std::uint32_t, 0u, 15u> a_index)
{
    using ll_gpio = soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio;

    (*a_p_register) = (*a_p_register) & static_cast<ll_gpio::ODR::Data>(~(0x1u << static_cast<std::uint32_t>(a_index)));
}

template<> constexpr void
bit::toggle<soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::ODR, xmcu::Limited<std::uint32_t, 0u, 15u>>(
    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::ODR* a_p_register,
    xmcu::Limited<std::uint32_t, 0u, 15u> a_index)
{
    (*a_p_register) = (*a_p_register) ^ (0x1u << static_cast<std::uint32_t>(a_index));
}

#if defined XMCU_GPIOA_PRESENT
template<> [[nodiscard]] constexpr bool bit::is<soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::IDR,
                                                soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::A>(
    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::IDR a_register,
    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::A a_index)
{
    using ll_gpio = soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio;

    const std::uint32_t flag = 0x1u << static_cast<std::uint32_t>(a_index);
    return flag == (static_cast<ll_gpio::IDR::Data>(a_register) & flag);
}

template<> [[nodiscard]] constexpr bool bit::is_any<soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::IDR,
                                                    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::A>(
    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::IDR a_register,
    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::A a_mask)
{
    return 0u != static_cast<std::uint32_t>(a_register & a_mask);
}

template<> constexpr void bit::set<soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::ODR,
                                   soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::A>(
    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::ODR* a_p_register,
    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::A a_index)
{
    using ll_gpio = soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio;

    (*a_p_register) = (*a_p_register) | static_cast<ll_gpio::ODR::Data>(0x1u << static_cast<std::uint32_t>(a_index));
}

template<> constexpr void bit::clear<soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::ODR,
                                     soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::A>(
    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::ODR* a_p_register,
    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::A a_index)
{
    using ll_gpio = soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio;

    (*a_p_register) = (*a_p_register) & static_cast<ll_gpio::ODR::Data>(~(0x1u << static_cast<std::uint32_t>(a_index)));
}

template<> constexpr void bit::toggle<soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::ODR,
                                      soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::A>(
    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::ODR* a_p_register,
    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::A a_index)
{
    (*a_p_register) = (*a_p_register) ^ (0x1u << static_cast<std::uint32_t>(a_index));
}
#endif
#if defined XMCU_GPIOB_PRESENT
template<> [[nodiscard]] constexpr bool bit::is<soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::IDR,
                                                soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::B>(
    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::IDR a_register,
    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::B a_index)
{
    using ll_gpio = soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio;

    const std::uint32_t flag = 0x1u << static_cast<std::uint32_t>(a_index);
    return flag == (static_cast<ll_gpio::IDR::Data>(a_register) & flag);
}

template<> [[nodiscard]] constexpr bool bit::is_any<soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::IDR,
                                                    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::B>(
    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::IDR a_register,
    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::B a_mask)
{
    return 0u != static_cast<std::uint32_t>(a_register & a_mask);
}

template<> constexpr void bit::set<soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::ODR,
                                   soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::B>(
    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::ODR* a_p_register,
    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::B a_index)
{
    using ll_gpio = soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio;

    (*a_p_register) = (*a_p_register) | static_cast<ll_gpio::ODR::Data>(0x1u << static_cast<std::uint32_t>(a_index));
}

template<> constexpr void bit::clear<soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::ODR,
                                     soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::B>(
    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::ODR* a_p_register,
    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::B a_index)
{
    using ll_gpio = soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio;

    (*a_p_register) = (*a_p_register) & static_cast<ll_gpio::ODR::Data>(~(0x1u << static_cast<std::uint32_t>(a_index)));
}

template<> constexpr void bit::toggle<soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::ODR,
                                      soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::B>(
    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::ODR* a_p_register,
    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::B a_index)
{
    (*a_p_register) = (*a_p_register) ^ (0x1u << static_cast<std::uint32_t>(a_index));
}
#endif
#if defined XMCU_GPIOC_PRESENT
template<> [[nodiscard]] constexpr bool bit::is<soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::IDR,
                                                soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::C>(
    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::IDR a_register,
    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::C a_index)
{
    using ll_gpio = soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio;

    const std::uint32_t flag = 0x1u << static_cast<std::uint32_t>(a_index);
    return flag == (static_cast<ll_gpio::IDR::Data>(a_register) & flag);
}

template<> [[nodiscard]] constexpr bool bit::is_any<soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::IDR,
                                                    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::C>(
    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::IDR a_register,
    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::C a_mask)
{
    return 0u != static_cast<std::uint32_t>(a_register & a_mask);
}

template<> constexpr void bit::set<soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::ODR,
                                   soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::C>(
    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::ODR* a_p_register,
    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::C a_index)
{
    using ll_gpio = soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio;

    (*a_p_register) = (*a_p_register) | static_cast<ll_gpio::ODR::Data>(0x1u << static_cast<std::uint32_t>(a_index));
}

template<> constexpr void bit::clear<soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::ODR,
                                     soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::C>(
    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::ODR* a_p_register,
    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::C a_index)
{
    using ll_gpio = soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio;

    (*a_p_register) = (*a_p_register) & static_cast<ll_gpio::ODR::Data>(~(0x1u << static_cast<std::uint32_t>(a_index)));
}

template<> constexpr void bit::toggle<soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::ODR,
                                      soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::C>(
    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::ODR* a_p_register,
    soc::st::arm::m0::l0::rm0451::peripherals::ll::gpio::C a_index)
{
    (*a_p_register) = (*a_p_register) ^ (0x1u << static_cast<std::uint32_t>(a_index));
}
#endif
} // namespace xmcu