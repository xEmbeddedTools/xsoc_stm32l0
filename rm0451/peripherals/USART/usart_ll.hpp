#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// soc
#include <rm0451/peripherals/USART/base.hpp>

// xmcu
#include <xmcu/Limited.hpp>
#include <xmcu/Non_copyable.hpp>
#include <xmcu/non_constructible.hpp>

// CMSIS
#include <stm32l0xx.h>

// std
#include <cstdint>
#include <type_traits>

#define XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(ReturnEnumType, LeftEnumType, RightEnumType)            \
                                                                                                         \
    constexpr ReturnEnumType operator|(LeftEnumType left_a, RightEnumType right_a) noexcept              \
    {                                                                                                    \
        return static_cast<ReturnEnumType>(static_cast<std::underlying_type_t<LeftEnumType>>(left_a) |   \
                                           static_cast<std::underlying_type_t<RightEnumType>>(right_a)); \
    }                                                                                                    \
                                                                                                         \
    constexpr ReturnEnumType operator&(LeftEnumType left_a, RightEnumType right_a) noexcept              \
    {                                                                                                    \
        return static_cast<ReturnEnumType>(static_cast<std::underlying_type_t<LeftEnumType>>(left_a) &   \
                                           static_cast<std::underlying_type_t<RightEnumType>>(right_a)); \
    }                                                                                                    \
                                                                                                         \
    constexpr ReturnEnumType operator^(LeftEnumType left_a, RightEnumType right_a) noexcept              \
    {                                                                                                    \
        return static_cast<ReturnEnumType>(static_cast<std::underlying_type_t<LeftEnumType>>(left_a) ^   \
                                           static_cast<std::underlying_type_t<RightEnumType>>(right_a)); \
    }

#define XSOC_USART_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(EnumType)            \
    constexpr EnumType& operator|=(EnumType& left_a, EnumType right_a) noexcept \
    {                                                                           \
        left_a = static_cast<EnumType>(left_a | right_a);                       \
        return left_a;                                                          \
    }                                                                           \
                                                                                \
    constexpr EnumType& operator&=(EnumType& left_a, EnumType right_a) noexcept \
    {                                                                           \
        left_a = static_cast<EnumType>(left_a & right_a);                       \
        return left_a;                                                          \
    }                                                                           \
                                                                                \
    constexpr EnumType& operator^=(EnumType& left_a, EnumType right_a) noexcept \
    {                                                                           \
        left_a = static_cast<EnumType>(left_a ^ right_a);                       \
        return left_a;                                                          \
    }

#define XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS(EnumType)   \
    constexpr EnumType operator~(EnumType right_a) noexcept        \
    {                                                              \
        using Type = std::underlying_type_t<EnumType>;             \
        return static_cast<EnumType>(~static_cast<Type>(right_a)); \
    }                                                              \
    constexpr bool operator!(EnumType right_a) noexcept            \
    {                                                              \
        using Type = std::underlying_type_t<EnumType>;             \
        return static_cast<Type>(right_a) == 0;                    \
    }

#define XSOC_USART_LL_GENERATE_COMPARISON_OPERATORS(LeftEnumType, RightEnumType)            \
    constexpr bool operator==(LeftEnumType left_a, RightEnumType right_a)                   \
    {                                                                                       \
        return (static_cast<std::uint32_t>(left_a) == static_cast<std::uint32_t>(right_a)); \
    }                                                                                       \
    constexpr bool operator==(RightEnumType left_a, LeftEnumType right_a)                   \
    {                                                                                       \
        return (static_cast<std::uint32_t>(left_a) == static_cast<std::uint32_t>(right_a)); \
    }                                                                                       \
    constexpr bool operator!=(LeftEnumType left_a, RightEnumType right_a)                   \
    {                                                                                       \
        return false == (left_a == right_a);                                                \
    }                                                                                       \
    constexpr bool operator!=(RightEnumType left_a, LeftEnumType right_a)                   \
    {                                                                                       \
        return false == (left_a == right_a);                                                \
    }

namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll {
struct usart : public usart_base
{
    struct CR1
    {
        enum class Flag : std::uint32_t
        {
            none = 0x0u,
            ue = USART_CR1_UE,
            uesm = USART_CR1_UESM,
            re = USART_CR1_RE,
            te = USART_CR1_TE,
            rxneie = USART_CR1_RXNEIE,
            tcie = USART_CR1_TCIE,
            txeie = USART_CR1_TXEIE,
            peie = USART_CR1_PEIE,
            ps = USART_CR1_PS,
            pce = USART_CR1_PCE,
            wake = USART_CR1_WAKE,
            m0 = USART_CR1_M0,
            mme = USART_CR1_MME,
            cmie = USART_CR1_CMIE,
            idleie = USART_CR1_IDLEIE,
            over8 = USART_CR1_OVER8,
            rtoie = USART_CR1_RTOIE,
            eobie = USART_CR1_EOBIE,
            m1 = USART_CR1_M1
        };
        enum class Shift_5 : std::uint32_t
        {
            deat = USART_CR1_DEAT_Pos,
            dedt = USART_CR1_DEDT_Pos
        };

        using enum Flag;
        using enum Shift_5;

        enum class Data : std::uint32_t;

        CR1& operator=(Flag value_a)
        {
            this->v = static_cast<Data>(value_a);
            return *this;
        }
        CR1& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }

        operator Data() const
        {
            return this->v;
        }

    private:
        volatile Data v;
    };
    struct CR2
    {
        enum class Flag : std::uint32_t
        {
            none = 0x0u,
            addm7 = USART_CR2_ADDM7,
            lbdl = USART_CR2_LBDL,
            lbdie = USART_CR2_LBDIE,
            lbcl = USART_CR2_LBCL,
            cpha = USART_CR2_CPHA,
            cpol = USART_CR2_CPOL,
            clken = USART_CR2_CLKEN,
            linen = USART_CR2_LINEN,
            swap = USART_CR2_SWAP,
            rxinv = USART_CR2_RXINV,
            txinv = USART_CR2_TXINV,
            datainv = USART_CR2_DATAINV,
            msbfirst = USART_CR2_MSBFIRST,
            abren = USART_CR2_ABREN,
            roten = USART_CR2_RTOEN
        };
        enum class Shift_2 : std::uint32_t
        {
            stop = USART_CR2_STOP_Pos,
            abrmod = USART_CR2_ABRMODE_Pos
        };
        enum class Shift_8 : std::uint32_t
        {
            add = USART_CR2_ADD_Pos
        };

        using enum Flag;
        using enum Shift_2;
        using enum Shift_8;

        enum class Data : std::uint32_t;

        CR2& operator=(Flag value_a)
        {
            this->v = static_cast<Data>(value_a);
            return *this;
        }
        CR2& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }

        operator Data() const
        {
            return this->v;
        }

    private:
        volatile Data v;
    };
    struct CR3
    {
        enum class Flag : std::uint32_t
        {
            none = 0x0u,
            eie = USART_CR3_EIE,
            iren = USART_CR3_IREN,
            irlp = USART_CR3_IRLP,
            hdsel = USART_CR3_HDSEL,
            nack = USART_CR3_NACK,
            scen = USART_CR3_SCEN,
            dmar = USART_CR3_DMAR,
            dmat = USART_CR3_DMAT,
            rtse = USART_CR3_RTSE,
            ctse = USART_CR3_CTSE,
            ctsie = USART_CR3_CTSIE,
            onebit = USART_CR3_ONEBIT,
            ovrdis = USART_CR3_OVRDIS,
            ddre = USART_CR3_DDRE,
            dem = USART_CR3_DEM,
            dep = USART_CR3_DEP,
            wufie = USART_CR3_WUFIE,
            ucesm = USART_CR3_UCESM,
            tcbgtie = 1u << 24u /*USART_CR3_TCBGTIE*/
        };
        enum class Shift_2 : std::uint32_t
        {
            wus = USART_CR3_WUS_Pos
        };
        enum class Shift_3 : std::uint32_t
        {
            scarcnt = USART_CR3_SCARCNT_Pos
        };

        using enum Flag;
        using enum Shift_2;
        using enum Shift_3;

        enum class Data : std::uint32_t;

        CR3& operator=(Flag value_a)
        {
            this->v = static_cast<Data>(value_a);
            return *this;
        }
        CR3& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }

        operator Data() const
        {
            return this->v;
        }

    private:
        volatile Data v;
    };
    struct BRR
    {
        BRR& operator=(std::uint32_t value_a)
        {
            this->v = value_a;
            return *this;
        }

    private:
        volatile std::uint32_t v;
    };
    struct GTPR
    {
        enum class Shift_8 : std::uint32_t
        {
            psc = USART_GTPR_PSC_Pos,
            gt = USART_GTPR_GT_Pos
        };

        using enum Shift_8;

        enum class Data : std::uint32_t;

        GTPR& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }

    private:
        volatile Data v;
    };
    struct RTOR
    {
        enum class Shift_23 : std::uint32_t
        {
            rto = USART_RTOR_RTO_Pos
        };

        enum class Shift_8 : std::uint32_t
        {
            blen = USART_RTOR_BLEN_Pos
        };

        using enum Shift_23;
        using enum Shift_8;

        enum class Data : std::uint32_t;

        RTOR& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }

    private:
        volatile Data v;
    };
    struct RQR
    {
        enum class Flag : std::uint32_t
        {
            abrrq = USART_RQR_ABRRQ,
            sbkrq = USART_RQR_SBKRQ,
            mmrq = USART_RQR_MMRQ,
            rxfrq = USART_RQR_RXFRQ,
            txfrq = USART_RQR_TXFRQ
        };

        using enum Flag;

        enum class Data : std::uint32_t;

        RQR& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }

        RQR& operator=(Flag value_a)
        {
            this->v = static_cast<Data>(value_a);
            return *this;
        }

    private:
        volatile Data v;
    };
    struct ISR
    {
        enum class Flag : std::uint32_t
        {
            none = 0x0u,
            pe = USART_ISR_PE,
            fe = USART_ISR_FE,
            ne = USART_ISR_NE,
            ore = USART_ISR_ORE,
            idle = USART_ISR_IDLE,
            rxne = USART_ISR_RXNE,
            tc = USART_ISR_TC,
            txe = USART_ISR_TXE,
            lbdf = USART_ISR_LBDF,
            ctsif = USART_ISR_CTSIF,
            cts = USART_ISR_CTS,
            rtof = USART_ISR_RTOF,
            eobf = USART_ISR_EOBF,
            abre = USART_ISR_ABRE,
            abrf = USART_ISR_ABRF,
            busy = USART_ISR_BUSY,
            cmf = USART_ISR_CMF,
            sbkf = USART_ISR_SBKF,
            rwu = USART_ISR_RWU,
            wuf = USART_ISR_WUF,
            teack = USART_ISR_TEACK,
            reack = USART_ISR_REACK,
            tcbgt = 0x1u << 25u /*USART_ISR_TCBGT*/
        };

        using enum Flag;

        enum class Data : std::uint32_t;

        ISR()
            : v(static_cast<Data>(0x0u))
        {
        }

        operator Data() const
        {
            return this->v;
        }

    private:
        const volatile Data v;
    };
    struct ICR
    {
        enum class Flag : std::uint32_t
        {
            none = 0x0u,
            pecf = USART_ICR_PECF,
            fecf = USART_ICR_FECF,
            ncf = USART_ICR_NCF,
            orecf = USART_ICR_ORECF,
            idlecf = USART_ICR_IDLECF,
            tccf = USART_ICR_TCCF,
            tcbgtcf = 0x1u << 7u /*USART_ICR_TCBGTCF*/,
            lbdcf = USART_ICR_LBDCF,
            ctscf = USART_ICR_CTSCF,
            rtocf = USART_ICR_RTOCF,
            eobcf = USART_ICR_EOBCF,
            cmcf = USART_ICR_CMCF,
            wucf = USART_ICR_WUCF
        };

        using enum Flag;

        enum class Data : std::uint32_t;

        ICR& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }

        ICR& operator=(Flag value_a)
        {
            this->v = static_cast<Data>(value_a);
            return *this;
        }

        operator Data() const
        {
            return this->v;
        }

    private:
        volatile Data v;
    };
    struct RDR
    {
        enum class Data : std::uint32_t;

        RDR()
            : v(static_cast<Data>(0x0u))
        {
        }

        constexpr operator std::uint32_t() const
        {
            return static_cast<std::uint32_t>(this->v);
        }

    private:
        const volatile Data v;
    };
    struct TDR
    {
        enum class Data : std::uint32_t;

        TDR(Limited<std::uint32_t, 0u, 0x1FF> value_a)
            : v(static_cast<Data>(value_a.get()))
        {
        }

        TDR& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }
        TDR& operator=(std::uint32_t value_a)
        {
            this->v = static_cast<Data>(value_a);
            return *this;
        }

    private:
        volatile Data v;
    };

    struct Registers : private xmcu::Non_copyable
    {
        CR1 cr1;   // control register 1
        CR2 cr2;   // control register 2
        CR3 cr3;   // control register 3
        BRR brr;   // baudrate register
        GTPR gtpr; // guard time and prescaler register
        RTOR rtor; // receiver timeout register
        RQR rqr;   // request register
        ISR isr;   // interrupt and status register
        ICR icr;   // interrupt flag clear register
        RDR rdr;   // receive data register
        TDR tdr;   // transmit data register
    };

    template<typename Port> [[nodiscard]] static constexpr Registers* registers() = delete;
};

#if defined XMCU_USART2_PRESENT
template<> [[nodiscard]] constexpr usart::Registers* usart::registers<usart::_2>()
{
    return reinterpret_cast<usart::Registers*>(USART2_BASE);
}
#endif

// CR1
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::CR1::Data, usart::CR1::Flag, usart::CR1::Flag);
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::CR1::Data, usart::CR1::Data, usart::CR1::Data);
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::CR1::Data, usart::CR1::Data, usart::CR1::Flag);
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::CR1::Data, usart::CR1::Flag, usart::CR1::Data);

XSOC_USART_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(usart::CR1::Flag);
XSOC_USART_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(usart::CR1::Data);

XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS(usart::CR1::Flag);
XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS(usart::CR1::Data);
XSOC_USART_LL_GENERATE_COMPARISON_OPERATORS(usart::CR1::Flag, usart::CR1::Data);

constexpr usart::CR1::Data operator<<(Limited<std::uint32_t, 0x0u, 0x1Fu> left_a, usart::CR1::Shift_5 right_a)
{
    return static_cast<usart::CR1::Data>(left_a.get() << static_cast<std::uint32_t>(right_a));
}

// CR2
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::CR2::Data, usart::CR2::Flag, usart::CR2::Flag);
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::CR2::Data, usart::CR2::Data, usart::CR2::Data);
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::CR2::Data, usart::CR2::Data, usart::CR2::Flag);
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::CR2::Data, usart::CR2::Flag, usart::CR2::Data);

XSOC_USART_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(usart::CR2::Flag);
XSOC_USART_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(usart::CR2::Data);

XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS(usart::CR2::Flag);
XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS(usart::CR2::Data);
XSOC_USART_LL_GENERATE_COMPARISON_OPERATORS(usart::CR2::Flag, usart::CR2::Data);

constexpr usart::CR2::Data operator<<(Limited<std::uint32_t, 0x0u, 0x3u> left_a, usart::CR2::Shift_2 right_a)
{
    return static_cast<usart::CR2::Data>(left_a.get() << static_cast<std::uint32_t>(right_a));
}
constexpr usart::CR2::Data operator<<(Limited<std::uint32_t, 0x0u, 0xFu> left_a, usart::CR2::Shift_8 right_a)
{
    return static_cast<usart::CR2::Data>(left_a.get() << static_cast<std::uint32_t>(right_a));
}

// CR3
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::CR3::Data, usart::CR3::Flag, usart::CR3::Flag);
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::CR3::Data, usart::CR3::Data, usart::CR3::Data);
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::CR3::Data, usart::CR3::Data, usart::CR3::Flag);
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::CR3::Data, usart::CR3::Flag, usart::CR3::Data);

XSOC_USART_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(usart::CR3::Flag);
XSOC_USART_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(usart::CR3::Data);

XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS(usart::CR3::Flag);
XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS(usart::CR3::Data);
XSOC_USART_LL_GENERATE_COMPARISON_OPERATORS(usart::CR3::Flag, usart::CR3::Data);

constexpr usart::CR3::Data operator<<(Limited<std::uint32_t, 0x0u, 0x3u> left_a, usart::CR3::Shift_2 right_a)
{
    return static_cast<usart::CR3::Data>(left_a.get() << static_cast<std::uint32_t>(right_a));
}
constexpr usart::CR3::Data operator<<(Limited<std::uint32_t, 0x0u, 0xFu> left_a, usart::CR3::Shift_3 right_a)
{
    return static_cast<usart::CR3::Data>(left_a.get() << static_cast<std::uint32_t>(right_a));
}

// GTPR
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::GTPR::Data, usart::GTPR::Data, usart::GTPR::Data);
XSOC_USART_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(usart::GTPR::Data);
XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS(usart::GTPR::Data);

constexpr usart::GTPR::Data operator<<(Limited<std::uint32_t, 0x0u, 0xFu> left_a, usart::GTPR::Shift_8 right_a)
{
    return static_cast<usart::GTPR::Data>(left_a.get() << static_cast<std::uint32_t>(right_a));
}

// RTOR
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::RTOR::Data, usart::RTOR::Data, usart::RTOR::Data);
XSOC_USART_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(usart::RTOR::Data);
XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS(usart::RTOR::Data);

constexpr usart::RTOR::Data operator<<(Limited<std::uint32_t, 0x0u, 0xFFFFFFu> left_a, usart::RTOR::Shift_23 right_a)
{
    return static_cast<usart::RTOR::Data>(left_a.get() << static_cast<std::uint32_t>(right_a));
}
constexpr usart::RTOR::Data operator<<(Limited<std::uint32_t, 0x0u, 0xFu> left_a, usart::RTOR::Shift_8 right_a)
{
    return static_cast<usart::RTOR::Data>(left_a.get() << static_cast<std::uint32_t>(right_a));
}

// RQR
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::RQR::Data, usart::RQR::Flag, usart::RQR::Flag);
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::RQR::Data, usart::RQR::Data, usart::RQR::Data);
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::RQR::Data, usart::RQR::Data, usart::RQR::Flag);
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::RQR::Data, usart::RQR::Flag, usart::RQR::Data);

XSOC_USART_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(usart::RQR::Flag);
XSOC_USART_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(usart::RQR::Data);

XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS(usart::RQR::Flag);
XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS(usart::RQR::Data);
XSOC_USART_LL_GENERATE_COMPARISON_OPERATORS(usart::RQR::Flag, usart::RQR::Data);

// ISR
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::ISR::Data, usart::ISR::Flag, usart::ISR::Flag);
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::ISR::Data, usart::ISR::Data, usart::ISR::Data);
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::ISR::Data, usart::ISR::Data, usart::ISR::Flag);
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::ISR::Data, usart::ISR::Flag, usart::ISR::Data);

XSOC_USART_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(usart::ISR::Flag);
XSOC_USART_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(usart::ISR::Data);

XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS(usart::ISR::Flag);
XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS(usart::ISR::Data);
XSOC_USART_LL_GENERATE_COMPARISON_OPERATORS(usart::ISR::Flag, usart::ISR::Data);

// ICR
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::ICR::Data, usart::ICR::Flag, usart::ICR::Flag);
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::ICR::Data, usart::ICR::Data, usart::ICR::Data);
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::ICR::Data, usart::ICR::Data, usart::ICR::Flag);
XSOC_USART_LL_GENERATE_BITMASK_OPERATORS(usart::ICR::Data, usart::ICR::Flag, usart::ICR::Data);

XSOC_USART_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(usart::ICR::Flag);
XSOC_USART_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(usart::ICR::Data);

XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS(usart::ICR::Flag);
XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS(usart::ICR::Data);
XSOC_USART_LL_GENERATE_COMPARISON_OPERATORS(usart::ICR::Flag, usart::ICR::Data);
} // namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll

namespace xmcu {
// CR1
template<> [[nodiscard]] inline xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll::usart::CR1::Flag
bit::flag::get(xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll::usart::CR1 register_a,
               xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll::usart::CR1::Flag mask_a)
{
    using ll_usart = xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll::usart;
    return static_cast<ll_usart::CR1::Flag>(register_a & mask_a);
}
// CR2
template<> [[nodiscard]] inline xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll::usart::CR2::Flag
bit::flag::get(xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll::usart::CR2 register_a,
               xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll::usart::CR2::Flag mask_a)
{
    return static_cast<xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll::usart::CR2::Flag>(register_a & mask_a);
}
// CR3
template<> [[nodiscard]] inline xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll::usart::CR3::Flag
bit::flag::get(xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll::usart::CR3 register_a,
               xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll::usart::CR3::Flag mask_a)
{
    return static_cast<xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll::usart::CR3::Flag>(register_a & mask_a);
}
// ISR
template<> [[nodiscard]] inline xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll::usart::ISR::Flag
bit::flag::get(xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll::usart::ISR register_a,
               xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll::usart::ISR::Flag mask_a)
{
    using ll_usart = xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll::usart;
    return static_cast<ll_usart::ISR::Flag>(register_a & mask_a);
}
// ICR
template<> [[nodiscard]] inline xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll::usart::ICR::Flag
bit::flag::get(xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll::usart::ICR register_a,
               xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll::usart::ICR::Flag mask_a)
{
    using ll_usart = xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll::usart;
    return static_cast<ll_usart::ICR::Flag>(register_a & mask_a);
}
} // namespace xmcu