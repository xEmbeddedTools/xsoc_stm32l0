#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// CMSIS
#include <stm32l0xx.h>

// soc
#include <rm0451/peripherals/USART/base.hpp>

// xmcu
#include <xmcu/Limited.hpp>
#include <xmcu/Non_copyable.hpp>
#include <xmcu/non_constructible.hpp>

namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll {
struct usart_cr1_descriptor : private xmcu::non_constructible
{
    enum class Flag : std::uint32_t
    {
        m1 = USART_CR1_M0,
        eobie = USART_CR1_EOBIE,
        rtoie = USART_CR1_RTOIE,
        over8 = USART_CR1_OVER8,
        cmie = USART_CR1_CMIE,
        mme = USART_CR1_MME,
        m0 = USART_CR1_M0,
        wake = USART_CR1_WAKE,
        pce = USART_CR1_PCE,
        ps = USART_CR1_PS,
        peie = USART_CR1_PEIE,
        txeie = USART_CR1_TXEIE,
        tcie = USART_CR1_TCIE,
        rxneie = USART_CR1_RXNEIE,
        te = USART_CR1_TE,
        re = USART_CR1_RE,
        uesm = USART_CR1_UESM,
        ue = USART_CR1_UE
    };

    enum class Shift : std::uint32_t
    {
        deat = USART_CR1_DEAT_Pos,
        dedt = USART_CR1_DEDT_Pos
    };
};
struct usart_cr2_descriptor : private xmcu::non_constructible
{
    enum class Flag : std::uint32_t
    {
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
        add = USART_CR2_ADD
    };

    enum class add : std::uint32_t;
};
struct usart_cr3_descriptor : private xmcu::non_constructible
{
    enum class Flag : std::uint32_t
    {
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

    enum class scarcnt : std::uint32_t;
    enum class wus : std::uint32_t;
};
struct usart_brr_descriptor : private xmcu::non_constructible
{
};
struct usart_gtpr_descriptor : private xmcu::non_constructible
{
    enum class Shift_8 : std::uint32_t
    {
        psc = USART_GTPR_PSC_Pos,
        gt = USART_GTPR_GT_Pos
    };
};
struct usart_rtor_descriptor : private xmcu::non_constructible
{
    enum class Shift_23 : std::uint32_t
    {
        rto = USART_RTOR_RTO_Pos
    };

    enum class Shift_8 : std::uint32_t
    {
        blen = USART_RTOR_BLEN_Pos
    };
};
struct usart_rqr_descriptor : private xmcu::non_constructible
{
    enum class Flag : std::uint32_t
    {
        abrrq = USART_RQR_ABRRQ,
        sbkrq = USART_RQR_SBKRQ,
        mmrq = USART_RQR_MMRQ,
        rxfrq = USART_RQR_RXFRQ,
        txfrq = USART_RQR_TXFRQ
    };
};
struct usart_isr_descriptor : private xmcu::non_constructible
{
    enum class Flag : std::uint32_t
    {
        pe = USART_ISR_PE,
        fe = USART_ISR_FE,
        nf = 0x1u << 2u /*USART_ISR_NF*/,
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
        wuf = USART_ISR_WUF,
        teack = USART_ISR_TEACK,
        reack = USART_ISR_REACK,
        tcbgt = 0x1u << 25u /*USART_ISR_TCBGT*/
    };
};
struct usart_icr_descriptor : private xmcu::non_constructible
{
    enum class Flag : std::uint32_t
    {
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
};
struct usart_rdr_descriptor : private xmcu::non_constructible
{
};
struct usart_tdr_descriptor : private xmcu::non_constructible
{
};

struct usart : public usart_base
{
private:
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
        operator const Data*() const
        {
            return &(this->v);
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
        operator const Data*() const
        {
            return &(this->v);
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
        operator const Data*() const
        {
            return &(this->v);
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
        operator const Data*() const
        {
            return &(this->v);
        }

    protected:
        volatile Data v;
    };

public:
    struct CR1 : public Reg_wrc<usart_cr1_descriptor>
    {
        using Flag = usart_cr1_descriptor::Flag;
        using Shift = usart_cr1_descriptor::Shift;

        using enum Flag;
        using enum Shift;

        using Data = Reg_wrc<usart_cr1_descriptor>::Data;

        CR1& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }
        CR1& operator=(Flag value_a)
        {
            this->v = static_cast<Data>(value_a);
            return *this;
        }
    };
    struct CR2 : public Reg_wrc<usart_cr2_descriptor>
    {
        using Flag = usart_cr2_descriptor::Flag;
        using Shift_2 = usart_cr2_descriptor::Shift_2;
        using Shift_8 = usart_cr2_descriptor::Shift_8;

        using enum Flag;
        using enum Shift_2;
        using enum Shift_8;

        using Data = Reg_wrc<usart_cr2_descriptor>::Data;

        CR2& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }
    };
    struct CR3 : public Reg_wrc<usart_cr3_descriptor>
    {
        using Flag = usart_cr3_descriptor::Flag;
        using scarcnt = usart_cr3_descriptor::scarcnt;
        using wus = usart_cr3_descriptor::wus;

        using enum Flag;

        using Data = Reg_wrc<usart_cr3_descriptor>::Data;

        CR3& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }
    };
    struct BRR : public Reg_wrc<usart_brr_descriptor>
    {
        using Data = Reg_wrc<usart_brr_descriptor>::Data;

        BRR& operator=(Data value_a)
        {
            this->v = static_cast<Data>(static_cast<std::uint32_t>(value_a) & 0xFFFF);
            return *this;
        }

        BRR operator=(Limited<std::uint32_t, 0x0u, 0xFFFFu> value_a)
        {
            this->v = static_cast<Data>(value_a.get());
            return *this;
        }
    };
    struct GTPR : public Reg_wrc<usart_gtpr_descriptor>
    {
        using Shift_8 = usart_gtpr_descriptor::Shift_8;

        using enum Shift_8;

        using Data = Reg_wrc<usart_gtpr_descriptor>::Data;

        GTPR& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }
    };
    struct RTOR : public Reg_wrc<usart_rtor_descriptor>
    {
        using Shift_23 = usart_rtor_descriptor::Shift_23;
        using Shift_8 = usart_rtor_descriptor::Shift_8;

        using enum Shift_23;
        using enum Shift_8;

        using Data = Reg_wrc<usart_rtor_descriptor>::Data;

        RTOR& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }
    };
    struct RQR : public Reg_wrc<usart_rqr_descriptor>
    {
        using Flag = usart_rqr_descriptor::Flag;

        using enum Flag;

        using Data = Reg_wrc<usart_rqr_descriptor>::Data;

        RQR& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }
    };
    struct ISR : public Reg_wrc<usart_isr_descriptor>
    {
        using Flag = usart_isr_descriptor::Flag;

        using enum Flag;

        using Data = Reg_wrc<usart_isr_descriptor>::Data;

        ISR& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }
    };
    struct ICR : public Reg_wrc<usart_icr_descriptor>
    {
        using Flag = usart_icr_descriptor::Flag;

        using enum Flag;

        using Data = Reg_wrc<usart_icr_descriptor>::Data;

        ICR& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }
    };
    struct RDR : public Reg_r<usart_rdr_descriptor>
    {
        using Data = Reg_r<usart_rqr_descriptor>::Data;

        constexpr operator std::uint8_t() const
        {
            return static_cast<std::uint8_t>(this->v);
        }
    };
    struct TDR : public Reg_wrc<usart_tdr_descriptor>
    {
    };

    struct Registers : private xmcu::Non_copyable
    {
        CR1 cr1;
        CR2 cr2;
        CR3 cr3;
        BRR brr;
        GTPR gtpr;
        RTOR rtor;
        RQR rqr;
        ISR isr;
        ICR icr;
        RDR rdr;
        TDR tdr;
    };

    template<typename Port> [[nodiscard]] static constexpr Registers* registers() = delete;
};

#if defined XMCU_USART2_PRESENT
template<> [[nodiscard]] constexpr usart::Registers* usart::registers<usart::_2>()
{
    return reinterpret_cast<usart::Registers*>(USART2_BASE);
}

// CR1
constexpr usart::CR1::Data operator|(usart::CR1::Flag left_a, usart::CR1::Flag right_a)
{
    return static_cast<usart::CR1::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr usart::CR1::Data operator|(usart::CR1::Data left_a, usart::CR1::Data right_a)
{
    return static_cast<usart::CR1::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr usart::CR1::Data operator|(usart::CR1::Flag left_a, usart::CR1::Data right_a)
{
    return static_cast<usart::CR1::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr usart::CR1::Data operator|(usart::CR1::Data left_a, usart::CR1::Flag right_a)
{
    return static_cast<usart::CR1::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}

constexpr usart::CR1::Data operator&(usart::CR1::Data left_a, usart::CR1::Data right_a)
{
    return static_cast<usart::CR1::Data>(static_cast<std::uint32_t>(left_a) & static_cast<std::uint32_t>(right_a));
}
constexpr usart::CR1::Data operator&(usart::CR1::Data left_a, usart::CR1::Flag right_a)
{
    return static_cast<usart::CR1::Data>(static_cast<std::uint32_t>(left_a) & static_cast<std::uint32_t>(right_a));
}

constexpr bool operator==(usart::CR1::Data left_a, usart::CR1::Flag right_a)
{
    return (static_cast<std::uint32_t>(left_a) == static_cast<std::uint32_t>(right_a));
}
constexpr bool operator==(usart::CR1::Flag left_a, usart::CR1::Data right_a)
{
    return (static_cast<std::uint32_t>(left_a) == static_cast<std::uint32_t>(right_a));
}
constexpr bool operator!=(usart::CR1::Data left_a, usart::CR1::Flag right_a)
{
    return not(left_a == right_a);
}
constexpr bool operator!=(usart::CR1::Flag left_a, usart::CR1::Data right_a)
{
    return not(left_a == right_a);
}

constexpr usart::CR1::Data operator~(usart::CR1::Flag left_a)
{
    return static_cast<usart::CR1::Data>(~static_cast<std::uint32_t>(left_a));
}
constexpr usart::CR1::Data operator~(usart::CR1::Data left_a)
{
    return static_cast<usart::CR1::Data>(~static_cast<std::uint32_t>(left_a));
}

constexpr usart::CR1::Data operator<<(Limited<std::uint32_t, 0x0u, 0x1Fu> left_a, usart::CR1::Shift right_a)
{
    return static_cast<usart::CR1::Data>(left_a.get() << static_cast<std::uint32_t>(right_a));
}

// CR2
constexpr usart::CR2::Data operator|(usart::CR2::Flag left_a, usart::CR2::Flag right_a)
{
    return static_cast<usart::CR2::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr usart::CR2::Data operator|(usart::CR2::Data left_a, usart::CR2::Data right_a)
{
    return static_cast<usart::CR2::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr usart::CR2::Data operator|(usart::CR2::Flag left_a, usart::CR2::Data right_a)
{
    return static_cast<usart::CR2::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr usart::CR2::Data operator|(usart::CR2::Data left_a, usart::CR2::Flag right_a)
{
    return static_cast<usart::CR2::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}

constexpr usart::CR2::Data operator&(usart::CR2::Data left_a, usart::CR2::Data right_a)
{
    return static_cast<usart::CR2::Data>(static_cast<std::uint32_t>(left_a) & static_cast<std::uint32_t>(right_a));
}
constexpr usart::CR2::Data operator&(usart::CR2::Data left_a, usart::CR2::Flag right_a)
{
    return static_cast<usart::CR2::Data>(static_cast<std::uint32_t>(left_a) & static_cast<std::uint32_t>(right_a));
}

constexpr bool operator==(usart::CR2::Data left_a, usart::CR2::Flag right_a)
{
    return (static_cast<std::uint32_t>(left_a) == static_cast<std::uint32_t>(right_a));
}
constexpr bool operator==(usart::CR2::Flag left_a, usart::CR2::Data right_a)
{
    return (static_cast<std::uint32_t>(left_a) == static_cast<std::uint32_t>(right_a));
}
constexpr bool operator!=(usart::CR2::Data left_a, usart::CR2::Flag right_a)
{
    return not(left_a == right_a);
}
constexpr bool operator!=(usart::CR2::Flag left_a, usart::CR2::Data right_a)
{
    return not(left_a == right_a);
}

constexpr usart::CR2::Data operator~(usart::CR2::Flag left_a)
{
    return static_cast<usart::CR2::Data>(~static_cast<std::uint32_t>(left_a));
}
constexpr usart::CR2::Data operator~(usart::CR2::Data left_a)
{
    return static_cast<usart::CR2::Data>(~static_cast<std::uint32_t>(left_a));
}

constexpr usart::CR1::Data operator<<(Limited<std::uint32_t, 0x0u, 0x3u> left_a, usart::CR2::Shift_2 right_a)
{
    return static_cast<usart::CR1::Data>(left_a.get() << static_cast<std::uint32_t>(right_a));
}
constexpr usart::CR1::Data operator<<(Limited<std::uint32_t, 0x0u, 0xFu> left_a, usart::CR2::Shift_8 right_a)
{
    return static_cast<usart::CR1::Data>(left_a.get() << static_cast<std::uint32_t>(right_a));
}

// CR3
constexpr usart::CR3::Data operator|(usart::CR3::Flag left_a, usart::CR3::Flag right_a)
{
    return static_cast<usart::CR3::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr usart::CR3::Data operator|(usart::CR3::Data left_a, usart::CR3::Data right_a)
{
    return static_cast<usart::CR3::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr usart::CR3::Data operator|(usart::CR3::Flag left_a, usart::CR3::Data right_a)
{
    return static_cast<usart::CR3::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr usart::CR3::Data operator|(usart::CR3::Data left_a, usart::CR3::Flag right_a)
{
    return static_cast<usart::CR3::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}

constexpr usart::CR3::Data operator&(usart::CR3::Data left_a, usart::CR3::Data right_a)
{
    return static_cast<usart::CR3::Data>(static_cast<std::uint32_t>(left_a) & static_cast<std::uint32_t>(right_a));
}
constexpr usart::CR3::Data operator&(usart::CR3::Data left_a, usart::CR3::Flag right_a)
{
    return static_cast<usart::CR3::Data>(static_cast<std::uint32_t>(left_a) & static_cast<std::uint32_t>(right_a));
}

constexpr bool operator==(usart::CR3::Data left_a, usart::CR3::Flag right_a)
{
    return (static_cast<std::uint32_t>(left_a) == static_cast<std::uint32_t>(right_a));
}
constexpr bool operator==(usart::CR3::Flag left_a, usart::CR3::Data right_a)
{
    return (static_cast<std::uint32_t>(left_a) == static_cast<std::uint32_t>(right_a));
}
constexpr bool operator!=(usart::CR3::Data left_a, usart::CR3::Flag right_a)
{
    return not(left_a == right_a);
}
constexpr bool operator!=(usart::CR3::Flag left_a, usart::CR3::Data right_a)
{
    return not(left_a == right_a);
}

constexpr usart::CR3::Data operator~(usart::CR3::Flag left_a)
{
    return static_cast<usart::CR3::Data>(~static_cast<std::uint32_t>(left_a));
}
constexpr usart::CR3::Data operator~(usart::CR3::Data left_a)
{
    return static_cast<usart::CR3::Data>(~static_cast<std::uint32_t>(left_a));
}

// GTPR
constexpr usart::CR1::Data operator<<(Limited<std::uint32_t, 0x0u, 0xFu> left_a, usart::GTPR::Shift_8 right_a)
{
    return static_cast<usart::CR1::Data>(left_a.get() << static_cast<std::uint32_t>(right_a));
}

// RTOR
constexpr usart::RTOR::Data operator<<(Limited<std::uint32_t, 0x0u, 0xFFFFFFu> left_a, usart::RTOR::Shift_23 right_a)
{
    return static_cast<usart::RTOR::Data>(left_a.get() << static_cast<std::uint32_t>(right_a));
}
constexpr usart::RTOR::Data operator<<(Limited<std::uint32_t, 0x0u, 0xFu> left_a, usart::RTOR::Shift_8 right_a)
{
    return static_cast<usart::RTOR::Data>(left_a.get() << static_cast<std::uint32_t>(right_a));
}

// RQR
constexpr usart::RQR::Data operator|(usart::RQR::Flag left_a, usart::RQR::Flag right_a)
{
    return static_cast<usart::RQR::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr usart::RQR::Data operator|(usart::RQR::Data left_a, usart::RQR::Data right_a)
{
    return static_cast<usart::RQR::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr usart::RQR::Data operator|(usart::RQR::Flag left_a, usart::RQR::Data right_a)
{
    return static_cast<usart::RQR::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr usart::RQR::Data operator|(usart::RQR::Data left_a, usart::RQR::Flag right_a)
{
    return static_cast<usart::RQR::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}

constexpr usart::RQR::Data operator&(usart::RQR::Data left_a, usart::RQR::Data right_a)
{
    return static_cast<usart::RQR::Data>(static_cast<std::uint32_t>(left_a) & static_cast<std::uint32_t>(right_a));
}
constexpr usart::RQR::Data operator&(usart::RQR::Data left_a, usart::RQR::Flag right_a)
{
    return static_cast<usart::RQR::Data>(static_cast<std::uint32_t>(left_a) & static_cast<std::uint32_t>(right_a));
}

constexpr bool operator==(usart::RQR::Data left_a, usart::RQR::Flag right_a)
{
    return (static_cast<std::uint32_t>(left_a) == static_cast<std::uint32_t>(right_a));
}
constexpr bool operator==(usart::RQR::Flag left_a, usart::RQR::Data right_a)
{
    return (static_cast<std::uint32_t>(left_a) == static_cast<std::uint32_t>(right_a));
}
constexpr bool operator!=(usart::RQR::Data left_a, usart::RQR::Flag right_a)
{
    return not(left_a == right_a);
}
constexpr bool operator!=(usart::RQR::Flag left_a, usart::RQR::Data right_a)
{
    return not(left_a == right_a);
}

constexpr usart::RQR::Data operator~(usart::RQR::Flag left_a)
{
    return static_cast<usart::RQR::Data>(~static_cast<std::uint32_t>(left_a));
}
constexpr usart::RQR::Data operator~(usart::RQR::Data left_a)
{
    return static_cast<usart::RQR::Data>(~static_cast<std::uint32_t>(left_a));
}

// ISR
constexpr usart::ISR::Data operator|(usart::ISR::Flag left_a, usart::ISR::Flag right_a)
{
    return static_cast<usart::ISR::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr usart::ISR::Data operator|(usart::ISR::Data left_a, usart::ISR::Data right_a)
{
    return static_cast<usart::ISR::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr usart::ISR::Data operator|(usart::ISR::Flag left_a, usart::ISR::Data right_a)
{
    return static_cast<usart::ISR::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr usart::ISR::Data operator|(usart::ISR::Data left_a, usart::ISR::Flag right_a)
{
    return static_cast<usart::ISR::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}

constexpr usart::ISR::Data operator&(usart::ISR::Data left_a, usart::ISR::Data right_a)
{
    return static_cast<usart::ISR::Data>(static_cast<std::uint32_t>(left_a) & static_cast<std::uint32_t>(right_a));
}
constexpr usart::ISR::Data operator&(usart::ISR::Data left_a, usart::ISR::Flag right_a)
{
    return static_cast<usart::ISR::Data>(static_cast<std::uint32_t>(left_a) & static_cast<std::uint32_t>(right_a));
}

constexpr bool operator==(usart::ISR::Data left_a, usart::ISR::Flag right_a)
{
    return (static_cast<std::uint32_t>(left_a) == static_cast<std::uint32_t>(right_a));
}
constexpr bool operator==(usart::ISR::Flag left_a, usart::ISR::Data right_a)
{
    return (static_cast<std::uint32_t>(left_a) == static_cast<std::uint32_t>(right_a));
}
constexpr bool operator!=(usart::ISR::Data left_a, usart::ISR::Flag right_a)
{
    return not(left_a == right_a);
}
constexpr bool operator!=(usart::ISR::Flag left_a, usart::ISR::Data right_a)
{
    return not(left_a == right_a);
}

constexpr usart::ISR::Data operator~(usart::ISR::Flag left_a)
{
    return static_cast<usart::ISR::Data>(~static_cast<std::uint32_t>(left_a));
}
constexpr usart::ISR::Data operator~(usart::ISR::Data left_a)
{
    return static_cast<usart::ISR::Data>(~static_cast<std::uint32_t>(left_a));
}

// ICR
constexpr usart::ICR::Data operator|(usart::ICR::Flag left_a, usart::ICR::Flag right_a)
{
    return static_cast<usart::ICR::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr usart::ICR::Data operator|(usart::ICR::Data left_a, usart::ICR::Data right_a)
{
    return static_cast<usart::ICR::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr usart::ICR::Data operator|(usart::ICR::Flag left_a, usart::ICR::Data right_a)
{
    return static_cast<usart::ICR::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}
constexpr usart::ICR::Data operator|(usart::ICR::Data left_a, usart::ICR::Flag right_a)
{
    return static_cast<usart::ICR::Data>(static_cast<std::uint32_t>(left_a) | static_cast<std::uint32_t>(right_a));
}

constexpr usart::ICR::Data operator&(usart::ICR::Data left_a, usart::ICR::Data right_a)
{
    return static_cast<usart::ICR::Data>(static_cast<std::uint32_t>(left_a) & static_cast<std::uint32_t>(right_a));
}
constexpr usart::ICR::Data operator&(usart::ICR::Data left_a, usart::ICR::Flag right_a)
{
    return static_cast<usart::ICR::Data>(static_cast<std::uint32_t>(left_a) & static_cast<std::uint32_t>(right_a));
}

constexpr bool operator==(usart::ICR::Data left_a, usart::ICR::Flag right_a)
{
    return (static_cast<std::uint32_t>(left_a) == static_cast<std::uint32_t>(right_a));
}
constexpr bool operator==(usart::ICR::Flag left_a, usart::ICR::Data right_a)
{
    return (static_cast<std::uint32_t>(left_a) == static_cast<std::uint32_t>(right_a));
}
constexpr bool operator!=(usart::ICR::Data left_a, usart::ICR::Flag right_a)
{
    return not(left_a == right_a);
}
constexpr bool operator!=(usart::ICR::Flag left_a, usart::ICR::Data right_a)
{
    return not(left_a == right_a);
}

constexpr usart::ICR::Data operator~(usart::ICR::Flag left_a)
{
    return static_cast<usart::ICR::Data>(~static_cast<std::uint32_t>(left_a));
}
constexpr usart::ICR::Data operator~(usart::ICR::Data left_a)
{
    return static_cast<usart::ICR::Data>(~static_cast<std::uint32_t>(left_a));
}

// RDR

// TDR
#endif
} // namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll

namespace xmcu {
// CR1
template<> [[nodiscard]] inline xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll::usart_cr1_descriptor::Flag
bit::flag::get(xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll::usart::CR1 register_a,
               xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll::usart::CR1::Flag mask_a)
{
    return static_cast<xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll::usart_cr1_descriptor::Flag>(register_a &
                                                                                                        mask_a);
}
// CR2
template<> [[nodiscard]] inline xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll::usart_cr2_descriptor::Flag
bit::flag::get(xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll::usart::CR2 register_a,
               xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll::usart::CR2::Flag mask_a)
{
    return static_cast<xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll::usart_cr2_descriptor::Flag>(register_a &
                                                                                                        mask_a);
}
// CR3
template<> [[nodiscard]] inline xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll::usart_cr3_descriptor::Flag
bit::flag::get(xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll::usart::CR3 register_a,
               xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll::usart::CR3::Flag mask_a)
{
    return static_cast<xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll::usart_cr3_descriptor::Flag>(register_a &
                                                                                                        mask_a);
}
} // namespace xmcu