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

namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll {
template<typename T> struct shift_operators_traits
{
    using return_type = void;
    static constexpr std::uint32_t max = 0x1u;
};
template<typename T> struct bitmask_operators_traits
{
    using return_type = void;
};
template<typename T>
concept is_bitmask = !std::is_same_v<typename bitmask_operators_traits<T>::return_type, void>;

template<is_bitmask L, is_bitmask R> constexpr auto operator|(L lhs_a, R rhs_a) noexcept
{
    return static_cast<typename bitmask_operators_traits<L>::return_type>(lhs_a) |
           static_cast<typename bitmask_operators_traits<R>::return_type>(rhs_a);
}
template<is_bitmask L, is_bitmask R> constexpr auto operator&(L lhs_a, R rhs_a) noexcept
{
    return static_cast<typename bitmask_operators_traits<L>::return_type>(lhs_a) &
           static_cast<typename bitmask_operators_traits<R>::return_type>(rhs_a);
}
template<is_bitmask L, is_bitmask R> constexpr auto operator^(L lhs_a, R rhs_a) noexcept
{
    return static_cast<typename bitmask_operators_traits<L>::return_type>(lhs_a) ^
           static_cast<typename bitmask_operators_traits<R>::return_type>(rhs_a);
}

template<is_bitmask L, is_bitmask R> constexpr auto operator==(L lhs_a, R rhs_a) noexcept
{
    return static_cast<typename bitmask_operators_traits<L>::return_type>(lhs_a) ==
           static_cast<typename bitmask_operators_traits<R>::return_type>(rhs_a);
}
template<is_bitmask L, is_bitmask R> constexpr auto operator!=(L lhs_a, R rhs_a) noexcept
{
    return static_cast<typename bitmask_operators_traits<L>::return_type>(lhs_a) !=
           static_cast<typename bitmask_operators_traits<R>::return_type>(rhs_a);
}

template<is_bitmask L, is_bitmask R> constexpr L& operator|=(L& lhs_a, R rhs_a) noexcept
{
    lhs_a = static_cast<L>(lhs_a | rhs_a);
    return lhs_a;
}
template<is_bitmask L, is_bitmask R> constexpr L& operator&=(L& lhs_a, R rhs_a) noexcept
{
    lhs_a = static_cast<L>(lhs_a & rhs_a);
    return lhs_a;
}
template<is_bitmask L, is_bitmask R> constexpr L& operator^=(L& lhs_a, R rhs_a) noexcept
{
    lhs_a = static_cast<L>(lhs_a ^ rhs_a);
    return lhs_a;
}

template<is_bitmask L> constexpr auto operator~(L lhs_a) noexcept
{
    return static_cast<L>(~static_cast<std::conditional_t<std::is_enum_v<L>, std::underlying_type_t<L>, L>>(lhs_a));
}
template<is_bitmask L> constexpr bool operator!(L lhs_a) noexcept
{
    return static_cast<typename bitmask_operators_traits<L>::return_type>(lhs_a) == 0u;
}

template<typename shift> constexpr auto
operator<<(Limited<std::uint32_t, 0x0u, shift_operators_traits<shift>::max> left_a, shift right_a) noexcept
{
    return static_cast<typename shift_operators_traits<shift>::return_type>(left_a.get()
                                                                            << static_cast<std::uint32_t>(right_a));
}

template<typename shift>
constexpr auto operator&(typename shift_operators_traits<shift>::return_type left_a, shift right_a) noexcept
{
    return static_cast<typename shift_operators_traits<shift>::return_type>(
        static_cast<std::underlying_type_t<typename shift_operators_traits<shift>::return_type>>(left_a) &
        (shift_operators_traits<shift>::max
         << static_cast<std::underlying_type_t<typename shift_operators_traits<shift>::return_type>>(right_a)));
}

template<typename shift>
constexpr auto operator&(shift right_a, typename shift_operators_traits<shift>::return_type left_a) noexcept
{
    return static_cast<typename shift_operators_traits<shift>::return_type>(
        static_cast<std::underlying_type_t<typename shift_operators_traits<shift>::return_type>>(left_a) &
        (shift_operators_traits<shift>::max
         << static_cast<std::underlying_type_t<typename shift_operators_traits<shift>::return_type>>(right_a)));
}

struct usart : public usart_base
{
    struct CR1
    {
        enum class Data : std::uint32_t;

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

        CR1(Data v_a)
            : v(v_a)
        {
        }
        CR1(Flag v_a)
            : v(static_cast<Data>(v_a))
        {
        }

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
        enum class Data : std::uint32_t;

    private:
        struct STOP
        {
            enum class Flag : std::uint32_t
            {
                _1_5 = USART_CR2_STOP_0 | USART_CR2_STOP_1,
                _2_0 = USART_CR2_STOP_1,
                _0_5 = USART_CR2_STOP_0,
                _1_0 = 0x0u
            };
            enum class Mask : std::uint32_t
            {
                mask = 0x3u << USART_CR2_STOP_Pos
            };

            using enum Flag;
            using enum Mask;

            operator Mask() const
            {
                return this->mask;
            }
        };
        struct ABRMOD
        {
            enum class Flag : std::uint32_t
            {
                start_bit = 0x0u,
                falling_edge_to_falling_edge = USART_CR2_ABRMODE_0,
                frame_0x75_detection = USART_CR2_ABRMODE_1,
                frame_0x55_detection = USART_CR2_ABRMODE_0 | USART_CR2_ABRMODE_1
            };
            enum class Mask : std::uint32_t
            {
                mask = 0x3u << USART_CR2_ABRMODE_Pos
            };

            using enum Flag;
            using enum Mask;

            operator Data() const
            {
                return static_cast<Data>(this->mask);
            }

            Data operator~() const
            {
                return static_cast<Data>(~static_cast<std::underlying_type_t<Mask>>(this->mask));
            }
        };

    public:
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
        enum class Shift_8 : std::uint32_t
        {
            add = USART_CR2_ADD_Pos
        };

        static STOP stop;
        static ABRMOD abrmod;

        using enum Flag;
        using enum Shift_8;

        CR2(Data v_a)
            : v(v_a)
        {
        }
        CR2(Flag v_a)
            : v(static_cast<Data>(v_a))
        {
        }

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

        CR2& operator=(STOP::Flag value_a)
        {
            this->v = static_cast<Data>(value_a);
            return *this;
        }
        CR2& operator=(ABRMOD::Flag value_a)
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
    struct CR3
    {
    public:
        enum class Data : std::uint32_t;

    private:
        struct WUS
        {
            enum class Flag : std::uint32_t
            {
                address_match = 0x0u,
                start_bit_detection = USART_CR3_WUS_1,
                rxne = USART_CR3_WUS_0 | USART_CR3_WUS_1
            };
            enum class Mask : std::uint32_t
            {
                mask = 0x3u << USART_CR3_WUS_Pos
            };

            using enum Flag;
            using enum Mask;

            operator Data() const
            {
                return static_cast<Data>(this->mask);
            }

            Data operator~() const
            {
                return static_cast<Data>(~static_cast<std::underlying_type_t<Mask>>(this->mask));
            }
        };

    public:
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
        enum class Shift_3 : std::uint32_t
        {
            scarcnt = USART_CR3_SCARCNT_Pos
        };

        using enum Flag;
        using enum Shift_3;

        static WUS wus;

        CR3(Data v_a)
            : v(v_a)
        {
        }
        CR3(Flag v_a)
            : v(static_cast<Data>(v_a))
        {
        }

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

        CR3& operator=(WUS::Flag value_a)
        {
            this->v = static_cast<Data>(value_a);
            return *this;
        }

        operator Data() const
        {
            return static_cast<Data>(this->v);
        }

    private:
        volatile Data v;
    };
    struct BRR
    {
        enum class Data : std::uint32_t;

        BRR& operator=(std::uint32_t value_a)
        {
            this->v = static_cast<Data>(value_a);
            return *this;
        }

        BRR& operator=(Data value_a)
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
    struct GTPR
    {
        enum class Data : std::uint32_t;

        enum class Shift_8 : std::uint32_t
        {
            psc = USART_GTPR_PSC_Pos,
            gt = USART_GTPR_GT_Pos
        };

        using enum Shift_8;

        GTPR(Data v_a)
            : v(v_a)
        {
        }

        GTPR& operator=(Data value_a)
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
    struct RTOR
    {
        enum class Data : std::uint32_t;

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

        RTOR(Data v_a)
            : v(v_a)
        {
        }

        RTOR& operator=(Data value_a)
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
    struct RQR
    {
        enum class Data : std::uint32_t;

        enum class Flag : std::uint32_t
        {
            abrrq = USART_RQR_ABRRQ,
            sbkrq = USART_RQR_SBKRQ,
            mmrq = USART_RQR_MMRQ,
            rxfrq = USART_RQR_RXFRQ,
            txfrq = USART_RQR_TXFRQ
        };

        using enum Flag;

        RQR(Data v_a)
            : v(v_a)
        {
        }
        RQR(Flag v_a)
            : v(static_cast<Data>(v_a))
        {
        }

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

        operator Data() const
        {
            return this->v;
        }

    private:
        volatile Data v;
    };
    struct ISR
    {
        enum class Data : std::uint32_t;

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
        enum class Data : std::uint32_t;

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

        ICR(Data v_a)
            : v(v_a)
        {
        }
        ICR(Flag v_a)
            : v(static_cast<Data>(v_a))
        {
        }

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
template<> struct bitmask_operators_traits<usart::CR1::Flag>
{
    using return_type = usart::CR1::Data;
};
template<> struct bitmask_operators_traits<usart::CR1::Data>
{
    using return_type = usart::CR1::Data;
};
template<> struct bitmask_operators_traits<usart::CR1>
{
    using return_type = usart::CR1::Data;
};
template<> struct shift_operators_traits<usart::CR1::Shift_5>
{
    using return_type = usart::CR1::Data;
    static constexpr std::uint32_t max = 0x1Fu;
};

constexpr usart::CR1::Data operator|(usart::CR1::Data left_a, usart::CR1::Data right_a) noexcept
{
    return static_cast<usart::CR1::Data>(static_cast<std::underlying_type_t<usart::CR1::Data>>(left_a) |
                                         static_cast<std::underlying_type_t<usart::CR1::Data>>(right_a));
}
constexpr usart::CR1::Data operator&(usart::CR1::Data left_a, usart::CR1::Data right_a) noexcept
{
    return static_cast<usart::CR1::Data>(static_cast<std::underlying_type_t<usart::CR1::Data>>(left_a) &
                                         static_cast<std::underlying_type_t<usart::CR1::Data>>(right_a));
}

constexpr bool operator==(usart::CR1::Data lhs_a, std::uint32_t rhs_a) noexcept
{
    return (static_cast<std::underlying_type_t<usart::CR1::Data>>(lhs_a) == rhs_a);
}
constexpr bool operator==(std::uint32_t lhs_a, usart::CR1::Data rhs_a) noexcept
{
    return (static_cast<std::underlying_type_t<usart::CR1::Data>>(rhs_a) == lhs_a);
}

// CR2
template<> struct bitmask_operators_traits<usart::CR2::Flag>
{
    using return_type = usart::CR2::Data;
};
template<> struct bitmask_operators_traits<usart::CR2::Data>
{
    using return_type = usart::CR2::Data;
};
template<> struct bitmask_operators_traits<usart::CR2>
{
    using return_type = usart::CR2::Data;
};
template<> struct shift_operators_traits<usart::CR2::Shift_8>
{
    using return_type = usart::CR2::Data;
    static constexpr std::uint32_t max = 0xFu;
};

constexpr usart::CR2::Data operator|(usart::CR2::Data left_a, usart::CR2::Data right_a) noexcept
{
    return static_cast<usart::CR2::Data>(static_cast<std::underlying_type_t<usart::CR2::Data>>(left_a) |
                                         static_cast<std::underlying_type_t<usart::CR2::Data>>(right_a));
}
constexpr usart::CR2::Data operator&(usart::CR2::Data left_a, usart::CR2::Data right_a) noexcept
{
    return static_cast<usart::CR2::Data>(static_cast<std::underlying_type_t<usart::CR2::Data>>(left_a) &
                                         static_cast<std::underlying_type_t<usart::CR2::Data>>(right_a));
}

constexpr bool operator==(usart::CR2::Data lhs_a, std::uint32_t rhs_a) noexcept
{
    return (static_cast<std::underlying_type_t<usart::CR2::Data>>(lhs_a) == rhs_a);
}
constexpr bool operator==(std::uint32_t lhs_a, usart::CR2::Data rhs_a) noexcept
{
    return (static_cast<std::underlying_type_t<usart::CR2::Data>>(rhs_a) == lhs_a);
}

// CR2::STOP
template<> struct bitmask_operators_traits<usart::CR2::STOP>
{
    using return_type = usart::CR2::Data;
};
template<> struct bitmask_operators_traits<usart::CR2::STOP::Flag>
{
    using return_type = usart::CR2::Data;
};

// CR2::ABRMOD
template<> struct bitmask_operators_traits<usart::CR2::ABRMOD>
{
    using return_type = usart::CR2::Data;
};
template<> struct bitmask_operators_traits<usart::CR2::ABRMOD::Flag>
{
    using return_type = usart::CR2::Data;
};

// CR3
template<> struct bitmask_operators_traits<usart::CR3::Flag>
{
    using return_type = usart::CR3::Data;
};
template<> struct bitmask_operators_traits<usart::CR3::Data>
{
    using return_type = usart::CR3::Data;
};
template<> struct bitmask_operators_traits<usart::CR3>
{
    using return_type = usart::CR3::Data;
};
template<> struct shift_operators_traits<usart::CR3::Shift_3>
{
    using return_type = usart::CR3::Data;
    static constexpr std::uint32_t max = 0x7u;
};

constexpr usart::CR3::Data operator|(usart::CR3::Data left_a, usart::CR3::Data right_a) noexcept
{
    return static_cast<usart::CR3::Data>(static_cast<std::underlying_type_t<usart::CR3::Data>>(left_a) |
                                         static_cast<std::underlying_type_t<usart::CR3::Data>>(right_a));
}
constexpr usart::CR3::Data operator&(usart::CR3::Data left_a, usart::CR3::Data right_a) noexcept
{
    return static_cast<usart::CR3::Data>(static_cast<std::underlying_type_t<usart::CR3::Data>>(left_a) &
                                         static_cast<std::underlying_type_t<usart::CR3::Data>>(right_a));
}

constexpr bool operator==(usart::CR3::Data lhs_a, std::uint32_t rhs_a) noexcept
{
    return (static_cast<std::underlying_type_t<usart::CR3::Data>>(lhs_a) == rhs_a);
}
constexpr bool operator==(std::uint32_t lhs_a, usart::CR3::Data rhs_a) noexcept
{
    return (static_cast<std::underlying_type_t<usart::CR3::Data>>(rhs_a) == lhs_a);
}

// CR3::WUS
template<> struct bitmask_operators_traits<usart::CR3::WUS>
{
    using return_type = usart::CR3::Data;
};
template<> struct bitmask_operators_traits<usart::CR3::WUS::Flag>
{
    using return_type = usart::CR3::Data;
};

// GTPR
template<> struct bitmask_operators_traits<usart::GTPR::Data>
{
    using return_type = usart::GTPR::Data;
};
template<> struct bitmask_operators_traits<usart::GTPR>
{
    using return_type = usart::GTPR::Data;
};
template<> struct shift_operators_traits<usart::GTPR::Shift_8>
{
    using return_type = usart::GTPR::Data;
    static constexpr std::uint32_t max = 0xFu;
};

constexpr usart::GTPR::Data operator|(usart::GTPR::Data left_a, usart::GTPR::Data right_a) noexcept
{
    return static_cast<usart::GTPR::Data>(static_cast<std::underlying_type_t<usart::GTPR::Data>>(left_a) |
                                          static_cast<std::underlying_type_t<usart::GTPR::Data>>(right_a));
}
constexpr usart::GTPR::Data operator&(usart::GTPR::Data left_a, usart::GTPR::Data right_a) noexcept
{
    return static_cast<usart::GTPR::Data>(static_cast<std::underlying_type_t<usart::GTPR::Data>>(left_a) &
                                          static_cast<std::underlying_type_t<usart::GTPR::Data>>(right_a));
}

constexpr bool operator==(usart::GTPR::Data left_a, std::uint32_t right_a) noexcept
{
    return (static_cast<std::underlying_type_t<usart::GTPR::Data>>(left_a) == right_a);
}
constexpr bool operator==(std::uint32_t right_a, usart::GTPR::Data left_a) noexcept
{
    return (static_cast<std::underlying_type_t<usart::GTPR::Data>>(left_a) == right_a);
}

// RTOR
template<> struct bitmask_operators_traits<usart::RTOR::Data>
{
    using return_type = usart::RTOR::Data;
};
template<> struct bitmask_operators_traits<usart::RTOR>
{
    using return_type = usart::RTOR::Data;
};
template<> struct shift_operators_traits<usart::RTOR::Shift_23>
{
    using return_type = usart::RTOR::Data;
    static constexpr std::uint32_t max = 0xFFFFFFu;
};
template<> struct shift_operators_traits<usart::RTOR::Shift_8>
{
    using return_type = usart::RTOR::Data;
    static constexpr std::uint32_t max = 0xFu;
};

constexpr usart::RTOR::Data operator|(usart::RTOR::Data left_a, usart::RTOR::Data right_a) noexcept
{
    return static_cast<usart::RTOR::Data>(static_cast<std::underlying_type_t<usart::RTOR::Data>>(left_a) |
                                          static_cast<std::underlying_type_t<usart::RTOR::Data>>(right_a));
}
constexpr usart::RTOR::Data operator&(usart::RTOR::Data left_a, usart::RTOR::Data right_a) noexcept
{
    return static_cast<usart::RTOR::Data>(static_cast<std::underlying_type_t<usart::RTOR::Data>>(left_a) &
                                          static_cast<std::underlying_type_t<usart::RTOR::Data>>(right_a));
}

constexpr bool operator==(usart::RTOR::Data lhs_a, std::uint32_t rhs_a) noexcept
{
    return (static_cast<std::underlying_type_t<usart::RTOR::Data>>(lhs_a) == rhs_a);
}
constexpr bool operator==(std::uint32_t lhs_a, usart::RTOR::Data rhs_a) noexcept
{
    return (static_cast<std::underlying_type_t<usart::RTOR::Data>>(rhs_a) == lhs_a);
}

// RQR
template<> struct bitmask_operators_traits<usart::RQR::Flag>
{
    using return_type = usart::RQR::Data;
};
template<> struct bitmask_operators_traits<usart::RQR::Data>
{
    using return_type = usart::RQR::Data;
};
template<> struct bitmask_operators_traits<usart::RQR>
{
    using return_type = usart::RQR::Data;
};

constexpr usart::RQR::Data operator|(usart::RQR::Data left_a, usart::RQR::Data right_a) noexcept
{
    return static_cast<usart::RQR::Data>(static_cast<std::underlying_type_t<usart::RQR::Data>>(left_a) |
                                         static_cast<std::underlying_type_t<usart::RQR::Data>>(right_a));
}
constexpr usart::RQR::Data operator&(usart::RQR::Data left_a, usart::RQR::Data right_a) noexcept
{
    return static_cast<usart::RQR::Data>(static_cast<std::underlying_type_t<usart::RQR::Data>>(left_a) &
                                         static_cast<std::underlying_type_t<usart::RQR::Data>>(right_a));
}

// ISR
template<> struct bitmask_operators_traits<usart::ISR::Flag>
{
    using register_type = usart::ISR;
    using return_type = usart::ISR::Data;
};
template<> struct bitmask_operators_traits<usart::ISR::Data>
{
    using register_type = usart::ISR;
    using return_type = usart::ISR::Data;
};
template<> struct bitmask_operators_traits<usart::ISR>
{
    using register_type = usart::ISR;
    using return_type = usart::ISR::Data;
};

constexpr usart::ISR::Data operator|(usart::ISR::Data left_a, usart::ISR::Data right_a) noexcept
{
    return static_cast<usart::ISR::Data>(static_cast<std::underlying_type_t<usart::ISR::Data>>(left_a) |
                                         static_cast<std::underlying_type_t<usart::ISR::Data>>(right_a));
}
constexpr usart::ISR::Data operator&(usart::ISR::Data left_a, usart::ISR::Data right_a) noexcept
{
    return static_cast<usart::ISR::Data>(static_cast<std::underlying_type_t<usart::ISR::Data>>(left_a) &
                                         static_cast<std::underlying_type_t<usart::ISR::Data>>(right_a));
}

// ICR
template<> struct bitmask_operators_traits<usart::ICR::Flag>
{
    using register_type = usart::ICR;
    using return_type = usart::ICR::Data;
};
template<> struct bitmask_operators_traits<usart::ICR::Data>
{
    using register_type = usart::ICR;
    using return_type = usart::ICR::Data;
};

template<> struct bitmask_operators_traits<usart::ICR>
{
    using register_type = usart::ICR;
    using return_type = usart::ICR::Data;
};

constexpr usart::ICR::Data operator|(usart::ICR::Data left_a, usart::ICR::Data right_a) noexcept
{
    return static_cast<usart::ICR::Data>(static_cast<std::underlying_type_t<usart::ICR::Data>>(left_a) |
                                         static_cast<std::underlying_type_t<usart::ICR::Data>>(right_a));
}
constexpr usart::ICR::Data operator&(usart::ICR::Data left_a, usart::ICR::Data right_a) noexcept
{
    return static_cast<usart::ICR::Data>(static_cast<std::underlying_type_t<usart::ICR::Data>>(left_a) &
                                         static_cast<std::underlying_type_t<usart::ICR::Data>>(right_a));
}

} // namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll
