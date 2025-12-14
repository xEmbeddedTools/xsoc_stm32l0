#pragma once

// xmcu
#include <xmcu/Non_copyable.hpp>
#include <xmcu/non_constructible.hpp>

// CMSIS
#include <stm32l0xx.h>

// std
#include <cstdint>
#include <type_traits>

#define XSOC_FLASH_LL_GENERATE_BITMASK_OPERATORS(ReturnEnumType, LeftEnumType, RightEnumType)            \
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

#define XSOC_FLASH_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(EnumType)            \
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

#define XSOC_FLASH_LL_GENERATE_BITMASK_UNARY_OPERATORS(EnumType)   \
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

#define XSOC_FLASH_LL_GENERATE_COMPARISON_OPERATORS(LeftEnumType, RightEnumType)            \
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
struct internal_flash : private xmcu::non_constructible
{
public:
    struct sr_descriptor : private xmcu::non_constructible
    {
    };

    struct optr_descriptor : private xmcu::non_constructible
    {
    };

    struct ACR
    {
        enum class Flag : std::uint32_t
        {
            latency = FLASH_ACR_LATENCY,
            prften = FLASH_ACR_PRFTEN,
            sleep_pd = FLASH_ACR_SLEEP_PD,
            run_pd = FLASH_ACR_RUN_PD,
            disab_buf = FLASH_ACR_DISAB_BUF,
            pre_read = FLASH_ACR_PRE_READ
        };

        using enum Flag;

        enum class Data : std::uint32_t;

        ACR& operator=(Flag value_a)
        {
            this->v = static_cast<Data>(value_a);
            return *this;
        }
        ACR& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }

        operator Data() const
        {
            return this->v;
        }

    private:
        Data v;
    };
    struct PECR
    {
        enum class Flag : std::uint32_t
        {
            pe_lock = FLASH_PECR_PELOCK,
            prg_lock = FLASH_PECR_PRGLOCK,
            prog = FLASH_PECR_PROG,
            opt_lock = FLASH_PECR_OPTLOCK,
            data = FLASH_PECR_DATA,
            fix = FLASH_PECR_FIX,
            erase = FLASH_PECR_ERASE,
            fprg = FLASH_PECR_FPRG,
            eopie = FLASH_PECR_EOPIE,
            errie = FLASH_PECR_ERRIE,
            obl_launch = FLASH_PECR_OBL_LAUNCH,
            nzdisable = 0x1u << 23u // not present in CMSIS
        };

        using enum Flag;

        enum class Data : std::uint32_t;

        PECR& operator=(Flag value_a)
        {
            this->v = static_cast<Data>(value_a);
            return *this;
        }
        PECR& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }

        operator Data() const
        {
            return this->v;
        }

    private:
        Data v;
    };
    struct PDKEYR
    {
        enum class Data : std::uint32_t;

        PDKEYR& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }
        PDKEYR& operator=(std::uint32_t value_a)
        {
            this->v = static_cast<Data>(value_a);
            return *this;
        }

        operator Data() const
        {
            return this->v;
        }

    private:
        Data v;
    };
    struct PEKEYR
    {
        enum class Data : std::uint32_t;

        PEKEYR& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }
        PEKEYR& operator=(std::uint32_t value_a)
        {
            this->v = static_cast<Data>(value_a);
            return *this;
        }

        operator Data() const
        {
            return this->v;
        }

    private:
        Data v;
    };
    struct OPTKEYR
    {
        enum class Data : std::uint32_t;

        OPTKEYR& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }
        OPTKEYR& operator=(std::uint32_t value_a)
        {
            this->v = static_cast<Data>(value_a);
            return *this;
        }

        operator Data() const
        {
            return this->v;
        }

    private:
        Data v;
    };
    struct SR
    {
        enum class Flag : std::uint32_t
        {
            bsy = FLASH_SR_BSY,
            eop = FLASH_SR_EOP,
            endhv = FLASH_SR_ENDHV,
            ready = FLASH_SR_READY,
            wrperr = FLASH_SR_WRPERR,
            pgaerr = FLASH_SR_PGAERR,
            sizerr = FLASH_SR_SIZERR,
            optverr = FLASH_SR_OPTVERR,
            rderr = FLASH_SR_RDERR,
            notzeroerr = FLASH_SR_NOTZEROERR,
            fwwer = FLASH_SR_FWWER
        };
        using enum Flag;

        enum class Data : std::uint32_t;

        SR& operator=(Flag value_a)
        {
            this->v = static_cast<Data>(value_a);
            return *this;
        }
        SR& operator=(Data value_a)
        {
            this->v = value_a;
            return *this;
        }

        operator Data() const
        {
            return this->v;
        }

    private:
        Data v;
    };
    struct OPTR
    {
        struct BOR : private xmcu::Non_copyable
        {
            enum class Flag : std::uint32_t
            {
                off,
                _1,
                _2,
                _3,
                _4,
                _5
            };
        } static bor;
    };
};

// ACR
XSOC_FLASH_LL_GENERATE_BITMASK_OPERATORS(internal_flash::ACR::Data,
                                         internal_flash::ACR::Flag,
                                         internal_flash::ACR::Flag);
XSOC_FLASH_LL_GENERATE_BITMASK_OPERATORS(internal_flash::ACR::Data,
                                         internal_flash::ACR::Data,
                                         internal_flash::ACR::Data);
XSOC_FLASH_LL_GENERATE_BITMASK_OPERATORS(internal_flash::ACR::Data,
                                         internal_flash::ACR::Data,
                                         internal_flash::ACR::Flag);
XSOC_FLASH_LL_GENERATE_BITMASK_OPERATORS(internal_flash::ACR::Data,
                                         internal_flash::ACR::Flag,
                                         internal_flash::ACR::Data);

XSOC_FLASH_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(internal_flash::ACR::Flag);
XSOC_FLASH_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(internal_flash::ACR::Data);

XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS(internal_flash::ACR::Flag);
XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS(internal_flash::ACR::Data);
XSOC_USART_LL_GENERATE_COMPARISON_OPERATORS(internal_flash::ACR::Flag, internal_flash::ACR::Data);

// PECR
XSOC_FLASH_LL_GENERATE_BITMASK_OPERATORS(internal_flash::PECR::Data,
                                         internal_flash::PECR::Flag,
                                         internal_flash::PECR::Flag);
XSOC_FLASH_LL_GENERATE_BITMASK_OPERATORS(internal_flash::PECR::Data,
                                         internal_flash::PECR::Data,
                                         internal_flash::PECR::Data);
XSOC_FLASH_LL_GENERATE_BITMASK_OPERATORS(internal_flash::PECR::Data,
                                         internal_flash::PECR::Data,
                                         internal_flash::PECR::Flag);
XSOC_FLASH_LL_GENERATE_BITMASK_OPERATORS(internal_flash::PECR::Data,
                                         internal_flash::PECR::Flag,
                                         internal_flash::PECR::Data);

XSOC_FLASH_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(internal_flash::PECR::Flag);
XSOC_FLASH_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(internal_flash::PECR::Data);

XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS(internal_flash::PECR::Flag);
XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS(internal_flash::PECR::Data);
XSOC_USART_LL_GENERATE_COMPARISON_OPERATORS(internal_flash::PECR::Flag, internal_flash::PECR::Data);

// SR
XSOC_FLASH_LL_GENERATE_BITMASK_OPERATORS(internal_flash::SR::Data, internal_flash::SR::Flag, internal_flash::SR::Flag);
XSOC_FLASH_LL_GENERATE_BITMASK_OPERATORS(internal_flash::SR::Data, internal_flash::SR::Data, internal_flash::SR::Data);
XSOC_FLASH_LL_GENERATE_BITMASK_OPERATORS(internal_flash::SR::Data, internal_flash::SR::Data, internal_flash::SR::Flag);
XSOC_FLASH_LL_GENERATE_BITMASK_OPERATORS(internal_flash::SR::Data, internal_flash::SR::Flag, internal_flash::SR::Data);

XSOC_FLASH_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(internal_flash::SR::Flag);
XSOC_FLASH_LL_GENERATE_BITMASK_ASSIGMENT_OPERATORS(internal_flash::SR::Data);

XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS(internal_flash::SR::Flag);
XSOC_USART_LL_GENERATE_BITMASK_UNARY_OPERATORS(internal_flash::SR::Data);
XSOC_USART_LL_GENERATE_COMPARISON_OPERATORS(internal_flash::SR::Flag, internal_flash::SR::Data);
} // namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll