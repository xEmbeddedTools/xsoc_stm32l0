#pragma once

// xmcu
#include <xmcu/non_constructible.hpp>
#include <xmcu/Non_copyable.hpp>

// CMSIS
#include <stm32l0xx.h>

// std
#include <cstdint>

namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll {
struct internal_flash : private xmcu::non_constructible
{
public:
    struct acr_descriptor : private xmcu::non_constructible
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
    };

    struct pecr_descriptor : private xmcu::non_constructible
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
    };

    struct pdkeyr_descriptor : private xmcu::non_constructible
    {
        using Data = std::uint32_t;
    };

    struct pekeyr_descriptor : private xmcu::non_constructible
    {
        using Data = std::uint32_t;
    };

    struct prgkeyr_descriptor : private xmcu::non_constructible
    {
        using Data = std::uint32_t;
    };

    struct optkeyr_descriptor : private xmcu::non_constructible
    {
        using Data = std::uint32_t;
    };

    struct sr_descriptor : private xmcu::non_constructible
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
    };

    struct optr_descriptor : private xmcu::non_constructible
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
        }static bor;
    };
};
} // namespace xmcu::soc::st::arm::m0::l0::rm0451::peripherals::ll
