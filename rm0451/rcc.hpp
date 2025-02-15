#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// std
#include <cstddef>
#include <cstdint>
#include <limits>

// xmcu
#include <xmcu/non_constructible.hpp>

namespace xmcu::soc::st::arm::m0::l0::rm0451 {
template<typename Periph_t, std::uint32_t id = std::numeric_limits<std::uint32_t>::max()> class rcc
    : private xmcu::non_constructible
{
};
} // namespace xmcu::soc::st::arm::m0::l0::rm0451