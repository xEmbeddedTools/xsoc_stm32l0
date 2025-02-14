/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// this
#include <rm0451/utils/wait_until.hpp>

namespace xmcu::soc::st::arm::m0::l0::rm0451::utils {
using namespace xmcu;

//void wait_until::all_bits_are_set(volatile const std::uint32_t& a_register, uint32_t a_mask)
//{
//    while (false == bit::flag::is(a_register, a_mask)) continue;
//}

//void wait_until::any_bit_is_set(volatile const std::uint32_t& a_register, uint32_t a_mask)
//{
//    while (false == bit::is_any(a_register, a_mask)) continue;
//}

//void wait_until::masked_bits_are_set(const std::uint32_t& a_register, uint32_t a_mask, uint32_t a_value)
//{
//    while (bit::flag::get(a_register, a_mask) != a_value) continue;
//}

//void wait_until::all_bits_are_cleared(const std::uint32_t& a_register, uint32_t a_mask)
//{
//    while (false == bit::flag::is(~a_register, a_mask)) continue;
//}

//void wait_until::any_bit_is_cleared(const std::uint32_t& a_register, uint32_t a_mask)
//{
//    while (false == bit::is_any(~a_register, a_mask)) continue;
//}

//bool wait_until::all_bits_are_set(volatile const std::uint32_t& a_register, uint32_t a_mask, Milliseconds a_timeout)
//{
//    const std::uint64_t timeout_end = tick_counter<Milliseconds>::get() + a_timeout.get();
//    bool status = false;
//
//    while (tick_counter<Milliseconds>::get() < timeout_end && false == status)
//    {
//        status = bit::flag::is(a_register, a_mask);
//    }
//
//    return status;
//}

//bool wait_until::masked_bits_are_set(const std::uint32_t& a_register,
//                                     uint32_t a_mask,
//                                     uint32_t a_value,
//                                     Milliseconds a_timeout)
//{
//    const std::uint64_t timeout_end = tick_counter<Milliseconds>::get() + a_timeout.get();
//    bool status = false;
//
//    while (tick_counter<Milliseconds>::get() < timeout_end && false == status)
//    {
//        status = (bit::flag::get(a_register, a_mask) == a_value);
//    }
//
//    return status;
//}

//bool wait_until::any_bit_is_set(const std::uint32_t& a_register, uint32_t a_mask, Milliseconds a_timeout)
//{
//    const std::uint64_t timeout_end = tick_counter<Milliseconds>::get() + a_timeout.get();
//    bool status = false;
//
//    while (tick_counter<Milliseconds>::get() < timeout_end && false == status)
//    {
//        status = bit::is_any(a_register, a_mask);
//    }
//
//    return status;
//}

//bool wait_until::all_bits_are_cleared(const std::uint32_t& a_register, uint32_t a_mask, Milliseconds a_timeout)
//{
//    const std::uint64_t timeout_end = tick_counter<Milliseconds>::get() + a_timeout.get();
//    bool status = false;
//
//    while (tick_counter<Milliseconds>::get() < timeout_end && false == status)
//    {
//        status = bit::flag::is(~a_register, a_mask);
//    }
//
//    return status;
//}

//bool wait_until::any_bit_is_cleared(const std::uint32_t& a_register, uint32_t a_mask, Milliseconds a_timeout)
//{
//    const std::uint64_t timeout_end = tick_counter<Milliseconds>::get() + a_timeout.get();
//    bool status = false;
//
//    while (tick_counter<Milliseconds>::get() < timeout_end && status == false)
//    {
//        status = bit::is_any(~a_register, a_mask);
//    }
//
//    return status;
//}
} // namespace xmcu::soc::st::arm::m0::l0::rm0451::utils