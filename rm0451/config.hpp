#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

#define XMCU_SOC_VENDOR_ST st
#define XMCU_SOC_ARCHITECTURE_ARM arm
#define XMCU_SOC_CORE_FAMILY_M0 m0
#define XMCU_SOC_VENDOR_FAMILY_L0 l0

#define XMCU_SOC_VENDOR XMCU_SOC_VENDOR_ST
#define XMCU_SOC_ARCHITECTURE XMCU_SOC_ARCHITECTURE_ARM
#define XMCU_SOC_CORE_FAMILY XMCU_SOC_CORE_FAMILY_M0
#define XMCU_SOC_VENDOR_FAMILY XMCU_SOC_VENDOR_FAMILY_L0

#if defined(XMCU_SOC_MODEL_STM32L010F4P6)
// GPIO
#define XMCU_GPIOA_PRESENT
#define XMCU_GPIOB_PRESENT
#define XMCU_GPIOC_PRESENT

// clock sources
#define XMCU_HSE_PRESENT
#define XMCU_HSI16_PRESENT
#define XMCU_MSI_PRESENT

// clocks
#define XMCU_HCLK_PRESENT
#define XMCU_PCLK_PRESENT
#define XMCU_SYSCLK_PRESENT

#define XMCU_SOC_VENDOR_FAMILY_RM0451 rm0451
#define XMCU_SOC_VENDOR_FAMILY_RM XMCU_SOC_VENDOR_FAMILY_RM0451
#endif

#if defined(XMCU_SOC_MODEL_STM32L010C6T6)
// GPIO
#define XMCU_GPIOA_PRESENT
#define XMCU_GPIOB_PRESENT
#define XMCU_GPIOC_PRESENT
#define XMCU_GPIOH_PRESENT

// clock sources
#define XMCU_HSE_PRESENT
#define XMCU_HSI16_PRESENT
#define XMCU_MSI_PRESENT

// clocks
#define XMCU_HCLK_PRESENT
#define XMCU_PCLK_PRESENT
#define XMCU_SYSCLK_PRESENT

#define XMCU_SOC_VENDOR_FAMILY_RM0451 rm0451
#define XMCU_SOC_VENDOR_FAMILY_RM XMCU_SOC_VENDOR_FAMILY_RM0451
#endif