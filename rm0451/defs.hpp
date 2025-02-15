#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

//TODO: move to config.hpp and unify related settings
#if defined(STM32L010x4)
#define STM32L010XX_CATEGORY 1
#elif defined(STM32L010x6)
#define STM32L010XX_CATEGORY 2
#elif defined(STM32L010x8)
#define STM32L010XX_CATEGORY 3
#elif defined(STM32L010xB)
#define STM32L010XX_CATEGORY 5
#else
#define STM32L010XX_CATEGORY 1
// #error "Unknown STM32L010XX_CATEGORY!"
#endif

#if STM32L010XX_CATEGORY == 1
#define STM32L010XX_DMA_CHANNEL_COUNT 5
#else
#define STM32L010XX_DMA_CHANNEL_COUNT 7
#endif

#if STM32L010XX_CATEGORY >= 2
#define STM32L010XX_HAS_GPIO_H
#endif

#if STM32L010XX_CATEGORY >= 3
#define STM32L010XX_HAS_GPIO_D
#endif

#if STM32L010XX_CATEGORY >= 5
#define STM32L010XX_HAS_GPIO_E
#define STM32L010XX_HAS_TIM22
#define STM32L010XX_HAS_USART2_SYNCHRONOUS
#define STM32L010XX_HAS_USART2_DUAL_CLOCK
#define STM32L010XX_HAS_USART2_RX_TIMEOUT
#define STM32L010XX_HAS_USART2_MODBUS
#define STM32L010XX_HAS_USART2_AUTO_BAUD
#define STM32L010XX_HAS_USART2_SMART_CARD
#endif
