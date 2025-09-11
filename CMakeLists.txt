#
# Copyright (c) xEmbeddedTools team and contributors.
# Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
#

cmake_minimum_required(VERSION 3.19.0)

if (XMCU_SOC_MODEL STREQUAL "XMCU_SOC_MODEL_STM32L010F4P6")
    file(GLOB_RECURSE XMCU_SOC_SRC
            "${CMAKE_CURRENT_LIST_DIR}/rm0451/*.cpp"
            "${CMAKE_CURRENT_LIST_DIR}/rm0451/*.c"
            "${CMAKE_CURRENT_LIST_DIR}/rm0451/*.asm"
            "${CMAKE_CURRENT_LIST_DIR}/rm0451/*.hpp"
            "${CMAKE_CURRENT_LIST_DIR}/rm0451/*.h"
            "${CMAKE_CURRENT_LIST_DIR}/CMSIS/Device/ST/STM32L0xx/Source/Templates/gcc/startup_stm32l010x4.s"
            "${CMAKE_CURRENT_LIST_DIR}/CMSIS/Device/ST/STM32L0xx/Source/Templates/system_stm32l0xx.c")
    set(XMCU_LD_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/CMSIS/Device/ST/STM32L0xx/Source/Templates/gcc/linker/STM32L010F4Px_FLASH.ld")
    add_library(XMCU_SOC OBJECT ${XMCU_SOC_SRC})
    add_compile_definitions("XMCU_SOC" "XMCU_SOC_MODEL_STM32L010F4P6" "STM32L010x4")
elseif(XMCU_SOC_MODEL STREQUAL "XMCU_SOC_MODEL_STM32L010C6T6")
    file(GLOB_RECURSE XMCU_SOC_SRC
            "${CMAKE_CURRENT_LIST_DIR}/rm0451/*.cpp"
            "${CMAKE_CURRENT_LIST_DIR}/rm0451/*.c"
            "${CMAKE_CURRENT_LIST_DIR}/rm0451/*.asm"
            "${CMAKE_CURRENT_LIST_DIR}/rm0451/*.hpp"
            "${CMAKE_CURRENT_LIST_DIR}/rm0451/*.h"
            "${CMAKE_CURRENT_LIST_DIR}/CMSIS/Device/ST/STM32L0xx/Source/Templates/gcc/startup_stm32l010x6.s"
            "${CMAKE_CURRENT_LIST_DIR}/CMSIS/Device/ST/STM32L0xx/Source/Templates/system_stm32l0xx.c")
    set(XMCU_LD_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/CMSIS/Device/ST/STM32L0xx/Source/Templates/gcc/linker/STM32L010C6Tx_FLASH.ld")
    add_library(XMCU_SOC OBJECT ${XMCU_SOC_SRC})
    add_compile_definitions("XMCU_SOC_MODEL_STM32L010C6T6" "STM32L010x6")
else()
    message("Unsupported SOC model")
endif()

set(XMCU_COMPILE_FLAGS ${XMCU_COMPILE_FLAGS} -mcpu=cortex-m0plus -mthumb)
set(XMCU_LINKER_FLAGS ${XMCU_LINKER_FLAGS} -mcpu=cortex-m0plus -mthumb)

if (${XMCU_NOSTDLIB} STREQUAL "ON")
    message("XMCU: no stdlib")
    set(XMCU_COMPILE_FLAGS ${XMCU_COMPILE_FLAGS} -nostdlib -nostartfiles)
    set(XMCU_LINKER_FLAGS ${XMCU_LINKER_FLAGS} -nostdlib -nostartfiles --specs=nosys.specs -lc_nano)
endif()

include_directories(
                    ${XMCU_PATH}
                    ${CMAKE_CURRENT_LIST_DIR}
                    "${XMCU_PATH}/soc/st/arm/CMSIS/Include"
                    "${CMAKE_CURRENT_LIST_DIR}/CMSIS/Device/ST/STM32L0xx/Include")
