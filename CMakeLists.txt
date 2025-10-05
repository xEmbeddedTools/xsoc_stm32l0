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
    add_library(XMCU_SOC OBJECT ${XMCU_SOC_SRC})
    target_compile_definitions(XMCU_SOC PUBLIC "XMCU_SOC_MODEL_STM32L010F4P6" "STM32L010x4")
elseif(XMCU_SOC_MODEL STREQUAL "XMCU_SOC_MODEL_STM32L010C6T6")
    file(GLOB_RECURSE XMCU_SOC_SRC
            "${CMAKE_CURRENT_LIST_DIR}/rm0451/*.cpp"
            "${CMAKE_CURRENT_LIST_DIR}/rm0451/*.c"
            "${CMAKE_CURRENT_LIST_DIR}/rm0451/*.asm"
            "${CMAKE_CURRENT_LIST_DIR}/rm0451/*.hpp"
            "${CMAKE_CURRENT_LIST_DIR}/rm0451/*.h"
            "${CMAKE_CURRENT_LIST_DIR}/CMSIS/Device/ST/STM32L0xx/Source/Templates/gcc/startup_stm32l010x6.s"
            "${CMAKE_CURRENT_LIST_DIR}/CMSIS/Device/ST/STM32L0xx/Source/Templates/system_stm32l0xx.c")
    add_library(XMCU_SOC OBJECT ${XMCU_SOC_SRC})
    target_compile_definitions(XMCU_SOC PUBLIC "XMCU_SOC_MODEL_STM32L010C6T6" "STM32L010x6")
else()
    message("Unsupported SOC model")
endif()
target_compile_features(XMCU_SOC PUBLIC cxx_std_23)
target_compile_features(XMCU_SOC PUBLIC c_std_11)
target_include_directories(XMCU_SOC PUBLIC
                                    ${XMCU_PATH}
                                    ${CMAKE_CURRENT_LIST_DIR}
                                    "${CMAKE_CURRENT_LIST_DIR}/CMSIS/Device/ST/STM32L0xx/Include/"
                                    "${XMCU_PATH}/soc/st/arm/CMSIS/Include")

target_compile_options(XMCU_SOC PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-mcpu=cortex-m0plus -mthumb>
                                       $<$<COMPILE_LANGUAGE:C>:-mcpu=cortex-m0plus -mthumb>
                                       $<$<COMPILE_LANGUAGE:ASM>:-mcpu=cortex-m0plus -mthumb>)

if (${XMCU_NOSTDLIB} STREQUAL "ON")
    message("XMCU: no stdlib")
    target_compile_options(XMCU_SOC PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-nostdlib -nostartfiles>
                                           $<$<COMPILE_LANGUAGE:C>:-nostdlib -nostartfiles>
                                           $<$<COMPILE_LANGUAGE:ASM>:-nostdlib -nostartfiles>)
endif()