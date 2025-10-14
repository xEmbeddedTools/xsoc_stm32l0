#
# Copyright (c) xEmbeddedTools team and contributors.
# Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
#

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

target_compile_features(XMCU_SOC PRIVATE cxx_std_23)
target_compile_features(XMCU_SOC PRIVATE c_std_11)

target_include_directories(XMCU_SOC PUBLIC
                                    ${XMCU_PATH}
                                    ${CMAKE_CURRENT_LIST_DIR}
                                    "${CMAKE_CURRENT_LIST_DIR}/CMSIS/Device/ST/STM32L0xx/Include/"
                                    "${XMCU_PATH}/soc/st/arm/CMSIS/Include")
if (DEFINED XMCU_NOSTDLIB)
    if (${XMCU_NOSTDLIB} STREQUAL "ON")
        target_compile_definitions(XMCU_SOC PUBLIC XMCU_NOSTDLIB)
    endif()
    message(STATUS "XMCU_NOSTDLIB: ${XMCU_NOSTDLIB}")
else()
    message(FATAL_ERROR "XMCU_NOSTDLIB is not set (expected values: ON/OFF)")
endif()