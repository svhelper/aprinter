#!/usr/bin/env bash
# 
# Copyright (c) 2014 Bernard `Guyzmo` Pratz
# Copyright (c) 2017 Ambroz Bizjak
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 
#####################################################################################
# STM32 SPECIFIC STUFF

configure_stm32() {
    CMSIS_DIR=${STM32CUBEF4_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx
    TEMPLATES_DIR=${CMSIS_DIR}/Source/Templates
    HAL_DIR=${STM32CUBEF4_DIR}/Drivers/STM32F4xx_HAL_Driver
    USB_DIR=${STM32CUBEF4_DIR}/Middlewares/ST/STM32_USB_Device_Library

    LINKER_SCRIPT=${ROOT}/aprinter/platform/stm32/stm32.ld
    
    configure_arm
    
    USB_FLAGS=()
    USB_C_SOURCES=()
    if [[ -n $USB_MODE ]]; then
        USB_FLAGS=( -DAPRINTER_ENABLE_USB )
        USB_C_SOURCES=(
            "${HAL_DIR}/Src/stm32f4xx_hal_pcd.c"
            "${HAL_DIR}/Src/stm32f4xx_hal_pcd_ex.c"
            "${HAL_DIR}/Src/stm32f4xx_ll_usb.c"
            "${USB_DIR}/Core/Src/usbd_core.c"
            "${USB_DIR}/Core/Src/usbd_ctlreq.c"
            "${USB_DIR}/Core/Src/usbd_ioreq.c"
            "${USB_DIR}/Class/CDC/Src/usbd_cdc.c"
            "${ROOT}/aprinter/platform/stm32/usbd_conf.c"
            "${ROOT}/aprinter/platform/stm32/usbd_desc.c"
        )
        
        if [[ $USB_MODE = "FS" ]]; then
            USB_FLAGS=( "${USB_FLAGS[@]}" -DUSE_USB_FS )
        elif [[ $USB_MODE = "HS" ]]; then
            USB_FLAGS=( "${USB_FLAGS[@]}" -DUSE_USB_HS )
        elif [[ $USB_MODE = "HS-in-FS" ]]; then
            USB_FLAGS=( "${USB_FLAGS[@]}" -DUSE_USB_HS -DUSE_USB_HS_IN_FS )
        else
            fail "Invalid USB_MODE"
        fi
    fi
    
    SDCARD_C_SOURCES=()
    if [[ -n $ENABLE_SDCARD ]]; then
        SDCARD_C_SOURCES=(
            "${HAL_DIR}/Src/stm32f4xx_hal_sd.c"
            "${HAL_DIR}/Src/stm32f4xx_ll_sdmmc.c"
        )
    fi
    
    if [[ $ARM_CPU = "cortex-m4" ]]; then
        FLAGS_C_CXX_LD+=(
            -mfpu=fpv4-sp-d16 -mfloat-abi=hard
        )
    fi

    FLAGS_C_CXX+=(
        "${CHIP_FLAGS[@]}"
        -DUSE_HAL_DRIVER -DHEAP_SIZE=16384
        "-DPLL_N_VALUE=${PLL_N_VALUE}" "-DPLL_M_VALUE=${PLL_M_VALUE}"
        "-DPLL_P_DIV_VALUE=${PLL_P_DIV_VALUE}" "-DPLL_Q_DIV_VALUE=${PLL_Q_DIV_VALUE}"
        "-DAPB1_PRESC_DIV=${APB1_PRESC_DIV}" "-DAPB2_PRESC_DIV=${APB2_PRESC_DIV}"
        "${USB_FLAGS[@]}"
        -I "${ROOT}/aprinter/platform/stm32"
        -I "${CMSIS_DIR}/Include"
        -I "${STM32CUBEF4_DIR}/Drivers/CMSIS/Include"
        -I "${HAL_DIR}/Inc"
        -I "${USB_DIR}/Core/Inc"
        -I "${USB_DIR}/Class/CDC/Inc"
    )
    
    CXX_SOURCES+=(
        "${ROOT}/aprinter/platform/stm32/stm32_support.cpp"
    )
    C_SOURCES+=(
        "${TEMPLATES_DIR}/system_stm32f4xx.c"
        "${HAL_DIR}/Src/stm32f4xx_hal.c"
        "${HAL_DIR}/Src/stm32f4xx_hal_cortex.c"
        "${HAL_DIR}/Src/stm32f4xx_hal_rcc.c"
        "${HAL_DIR}/Src/stm32f4xx_hal_iwdg.c"
        "${HAL_DIR}/Src/stm32f4xx_hal_gpio.c"
        "${HAL_DIR}/Src/stm32f4xx_hal_dma.c"
        "${ROOT}/aprinter/platform/newlib_common.c"
        "${USB_C_SOURCES[@]}" "${SDCARD_C_SOURCES[@]}"
    )
    ASM_SOURCES+=(
        "${ROOT}/aprinter/platform/stm32/startup_stm32.s"
    )

    # define target functions
    RUNBUILD=build_stm32
    CHECK=check_depends_stm32
}

build_stm32() {
    build_arm
}

check_depends_stm32() {
    check_depends_arm
    [ -d "${STM32CUBEF4_DIR}" ] || fail "STM32 framework missing in dependences"
}
