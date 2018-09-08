/*
 * Copyright (c) 2013 Ambroz Bizjak
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef AMBROLIB_STM32_PINS_H
#define AMBROLIB_STM32_PINS_H

#include <stdint.h>

#include <aprinter/meta/ServiceUtils.h>
#include <aprinter/base/Object.h>
#include <aprinter/base/DebugObject.h>
#include <aprinter/base/Lock.h>
#include <aprinter/system/InterruptLock.h>

namespace APrinter {

template <uint32_t TGpioAddr>
struct Stm32Port {
    static GPIO_TypeDef * gpio () { return (GPIO_TypeDef *)TGpioAddr; }
};

APRINTER_IF_DEFINED(GPIOA_BASE)(using Stm32PortA = Stm32Port<GPIOA_BASE>;)
APRINTER_IF_DEFINED(GPIOB_BASE)(using Stm32PortB = Stm32Port<GPIOB_BASE>;)
APRINTER_IF_DEFINED(GPIOC_BASE)(using Stm32PortC = Stm32Port<GPIOC_BASE>;)
APRINTER_IF_DEFINED(GPIOD_BASE)(using Stm32PortD = Stm32Port<GPIOD_BASE>;)
APRINTER_IF_DEFINED(GPIOE_BASE)(using Stm32PortE = Stm32Port<GPIOE_BASE>;)
APRINTER_IF_DEFINED(GPIOF_BASE)(using Stm32PortF = Stm32Port<GPIOF_BASE>;)
APRINTER_IF_DEFINED(GPIOG_BASE)(using Stm32PortG = Stm32Port<GPIOG_BASE>;)
APRINTER_IF_DEFINED(GPIOH_BASE)(using Stm32PortH = Stm32Port<GPIOH_BASE>;)
APRINTER_IF_DEFINED(GPIOI_BASE)(using Stm32PortI = Stm32Port<GPIOI_BASE>;)
APRINTER_IF_DEFINED(GPIOJ_BASE)(using Stm32PortJ = Stm32Port<GPIOJ_BASE>;)
APRINTER_IF_DEFINED(GPIOK_BASE)(using Stm32PortK = Stm32Port<GPIOK_BASE>;)

template <typename TPort, int TPinIndex>
struct Stm32Pin {
    using Port = TPort;
    static const int PinIndex = TPinIndex;
};

template <uint8_t TPupdr>
struct Stm32PinPullMode {
    static uint8_t const Pupdr = TPupdr;
};
using Stm32PinPullModeNone = Stm32PinPullMode<0>;
using Stm32PinPullModePullUp = Stm32PinPullMode<1>;
using Stm32PinPullModePullDown = Stm32PinPullMode<2>;

template <uint8_t TOptyper>
struct Stm32PinOutputType {
    static uint8_t const Optyper = TOptyper;
};
using Stm32PinOutputTypeNormal = Stm32PinOutputType<0>;
using Stm32PinOutputTypeOpenDrain = Stm32PinOutputType<1>;

template <uint8_t TOspeedr>
struct Stm32PinOutputSpeed {
    static uint8_t const Ospeedr = TOspeedr;
};
using Stm32PinOutputSpeedLow = Stm32PinOutputSpeed<0>;
using Stm32PinOutputSpeedMedium = Stm32PinOutputSpeed<1>;
using Stm32PinOutputSpeedFast = Stm32PinOutputSpeed<2>;
using Stm32PinOutputSpeedHigh = Stm32PinOutputSpeed<3>;

template <typename PullMode>
struct Stm32PinInputMode {
    static uint8_t const Pupdr = PullMode::Pupdr;
};
using Stm32PinInputModeNormal = Stm32PinInputMode<Stm32PinPullModeNone>;
using Stm32PinInputModePullUp = Stm32PinInputMode<Stm32PinPullModePullUp>;
using Stm32PinInputModePullDown = Stm32PinInputMode<Stm32PinPullModePullDown>;

template <typename OutputType, typename OutputSpeed, typename PullMode>
struct Stm32PinOutputMode {
    static uint8_t const Optyper = OutputType::Optyper;
    static uint8_t const Ospeedr = OutputSpeed::Ospeedr;
    static uint8_t const Pupdr = PullMode::Pupdr;
};
using Stm32PinOutputModeNormal = Stm32PinOutputMode<Stm32PinOutputTypeNormal, Stm32PinOutputSpeedLow, Stm32PinPullModeNone>;
using Stm32PinOutputModeOpenDrain = Stm32PinOutputMode<Stm32PinOutputTypeOpenDrain, Stm32PinOutputSpeedLow, Stm32PinPullModeNone>;

template <typename Arg>
class Stm32Pins {
    using Context      = typename Arg::Context;
    using ParentObject = typename Arg::ParentObject;
    
public:
    struct Object;
    
private:
    using TheDebugObject = DebugObject<Context, Object>;
    
public:
    static void init (Context c)
    {
        APRINTER_IF_DEFINED(GPIOA_BASE)(__HAL_RCC_GPIOA_CLK_ENABLE());
        APRINTER_IF_DEFINED(GPIOB_BASE)(__HAL_RCC_GPIOB_CLK_ENABLE());
        APRINTER_IF_DEFINED(GPIOC_BASE)(__HAL_RCC_GPIOC_CLK_ENABLE());
        APRINTER_IF_DEFINED(GPIOD_BASE)(__HAL_RCC_GPIOD_CLK_ENABLE());
        APRINTER_IF_DEFINED(GPIOE_BASE)(__HAL_RCC_GPIOE_CLK_ENABLE());
        APRINTER_IF_DEFINED(GPIOF_BASE)(__HAL_RCC_GPIOF_CLK_ENABLE());
        APRINTER_IF_DEFINED(GPIOG_BASE)(__HAL_RCC_GPIOG_CLK_ENABLE());
        APRINTER_IF_DEFINED(GPIOH_BASE)(__HAL_RCC_GPIOH_CLK_ENABLE());
        APRINTER_IF_DEFINED(GPIOI_BASE)(__HAL_RCC_GPIOI_CLK_ENABLE());
        APRINTER_IF_DEFINED(GPIOJ_BASE)(__HAL_RCC_GPIOJ_CLK_ENABLE());
        APRINTER_IF_DEFINED(GPIOK_BASE)(__HAL_RCC_GPIOK_CLK_ENABLE());
        
        TheDebugObject::init(c);
    }
    
    static void deinit (Context c)
    {
        TheDebugObject::deinit(c);
        
        APRINTER_IF_DEFINED(GPIOK_BASE)(__HAL_RCC_GPIOK_CLK_DISABLE());
        APRINTER_IF_DEFINED(GPIOJ_BASE)(__HAL_RCC_GPIOJ_CLK_DISABLE());
        APRINTER_IF_DEFINED(GPIOI_BASE)(__HAL_RCC_GPIOI_CLK_DISABLE());
        APRINTER_IF_DEFINED(GPIOH_BASE)(__HAL_RCC_GPIOH_CLK_DISABLE());
        APRINTER_IF_DEFINED(GPIOG_BASE)(__HAL_RCC_GPIOG_CLK_DISABLE());
        APRINTER_IF_DEFINED(GPIOF_BASE)(__HAL_RCC_GPIOF_CLK_DISABLE());
        APRINTER_IF_DEFINED(GPIOE_BASE)(__HAL_RCC_GPIOE_CLK_DISABLE());
        APRINTER_IF_DEFINED(GPIOD_BASE)(__HAL_RCC_GPIOD_CLK_DISABLE());
        APRINTER_IF_DEFINED(GPIOC_BASE)(__HAL_RCC_GPIOC_CLK_DISABLE());
        APRINTER_IF_DEFINED(GPIOB_BASE)(__HAL_RCC_GPIOB_CLK_DISABLE());
        APRINTER_IF_DEFINED(GPIOA_BASE)(__HAL_RCC_GPIOA_CLK_DISABLE());
    }
    
    template <typename Pin, typename Mode = Stm32PinInputModeNormal, typename ThisContext>
    static void setInput (ThisContext c)
    {
        TheDebugObject::access(c);
        
        AMBRO_LOCK_T(InterruptTempLock(), c, lock_c) {
            set_pupdr<Pin, Mode::Pupdr>();
            set_moder<Pin, 0>();
        }
    }
    
    template <typename Pin, typename Mode = Stm32PinOutputModeNormal, typename ThisContext>
    static void setOutput (ThisContext c)
    {
        TheDebugObject::access(c);
        
        AMBRO_LOCK_T(InterruptTempLock(), c, lock_c) {
            set_pupdr<Pin, Mode::Pupdr>();
            set_optyper<Pin, Mode::Optyper>();
            set_ospeedr<Pin, Mode::Ospeedr>();
            set_moder<Pin, 1>();
        }
    }
    
    template <typename Pin, int AfNumber, typename Mode = Stm32PinOutputModeNormal, typename ThisContext>
    static void setAlternateFunction (ThisContext c)
    {
        TheDebugObject::access(c);
        
        AMBRO_LOCK_T(InterruptTempLock(), c, lock_c) {
            set_pupdr<Pin, Mode::Pupdr>();
            set_optyper<Pin, Mode::Optyper>();
            set_ospeedr<Pin, Mode::Ospeedr>();
            set_af<Pin, AfNumber>();
            set_moder<Pin, 2>();
        }
    }
    
    template <typename Pin, typename ThisContext>
    static void setAnalog (ThisContext c)
    {
        TheDebugObject::access(c);
        
        AMBRO_LOCK_T(InterruptTempLock(), c, lock_c) {
            set_pupdr<Pin, 0>();
            set_moder<Pin, 3>();
        }
    }
    
    template <typename Pin, typename ThisContext>
    static bool get (ThisContext c)
    {
        TheDebugObject::access(c);
        
        return (Pin::Port::gpio()->IDR & (UINT32_C(1) << Pin::PinIndex));
    }
    
    template <typename Pin, typename ThisContext>
    static void set (ThisContext c, bool x)
    {
        TheDebugObject::access(c);
        
        if (x) {
            Pin::Port::gpio()->BSRR = (UINT32_C(1) << Pin::PinIndex);
        } else {
            Pin::Port::gpio()->BSRR = (UINT32_C(1) << (16 + Pin::PinIndex));
        }
    }
    
    template <typename Pin>
    static void emergencySet (bool x)
    {
        if (x) {
            Pin::Port::gpio()->BSRR = (UINT32_C(1) << Pin::PinIndex);
        } else {
            Pin::Port::gpio()->BSRR = (UINT32_C(1) << (16 + Pin::PinIndex));
        }
    }
    
private:
    template <typename Pin, uint8_t Value>
    static void set_moder ()
    {
        Pin::Port::gpio()->MODER = set_bits(Pin::Port::gpio()->MODER, 2 * Pin::PinIndex, 2, Value);
    }
    
    template <typename Pin, uint8_t Value>
    static void set_pupdr ()
    {
        Pin::Port::gpio()->PUPDR = set_bits(Pin::Port::gpio()->PUPDR, 2 * Pin::PinIndex, 2, Value);
    }
    
    template <typename Pin, uint8_t Value>
    static void set_optyper ()
    {
        Pin::Port::gpio()->OTYPER = set_bits(Pin::Port::gpio()->OTYPER, Pin::PinIndex, 1, Value);
    }
    
    template <typename Pin, uint8_t Value>
    static void set_ospeedr ()
    {
        Pin::Port::gpio()->OSPEEDR = set_bits(Pin::Port::gpio()->OSPEEDR, 2 * Pin::PinIndex, 2, Value);
    }
    
    template <typename Pin, uint8_t Value>
    static void set_af ()
    {
        Pin::Port::gpio()->AFR[Pin::PinIndex / 8] = set_bits(Pin::Port::gpio()->AFR[Pin::PinIndex / 8], 4 * (Pin::PinIndex % 8), 4, Value);
    }
    
    static uint32_t set_bits (uint32_t orig, int offset, int bits, uint32_t val)
    {
        return (orig & (uint32_t)~(((uint32_t)-1 >> (32 - bits)) << offset)) | (uint32_t)(val << offset);
    }
    
public:
    struct Object : public ObjBase<Stm32Pins, ParentObject, MakeTypeList<TheDebugObject>> {};
};

struct Stm32PinsService {
    APRINTER_ALIAS_STRUCT_EXT(Pins, (
        APRINTER_AS_TYPE(Context),
        APRINTER_AS_TYPE(ParentObject)
    ), (
        APRINTER_DEF_INSTANCE(Pins, Stm32Pins)
    ))
};

}

#endif
