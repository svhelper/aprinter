/*
 * Copyright (c) 2013 Ambroz Bizjak
 * Copyright (c) 2018 BOBAH
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

#ifndef AMBROLIB_STM32F2_PINS_H
#define AMBROLIB_STM32F2_PINS_H

#include <stdint.h>

#include <aprinter/meta/ServiceUtils.h>
#include <aprinter/base/Object.h>
#include <aprinter/base/DebugObject.h>
#include <aprinter/base/Lock.h>
#include <aprinter/system/InterruptLock.h>

namespace APrinter {

template <uint32_t TGpioAddr>
struct Stm32f2Port {
    static GPIO_TypeDef * gpio () { return (GPIO_TypeDef *)TGpioAddr; }
};

using Stm32f2PortA = Stm32f2Port<GPIOA_BASE>;
using Stm32f2PortB = Stm32f2Port<GPIOB_BASE>;
using Stm32f2PortC = Stm32f2Port<GPIOC_BASE>;
using Stm32f2PortD = Stm32f2Port<GPIOD_BASE>;
#ifdef GPIOE
using Stm32f2PortE = Stm32f2Port<GPIOE_BASE>;
#endif
#ifdef GPIOF
using Stm32f2PortF = Stm32f2Port<GPIOF_BASE>;
#endif
#ifdef GPIOG
using Stm32f2PortG = Stm32f2Port<GPIOG_BASE>;
#endif
using Stm32f2PortH = Stm32f2Port<GPIOH_BASE>;
#ifdef GPIOI
using Stm32f2PortI = Stm32f2Port<GPIOI_BASE>;
#endif
#ifdef GPIOJ
using Stm32f2PortJ = Stm32f2Port<GPIOJ_BASE>;
#endif
#ifdef GPIOK
using Stm32f2PortK = Stm32f2Port<GPIOK_BASE>;
#endif

template <typename TPort, int TPinIndex>
struct Stm32f2Pin {
    using Port = TPort;
    static const int PinIndex = TPinIndex;
};

template <uint8_t TPupdr>
struct Stm32f2PinPullMode {
    static uint8_t const Pupdr = TPupdr;
};
using Stm32f2PinPullModeNone = Stm32f2PinPullMode<0>;
using Stm32f2PinPullModePullUp = Stm32f2PinPullMode<1>;
using Stm32f2PinPullModePullDown = Stm32f2PinPullMode<2>;

template <uint8_t TOptyper>
struct Stm32f2PinOutputType {
    static uint8_t const Optyper = TOptyper;
};
using Stm32f2PinOutputTypeNormal = Stm32f2PinOutputType<0>;
using Stm32f2PinOutputTypeOpenDrain = Stm32f2PinOutputType<1>;

template <uint8_t TOspeedr>
struct Stm32f2PinOutputSpeed {
    static uint8_t const Ospeedr = TOspeedr;
};
using Stm32f2PinOutputSpeedLow = Stm32f2PinOutputSpeed<0>;
using Stm32f2PinOutputSpeedMedium = Stm32f2PinOutputSpeed<1>;
using Stm32f2PinOutputSpeedFast = Stm32f2PinOutputSpeed<2>;
using Stm32f2PinOutputSpeedHigh = Stm32f2PinOutputSpeed<3>;

template <typename PullMode>
struct Stm32f2PinInputMode {
    static uint8_t const Pupdr = PullMode::Pupdr;
};
using Stm32f2PinInputModeNormal = Stm32f2PinInputMode<Stm32f2PinPullModeNone>;
using Stm32f2PinInputModePullUp = Stm32f2PinInputMode<Stm32f2PinPullModePullUp>;
using Stm32f2PinInputModePullDown = Stm32f2PinInputMode<Stm32f2PinPullModePullDown>;

template <typename OutputType, typename OutputSpeed, typename PullMode>
struct Stm32f2PinOutputMode {
    static uint8_t const Optyper = OutputType::Optyper;
    static uint8_t const Ospeedr = OutputSpeed::Ospeedr;
    static uint8_t const Pupdr = PullMode::Pupdr;
};
using Stm32f2PinOutputModeNormal = Stm32f2PinOutputMode<Stm32f2PinOutputTypeNormal, Stm32f2PinOutputSpeedLow, Stm32f2PinPullModeNone>;
using Stm32f2PinOutputModeOpenDrain = Stm32f2PinOutputMode<Stm32f2PinOutputTypeOpenDrain, Stm32f2PinOutputSpeedLow, Stm32f2PinPullModeNone>;

template <typename Arg>
class Stm32f2Pins {
    using Context      = typename Arg::Context;
    using ParentObject = typename Arg::ParentObject;
    
public:
    struct Object;
    
private:
    using TheDebugObject = DebugObject<Context, Object>;
    
public:
    static void init (Context c)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();
        __HAL_RCC_GPIOE_CLK_ENABLE();
#ifdef GPIOF
        __HAL_RCC_GPIOF_CLK_ENABLE();
#endif
#ifdef GPIOG
        __HAL_RCC_GPIOG_CLK_ENABLE();
#endif
        __HAL_RCC_GPIOH_CLK_ENABLE();
#ifdef GPIOI
        __HAL_RCC_GPIOI_CLK_ENABLE();
#endif
#ifdef GPIOJ
        __HAL_RCC_GPIOJ_CLK_ENABLE();
#endif
#ifdef GPIOK
        __HAL_RCC_GPIOK_CLK_ENABLE();
#endif
        
        TheDebugObject::init(c);
    }
    
    static void deinit (Context c)
    {
        TheDebugObject::deinit(c);
        
#ifdef GPIOK
        __HAL_RCC_GPIOK_CLK_DISABLE();
#endif
#ifdef GPIOJ
        __HAL_RCC_GPIOJ_CLK_DISABLE();
#endif
#ifdef GPIOI
        __HAL_RCC_GPIOI_CLK_DISABLE();
#endif
        __HAL_RCC_GPIOH_CLK_DISABLE();
#ifdef GPIOG
        __HAL_RCC_GPIOG_CLK_DISABLE();
#endif
#ifdef GPIOF
        __HAL_RCC_GPIOF_CLK_DISABLE();
#endif
        __HAL_RCC_GPIOE_CLK_DISABLE();
        __HAL_RCC_GPIOD_CLK_DISABLE();
        __HAL_RCC_GPIOC_CLK_DISABLE();
        __HAL_RCC_GPIOB_CLK_DISABLE();
        __HAL_RCC_GPIOA_CLK_DISABLE();
    }
    
    template <typename Pin, typename Mode = Stm32f2PinInputModeNormal, typename ThisContext>
    static void setInput (ThisContext c)
    {
        TheDebugObject::access(c);
        
        AMBRO_LOCK_T(InterruptTempLock(), c, lock_c) {
            set_pupdr<Pin, Mode::Pupdr>();
            set_moder<Pin, 0>();
        }
    }
    
    template <typename Pin, typename Mode = Stm32f2PinOutputModeNormal, typename ThisContext>
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
    
    template <typename Pin, int AfNumber, typename Mode = Stm32f2PinOutputModeNormal, typename ThisContext>
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
    struct Object : public ObjBase<Stm32f2Pins, ParentObject, MakeTypeList<TheDebugObject>> {};
};

struct Stm32f2PinsService {
    APRINTER_ALIAS_STRUCT_EXT(Pins, (
        APRINTER_AS_TYPE(Context),
        APRINTER_AS_TYPE(ParentObject)
    ), (
        APRINTER_DEF_INSTANCE(Pins, Stm32f2Pins)
    ))
};

}

#endif
