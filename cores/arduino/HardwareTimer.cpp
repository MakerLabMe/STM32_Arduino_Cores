/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Bryan Newbold.
 * Copyright (c) 2015 Andy Sze(andy.sze.mail@gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

#include <HardwareTimer.h>

#include <Arduino.h> // for noInterrupts(), interrupts()
// #include <wirish/wirish_math.h>
// #include <board/board.h>           // for CYCLES_PER_MICROSECOND

// TODO [0.1.0] Remove deprecated pieces

/*
 * Evil hack to infer this->dev from timerNum in the HardwareTimer
 * constructor. See:
 *
 * http://www.parashift.com/c++-faq-lite/pointers-to-members.html#faq-33.2
 * http://yosefk.com/c++fqa/function.html#fqa-33.2
 */

extern "C" {
    static timer_dev **this_devp;
    // static rcc_clk_id this_id;
    // static void set_this_dev(timer_dev *dev) {
    //     if (dev->clk_id == this_id) {
    //         *this_devp = dev;
    //     }
    // }
}

// const HardwareTimer::Timer HardwareTimer::Timers[NUM_TIMERS] = {
//     {TIM1,},
//     {TIM2,},
//     {TIM3,},
//     {TIM4,},
//     {TIM5,},
//     {TIM6,},
//     {TIM7,},
//     {TIM8,},
// };

void (*HardwareTimer::callbacks[NUM_TIMERS])() = {};

/*
 * HardwareTimer routines
 */

HardwareTimer::HardwareTimer(uint8_t timerNum) {
    // this->dev = Timers[timerNum];
    // noInterrupts(); // Hack to ensure we're the only ones using
    //                 // set_this_dev() and friends. TODO: use a lock.
    // this_id = timerID;
    // this_devp = &this->dev;
    // timer_foreach(set_this_dev);
    // interrupts();
    // ASSERT(this->dev != NULL);
}
void HardwareTimer::pause(void) 
{
    // TIM_Cmd(this->dev，DISABLE);
}
void HardwareTimer::resume(void) {
    // TIM_Cmd(this->dev，ENABLE);
}

uint32_t HardwareTimer::getPrescaleFactor(void) {
    return TIM_GetPrescaler(this->dev);
}

void HardwareTimer::setPrescaleFactor(uint32_t factor) {
    TIM_PrescalerConfig(this->dev, factor, TIM_PSCReloadMode_Immediate);
}

uint16_t HardwareTimer::getOverflow() {
    // return timer_get_reload(this->dev);
}

void HardwareTimer::setOverflow(uint16_t val) {
    TIM_SetAutoreload(this->dev, val);
}

uint16_t HardwareTimer::getCount(void) {
    // return TIM_GetCounter(this->dev);
}

void HardwareTimer::setCount(uint16_t val) {
    // uint16 ovf = this->getOverflow();
    // timer_set_count(this->dev, min(val, ovf));
}

#define MAX_RELOAD ((1 << 16) - 1)
uint16_t HardwareTimer::setPeriod(uint32_t microseconds) {
    // Not the best way to handle this edge case?
    // if (!microseconds) {
    //     this->setPrescaleFactor(1);
    //     this->setOverflow(1);
    //     return this->getOverflow();
    // }

    // uint32 period_cyc = microseconds * CYCLES_PER_MICROSECOND;
    // uint16 prescaler = (uint16)(period_cyc / MAX_RELOAD + 1);
    // uint16 overflow = (uint16)((period_cyc + (prescaler / 2)) / prescaler);
    // this->setPrescaleFactor(prescaler);
    // this->setOverflow(overflow);
    // return overflow;
}

void HardwareTimer::setMode(int channel, timer_mode mode) {
    TIM_OCInitStructure.TIM_OCMode =  mode;
    if (channel == 1)
    {
        TIM_OC1Init(this->dev, &TIM_OCInitStructure); 
    }else if (channel == 2)
    {
        TIM_OC2Init(this->dev, &TIM_OCInitStructure); 
    }else if (channel == 3)
    {
        TIM_OC3Init(this->dev, &TIM_OCInitStructure); 
    }else if (channel == 4)
    {
        TIM_OC4Init(this->dev, &TIM_OCInitStructure); 
    }
}

uint16_t HardwareTimer::getCompare(int channel) {
    // return timer_get_compare(this->dev, (uint8)channel);
}

void HardwareTimer::setCompare(int channel, uint16_t val) {
    // uint16 ovf = this->getOverflow();
    // timer_set_compare(this->dev, (uint8_t)channel, min(val, ovf));
}

void HardwareTimer::attachInterrupt(int channel, voidFuncPtr handler) {
    // timer_attach_interrupt(this->dev, (uint8_t)channel, handler);
}

void HardwareTimer::detachInterrupt(int channel) {
    // timer_detach_interrupt(this->dev, (uint8_t)channel);
}

void HardwareTimer::refresh(void) {
    // timer_generate_update(this->dev);
}

/* -- Deprecated predefined instances -------------------------------------- */

HardwareTimer Timer1(1);
HardwareTimer Timer2(2);
HardwareTimer Timer3(3);
HardwareTimer Timer4(4);
#ifdef STM32_HIGH_DENSITY
HardwareTimer Timer5(5);
HardwareTimer Timer6(6);
HardwareTimer Timer7(7);
HardwareTimer Timer8(8);
#endif