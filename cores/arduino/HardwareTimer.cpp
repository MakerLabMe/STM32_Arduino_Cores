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

// extern "C" {
//     static timer_dev **this_devp;
    // static rcc_clk_id this_id;
    // static void set_this_dev(timer_dev *dev) {
    //     if (dev->clk_id == this_id) {
    //         *this_devp = dev;
    //     }
    // }
// }

typedef struct {
  TIM_TypeDef *Timer;
} Timer;

const Timer Timers[NUM_TIMERS] = {
    // ((TIM_TypeDef *) TIM2_BASE),
    {TIM1},
    {TIM2},
    {TIM3},
    {TIM4},
    {TIM5},
    {TIM6},
    {TIM7},
    {TIM8},
};

// const TIM_TypeDef T1 = TIM1;

// Every timer has 4 channels
void (*HardwareTimer::callbacks[NUM_TIMERS * 4])() = {NULL};

/*
 * HardwareTimer routines
 */

HardwareTimer::HardwareTimer(uint8_t _timerNum) {
    this->dev = Timers[_timerNum - 1].Timer;
    this->timerNum = _timerNum;
  
    // noInterrupts(); // Hack to ensure we're the only ones using
    //                 // set_this_dev() and friends. TODO: use a lock.
    // this_id = timerID;
    // this_devp = &this->dev;
    // timer_foreach(set_this_dev);
    // interrupts();
    // ASSERT(this->dev != NULL);
}
void HardwareTimer::begin(void) 
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    if(timerNum == 1)
    {
        NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    }else if(timerNum == 2)
    {

      /* Enable the TIM2 global Interrupt */
        NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;

        NVIC_Init(&NVIC_InitStructure);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    }else if(timerNum == 3)
    {

      /* Enable the TIM3 global Interrupt */
        NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;

        NVIC_Init(&NVIC_InitStructure);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    }else if(timerNum == 4)
    {
        NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;

        NVIC_Init(&NVIC_InitStructure);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    }else if(timerNum == 5)
    {
        NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;

        NVIC_Init(&NVIC_InitStructure);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    }else if(timerNum == 6)
    {
        NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 6;

        NVIC_Init(&NVIC_InitStructure);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
    }else if(timerNum == 7)
    {
        NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;

        NVIC_Init(&NVIC_InitStructure);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
    }else if(timerNum == 8)
    {
        NVIC_InitStructure.NVIC_IRQChannel = TIM8_CC_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 8;

        NVIC_Init(&NVIC_InitStructure);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
    } 


      /* Time Base configuration */

    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(this->dev, &TIM_TimeBaseStructure);


    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  // TIM_OCInitStructure.TIM_Pulse = 1; //default value

  if(callbacks[ (timerNum - 1) * 4 + 1 -1 ])
  {
    TIM_OCInitStructure.TIM_Pulse = this->dev->CCR1;
    TIM_OC1Init(this->dev, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(this->dev, TIM_OCPreload_Disable);
    TIM_ITConfig(this->dev, TIM_IT_CC1 , ENABLE);

}
if (callbacks[ (timerNum - 1) * 4 + 2 -1 ])
{
    TIM_OCInitStructure.TIM_Pulse = this->dev->CCR2;
    TIM_OC2Init(this->dev, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(this->dev, TIM_OCPreload_Disable);
    TIM_ITConfig(this->dev, TIM_IT_CC1 , ENABLE);
}
if (callbacks[ (timerNum - 1) * 4 + 3 -1 ])
{
    TIM_OCInitStructure.TIM_Pulse = this->dev->CCR3;
    TIM_OC3Init(this->dev, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(this->dev, TIM_OCPreload_Disable);
    TIM_ITConfig(this->dev, TIM_IT_CC1 , ENABLE);
}
if (callbacks[ (timerNum - 1) * 4 + 4 -1 ])
{
    TIM_OCInitStructure.TIM_Pulse = this->dev->CCR4;
    TIM_OC4Init(this->dev, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(this->dev, TIM_OCPreload_Disable);
    TIM_ITConfig(this->dev, TIM_IT_CC1 , ENABLE);
}


TIM_ARRPreloadConfig(this->dev, ENABLE);

TIM_Cmd(this->dev, ENABLE);
}

void HardwareTimer::end(void) 
{
    // TIM_Cmd(this->dev，DISABLE);
    TIM_Cmd(this->dev, DISABLE);
}
void HardwareTimer::resume(void) {
    TIM_Cmd(this->dev, ENABLE);
}

uint32_t HardwareTimer::getPrescaleFactor(void) {
    return TIM_TimeBaseStructure.TIM_Prescaler;
}

void HardwareTimer::setPrescaleFactor(uint32_t factor) {
    TIM_TimeBaseStructure.TIM_Prescaler = factor;
    TIM_PrescalerConfig(this->dev, factor, TIM_PSCReloadMode_Update);
}

uint16_t HardwareTimer::getOverflow() {
    return TIM_TimeBaseStructure.TIM_Period;
}

void HardwareTimer::setOverflow(uint16_t val) {
    TIM_TimeBaseStructure.TIM_Period = val;
    this->dev->ARR = val;
}

uint16_t HardwareTimer::getCount(void) {
    return this->dev->CNT;
}

void HardwareTimer::setCount(uint16_t val) {
    uint16_t ovf = this->getOverflow();
    this->dev->CNT = min(val, ovf);
}

#define MAX_RELOAD ((1 << 16) - 1)//65535
#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
    #define CYCLES_PER_MICROSECOND    72 
#endif
uint16_t HardwareTimer::setPeriod(uint32_t microseconds) {
    // Not the best way to handle this edge case?
    if (!microseconds) {
        this->setPrescaleFactor(1);
        this->setOverflow(1);
        return this->getOverflow();
    }

    uint32_t period_cyc = microseconds * CYCLES_PER_MICROSECOND;
    uint16_t prescaler = (uint16_t)(period_cyc / MAX_RELOAD + 1);
    uint16_t overflow = (uint16_t)((period_cyc + (prescaler / 2)) / prescaler);
    this->setPrescaleFactor(prescaler);
    this->setOverflow(overflow);
    return overflow;
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
    if (channel == 1)
    {
        return this->dev->CCR1; 
    }else if (channel == 2)
    {
        return this->dev->CCR2; 
    }else if (channel == 3)
    {
        return this->dev->CCR3; 
    }else if (channel == 4)
    {
        return this->dev->CCR4; 
    }
}

void HardwareTimer::setCompare(int channel, uint16_t val) {
    uint16_t ovf = this->getOverflow();
    // timer_set_compare(this->dev, (uint8_t)channel, min(val, ovf));
    if (channel == 1)
    {
        this->dev->CCR1 = min(val, ovf); 
    }else if (channel == 2)
    {
        this->dev->CCR2 = min(val, ovf); 
    }else if (channel == 3)
    {
        this->dev->CCR3 = min(val, ovf); 
    }else if (channel == 4)
    {
        this->dev->CCR4 = min(val, ovf); 
    }
}

void HardwareTimer::attachInterrupt(int channel, voidFuncPtr handler) {
    // timer_attach_interrupt(this->dev, (uint8_t)channel, handler);
    callbacks[ (timerNum - 1) * 4 + channel -1 ] = handler;
}

void HardwareTimer::detachInterrupt(int channel) {
    // timer_detach_interrupt(this->dev, (uint8_t)channel);
    callbacks[ (timerNum - 1) * 4 + channel -1 ] = NULL;
    end();
}

void HardwareTimer::refresh(void) {
    // timer_generate_update(this->dev);
    this->dev->EGR = TIM_PSCReloadMode_Immediate; 
}

/* -- Deprecated predefined instances -------------------------------------- */

HardwareTimer Timer1(1);
HardwareTimer Timer2(2);
HardwareTimer Timer3(3);
HardwareTimer Timer4(4);
#ifdef STM32F10X_HD
HardwareTimer Timer5(5);
HardwareTimer Timer6(6);
HardwareTimer Timer7(7);
HardwareTimer Timer8(8);
#endif


/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/

/**
  * @brief  This function handles TIM2 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
  {
    // Serial.println("1");
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);    
    if(HardwareTimer::callbacks[1*4 + 0] != NULL)
        HardwareTimer::callbacks[1*4 + 0]();
  }
  if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)
  {
    // Serial.println("2");
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
    void (*handler)(void) = HardwareTimer::callbacks[1*4 + 1];
    if( handler)
        handler();
    // Serial.println("21");
  }
  if (TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);
    if(HardwareTimer::callbacks[1*4 + 2] != NULL)
        HardwareTimer::callbacks[1*4 + 2]();
  }
  if (TIM_GetITStatus(TIM2, TIM_IT_CC4) != RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
    if(HardwareTimer::callbacks[1*4 + 3] != NULL)
        HardwareTimer::callbacks[1*4 + 3]();
  }    
}

void TIM3_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);    
    if(HardwareTimer::callbacks[2*4 + 0] != NULL)
        HardwareTimer::callbacks[2*4 + 0]();
  }
  if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
    void (*handler)(void) = HardwareTimer::callbacks[2*4 + 1];
    if( handler)
        handler();
  }
  if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
    if(HardwareTimer::callbacks[2*4 + 2] != NULL)
        HardwareTimer::callbacks[2*4 + 2]();
  }
  if (TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
    if(HardwareTimer::callbacks[2*4 + 3] != NULL)
        HardwareTimer::callbacks[2*4 + 3]();
  } 
}

void TIM4_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET)
  {
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);    
    if(HardwareTimer::callbacks[3*4 + 0] != NULL)
        HardwareTimer::callbacks[3*4 + 0]();
  }
  if (TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET)
  {
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);
    void (*handler)(void) = HardwareTimer::callbacks[3*4 + 1];
    if( handler)
        handler();
  }
  if (TIM_GetITStatus(TIM4, TIM_IT_CC3) != RESET)
  {
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC3);
    if(HardwareTimer::callbacks[3*4 + 2] != NULL)
        HardwareTimer::callbacks[3*4 + 2]();
  }
  if (TIM_GetITStatus(TIM4, TIM_IT_CC4) != RESET)
  {
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC4);
    if(HardwareTimer::callbacks[3*4 + 3] != NULL)
        HardwareTimer::callbacks[3*4 + 3]();
  } 
}

void TIM5_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM5, TIM_IT_CC1) != RESET)
  {
    TIM_ClearITPendingBit(TIM5, TIM_IT_CC1);    
    if(HardwareTimer::callbacks[4*4 + 0] != NULL)
        HardwareTimer::callbacks[4*4 + 0]();
  }
  if (TIM_GetITStatus(TIM5, TIM_IT_CC2) != RESET)
  {
    TIM_ClearITPendingBit(TIM5, TIM_IT_CC2);
    if( HardwareTimer::callbacks[4*4 + 1])
        HardwareTimer::callbacks[4*4 + 1]();
  }
  if (TIM_GetITStatus(TIM5, TIM_IT_CC3) != RESET)
  {
    TIM_ClearITPendingBit(TIM5, TIM_IT_CC3);
    if(HardwareTimer::callbacks[4*4 + 2] != NULL)
        HardwareTimer::callbacks[4*4 + 2]();
  }
  if (TIM_GetITStatus(TIM5, TIM_IT_CC4) != RESET)
  {
    TIM_ClearITPendingBit(TIM5, TIM_IT_CC4);
    if(HardwareTimer::callbacks[4*4 + 3] != NULL)
        HardwareTimer::callbacks[4*4 + 3]();
  } 
}

void TIM1_CC_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET)
  {
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);    
    if(HardwareTimer::callbacks[0*4 + 0] != NULL)
        HardwareTimer::callbacks[0*4 + 0]();
  }
  if (TIM_GetITStatus(TIM1, TIM_IT_CC2) != RESET)
  {
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);
    if( HardwareTimer::callbacks[0*4 + 1])
        HardwareTimer::callbacks[0*4 + 1]();
  }
  if (TIM_GetITStatus(TIM1, TIM_IT_CC3) != RESET)
  {
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC3);
    if(HardwareTimer::callbacks[0*4 + 2] != NULL)
        HardwareTimer::callbacks[0*4 + 2]();
  }
  if (TIM_GetITStatus(TIM1, TIM_IT_CC4) != RESET)
  {
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC4);
    if(HardwareTimer::callbacks[0*4 + 3] != NULL)
        HardwareTimer::callbacks[0*4 + 3]();
  } 
}

void TIM8_CC_IRQHandler(void)
{ 
    if (TIM_GetITStatus(TIM8, TIM_IT_CC1) != RESET)
  {
    TIM_ClearITPendingBit(TIM8, TIM_IT_CC1);    
    if(HardwareTimer::callbacks[7*4 + 0] != NULL)
        HardwareTimer::callbacks[7*4 + 0]();
  }
  if (TIM_GetITStatus(TIM8, TIM_IT_CC2) != RESET)
  {
    TIM_ClearITPendingBit(TIM8, TIM_IT_CC2);
    if( HardwareTimer::callbacks[7*4 + 1])
        HardwareTimer::callbacks[7*4 + 1]();
  }
  if (TIM_GetITStatus(TIM8, TIM_IT_CC3) != RESET)
  {
    TIM_ClearITPendingBit(TIM8, TIM_IT_CC3);
    if(HardwareTimer::callbacks[7*4 + 2] != NULL)
        HardwareTimer::callbacks[7*4 + 2]();
  }
  if (TIM_GetITStatus(TIM8, TIM_IT_CC4) != RESET)
  {
    TIM_ClearITPendingBit(TIM8, TIM_IT_CC4);
    if(HardwareTimer::callbacks[7*4 + 3] != NULL)
        HardwareTimer::callbacks[7*4 + 3]();
  } 

}
