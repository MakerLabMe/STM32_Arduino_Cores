/*
 Copyright (c) 2014 MakerLab.me & Andy Sze(andy.sze.mail@gmail.com)  All right reserved.
 Copyright (c) 2011 Arduino.  All right reserved.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 See the GNU Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

static int _readResolution = 10;
static int _writeResolution = 8;

void analogReadResolution(int res) {
	_readResolution = res;
}

void analogWriteResolution(int res) {
	_writeResolution = res;
}

static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to) {
	if (from == to)
		return value;
	if (from > to)
		return value >> (from-to);
	else
		return value << (to-from);
}

eAnalogReference analog_reference = AR_DEFAULT;

void analogReference(eAnalogReference ulMode)
{
	analog_reference = ulMode;
}

uint32_t analogRead(uint32_t ulPin)
{
  uint32_t ulValue = 0;
  uint32_t ulChannel;

  if (ulPin < A0)
    ulPin += A0;

  ulChannel = g_APinDescription[ulPin].ulADCChannelNumber ;

	if (ulPin >= PINS_COUNT || ulChannel == NONE )
	{
		return -1;
	}

  pinMode(ulPin,AN_INPUT);

#if defined (STM32F10X_HD) || (STM32F10X_MD)
	ADC_RegularChannelConfig(ADC1, g_APinDescription[ulPin].ulADCChannelNumber, 1, ADC_SampleTime_55Cycles5);
#elif defined (STM32F40_41xxx)
	ADC_RegularChannelConfig(ADC1, g_APinDescription[ulPin].ulADCChannelNumber, 1, ADC_SampleTime_15Cycles);
#endif
	//Start ADC1 Software Conversion
#if defined (STM32F10X_HD) || (STM32F10X_MD)
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
#elif defined (STM32F40_41xxx)	
	ADC_SoftwareStartConv(ADC1);
#endif

	// Wait until conversion completion
	// while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));

  // Read the value
  ulValue = ADC_GetConversionValue(ADC1);
  ulValue = mapResolution(ulValue, ADC_RESOLUTION, _readResolution);

	return ulValue;
}

#if 0
static void TC_SetCMR_ChannelA(Tc *tc, uint32_t chan, uint32_t v)
{
	tc->TC_CHANNEL[chan].TC_CMR = (tc->TC_CHANNEL[chan].TC_CMR & 0xFFF0FFFF) | v;
}

static void TC_SetCMR_ChannelB(Tc *tc, uint32_t chan, uint32_t v)
{
	tc->TC_CHANNEL[chan].TC_CMR = (tc->TC_CHANNEL[chan].TC_CMR & 0xF0FFFFFF) | v;
}
#endif

static uint8_t pinEnabled[PINS_COUNT];

void analogOutputInit(void) {
	uint8_t i;
	for (i=0; i<PINS_COUNT; i++)
		pinEnabled[i] = 0;
}

// Right now, PWM output only works on the pins with
// hardware support.  These are defined in the appropriate
// pins_*.c file.  For the rest of the pins, we default
// to digital output.
void analogWrite(uint32_t ulPin, uint32_t ulValue) {

	if (ulPin >= PINS_COUNT )
	{
		return;
	}

	if ( g_APinDescription[ulPin].ulTimerPeripheral == NULL)
	{
    // Defaults to digital write
    pinMode(ulPin, OUTPUT);
    ulValue = mapResolution(ulValue, _writeResolution, 8);
    if (ulValue < 128)
      digitalWrite(ulPin, LOW);
    else
      digitalWrite(ulPin, HIGH);

    return;
  }

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

  ulValue = mapResolution(ulValue, _writeResolution, PWM_RESOLUTION);

	//PWM Frequency : 1000 Hz,Timer counter clk:1MHz
	uint16_t TIM_Prescaler = (uint16_t)(SystemCoreClock / 1000000) - 1;
	uint16_t TIM_ARR = (uint16_t)(1000000 / PWM_FREQUENCY) - 1;

	uint16_t Duty_Cycle = (uint16_t)((ulValue * 100) / 255);
	// TIM Channel Duty Cycle(%) = (TIM_CCR / TIM_ARR + 1) * 100
	uint16_t TIM_CCR = (uint16_t)((Duty_Cycle * (TIM_ARR + 1)) / 100);

#if defined (STM32F40_41xxx)
  uint8_t GPIO_AF_TIM;
#endif

  pinMode(ulPin, AF_OUTPUT_PUSHPULL);

  if (!pinEnabled[ulPin]) {
    // Setup PWM for this pin

    // AFIO clock enable
#if defined(STM32F10X_HD) || defined (STM32F10X_MD)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
#elif defined (STM32F40_41xxx)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
#endif


    // TIM clock enable
    if(g_APinDescription[ulPin].ulTimerPeripheral == TIM1)
    {
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);
#if defined (STM32F40_41xxx)
      GPIO_AF_TIM = GPIO_AF_TIM1;
#endif
    }
    else if(g_APinDescription[ulPin].ulTimerPeripheral == TIM2)
    {
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
#if defined (STM32F40_41xxx)
      GPIO_AF_TIM = GPIO_AF_TIM2;
#endif
    }
    else if(g_APinDescription[ulPin].ulTimerPeripheral == TIM3)
    {
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
#if defined (STM32F40_41xxx)
      GPIO_AF_TIM = GPIO_AF_TIM3;
#endif
    }
    else if(g_APinDescription[ulPin].ulTimerPeripheral == TIM4)
    {
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
#if defined (STM32F40_41xxx)
      GPIO_AF_TIM = GPIO_AF_TIM4;
#endif
    }
    else if(g_APinDescription[ulPin].ulTimerPeripheral == TIM5)
    {
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
#if defined (STM32F40_41xxx)
      GPIO_AF_TIM = GPIO_AF_TIM5;
#endif
    }
    else if(g_APinDescription[ulPin].ulTimerPeripheral == TIM8)
    {
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8 , ENABLE);
#if defined (STM32F40_41xxx)
      GPIO_AF_TIM = GPIO_AF_TIM8;
#endif
    }

  #if defined (STM32F40_41xxx)
    uint8_t pin_source=0;
    uint16_t pin_temp = ulPin;
    while(pin_temp != 0x0001)
    {
      pin_temp = pin_temp>>1;
      pin_source +=1;
    }
  
    GPIO_PinAFConfig(g_APinDescription[ulPin].pPort, pin_source, GPIO_AF_TIM);
  #endif

    // Time base configuration
    TIM_TimeBaseStructure.TIM_Period = TIM_ARR;
    TIM_TimeBaseStructure.TIM_Prescaler = TIM_Prescaler;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//0
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    //for TIM1 and TIM8
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(g_APinDescription[ulPin].ulTimerPeripheral, &TIM_TimeBaseStructure);
    pinEnabled[ulPin] = 1;
  }

	// PWM1 Mode configuration
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = TIM_CCR;

  //for TIM1 and TIM8
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

	if(g_APinDescription[ulPin].ulTimerChannel == TIM_Channel_1)
	{
		// PWM1 Mode configuration: Channel1
		TIM_OC1Init(g_APinDescription[ulPin].ulTimerPeripheral, &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(g_APinDescription[ulPin].ulTimerPeripheral, TIM_OCPreload_Enable);
	}
	else if(g_APinDescription[ulPin].ulTimerChannel == TIM_Channel_2)
	{
		// PWM1 Mode configuration: Channel2
		TIM_OC2Init(g_APinDescription[ulPin].ulTimerPeripheral, &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(g_APinDescription[ulPin].ulTimerPeripheral, TIM_OCPreload_Enable);
	}
	else if(g_APinDescription[ulPin].ulTimerChannel == TIM_Channel_3)
	{
		// PWM1 Mode configuration: Channel3
		TIM_OC3Init(g_APinDescription[ulPin].ulTimerPeripheral, &TIM_OCInitStructure);
		TIM_OC3PreloadConfig(g_APinDescription[ulPin].ulTimerPeripheral, TIM_OCPreload_Enable);
	}
	else if(g_APinDescription[ulPin].ulTimerChannel == TIM_Channel_4)
	{
		// PWM1 Mode configuration: Channel4
		TIM_OC4Init(g_APinDescription[ulPin].ulTimerPeripheral, &TIM_OCInitStructure);
		TIM_OC4PreloadConfig(g_APinDescription[ulPin].ulTimerPeripheral, TIM_OCPreload_Enable);
	}

	TIM_ARRPreloadConfig(g_APinDescription[ulPin].ulTimerPeripheral, ENABLE);

	// TIM enable counter
	TIM_Cmd(g_APinDescription[ulPin].ulTimerPeripheral, ENABLE);

  //for TIM1 and TIM8
  TIM_CtrlPWMOutputs(g_APinDescription[ulPin].ulTimerPeripheral, ENABLE);

}

#ifdef __cplusplus
}
#endif
