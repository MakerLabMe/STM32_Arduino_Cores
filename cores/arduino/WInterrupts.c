/*
  Copyright (c) 2014 MakerLab.me & Andy Sze(andy.sze.mail@gmail.com)  All right reserved.
  Copyright (c) 2011-2012 Arduino.  All right reserved.

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

#include "WInterrupts.h"

//Interrupts
 const uint8_t GPIO_IRQn[] = {
  EXTI0_IRQn,     //0
  EXTI1_IRQn,     //1
  EXTI2_IRQn,     //2
  EXTI3_IRQn,     //3
  EXTI4_IRQn,     //4
  EXTI9_5_IRQn,   //5
  EXTI9_5_IRQn,   //6
  EXTI9_5_IRQn,   //7
  EXTI9_5_IRQn,   //8
  EXTI9_5_IRQn,   //9
  EXTI15_10_IRQn, //10
  EXTI15_10_IRQn, //11
  EXTI15_10_IRQn, //12
  EXTI15_10_IRQn, //13
  EXTI15_10_IRQn, //14
  EXTI15_10_IRQn  //15
 };

typedef void (*interruptCB)(void);

static interruptCB callbacksEXTI[16];//EXTI line:0~15

/* Configure PIO interrupt sources */
static void __initialize() {
	uint16_t i;
	for (i=0; i<16; i++) {
		callbacksEXTI[i] = NULL;
	}

#if 0
	pmc_enable_periph_clk(ID_PIOA);
	NVIC_DisableIRQ(PIOA_IRQn);
	NVIC_ClearPendingIRQ(PIOA_IRQn);
	NVIC_SetPriority(PIOA_IRQn, 0);
	NVIC_EnableIRQ(PIOA_IRQn);

	pmc_enable_periph_clk(ID_PIOB);
	NVIC_DisableIRQ(PIOB_IRQn);
	NVIC_ClearPendingIRQ(PIOB_IRQn);
	NVIC_SetPriority(PIOB_IRQn, 0);
	NVIC_EnableIRQ(PIOB_IRQn);

	pmc_enable_periph_clk(ID_PIOC);
	NVIC_DisableIRQ(PIOC_IRQn);
	NVIC_ClearPendingIRQ(PIOC_IRQn);
	NVIC_SetPriority(PIOC_IRQn, 0);
	NVIC_EnableIRQ(PIOC_IRQn);

	pmc_enable_periph_clk(ID_PIOD);
	NVIC_DisableIRQ(PIOD_IRQn);
	NVIC_ClearPendingIRQ(PIOD_IRQn);
	NVIC_SetPriority(PIOD_IRQn, 0);
	NVIC_EnableIRQ(PIOD_IRQn);
#endif
}


void attachInterrupt(uint32_t pin, void (*callback)(void), uint32_t mode)
{
	static int enabled = 0;
	if (!enabled) {
		__initialize();
		enabled = 1;
	}

	uint8_t GPIO_PortSource = 0;	//variable to hold the port number
	uint8_t GPIO_PinSource = 0;	//variable to hold the pin number
	uint16_t PinNumber;				//temp variable to calculate the pin number


	//EXTI structure to init EXT
	EXTI_InitTypeDef EXTI_InitStructure;
	//NVIC structure to set up NVIC controller
	NVIC_InitTypeDef NVIC_InitStructure;

	GPIO_TypeDef *gpio_port = g_APinDescription[pin].pPort;
	uint16_t gpio_pin = g_APinDescription[pin].ulPin;

#if defined(STM32F10X_HD) || defined (STM32F10X_MD)
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
#elif defined (STM32F40_41xxx)
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
#endif

	//Select the port source
	if (gpio_port == GPIOA )
	{
#if defined(STM32F10X_HD) || defined (STM32F10X_MD)
		GPIO_PortSource = GPIO_PortSourceGPIOA;
#elif defined (STM32F40_41xxx)
		GPIO_PortSource = EXTI_PortSourceGPIOA;
#endif
	}
	else if (gpio_port == GPIOB )
	{
#if defined(STM32F10X_HD) || defined (STM32F10X_MD)
		GPIO_PortSource = GPIO_PortSourceGPIOB;
#elif defined (STM32F40_41xxx)
		GPIO_PortSource = EXTI_PortSourceGPIOB;
#endif
	}
	else if (gpio_port == GPIOC )
	{
#if defined(STM32F10X_HD) || defined (STM32F10X_MD)
		GPIO_PortSource = GPIO_PortSourceGPIOC;
#elif defined (STM32F40_41xxx)
		GPIO_PortSource = EXTI_PortSourceGPIOC;
#endif
	}
	else if (gpio_port == GPIOD )
	{
#if defined(STM32F10X_HD) || defined (STM32F10X_MD)
		GPIO_PortSource = GPIO_PortSourceGPIOD;
#elif defined (STM32F40_41xxx)
		GPIO_PortSource = EXTI_PortSourceGPIOD;
#endif
	}
	else if (gpio_port == GPIOE )
	{
#if defined(STM32F10X_HD) || defined (STM32F10X_MD)
		GPIO_PortSource = GPIO_PortSourceGPIOE;
#elif defined (STM32F40_41xxx)
		GPIO_PortSource = EXTI_PortSourceGPIOE;
#endif
	}

	//Find out the pin number from the mask
	PinNumber = gpio_pin;
	PinNumber = PinNumber >> 1;
	while(PinNumber)
	{
		PinNumber = PinNumber >> 1;
		GPIO_PinSource++;
	}


	// Register the handler for the user function name
  callbacksEXTI[GPIO_PinSource] = callback;

	//Connect EXTI Line to appropriate Pin
	GPIO_EXTILineConfig(GPIO_PortSource, GPIO_PinSource);

	//Configure GPIO EXTI line
	EXTI_InitStructure.EXTI_Line = gpio_pin;//EXTI_Line;
	
	//select the interrupt mode
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	switch (mode)
	{
		//case LOW:
			//There is no LOW mode in STM32, so using falling edge as default
			//EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
			//break;
		case CHANGE:
			//generate interrupt on rising or falling edge
			EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
			break;
		case RISING:
			//generate interrupt on rising edge
			EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
			break;
		case FALLING:
			//generate interrupt on falling edge
			EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
			break;
	}

	//enable EXTI line
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	//send values to registers
	EXTI_Init(&EXTI_InitStructure);

  //configure NVIC
  //select NVIC channel to configure
  NVIC_InitStructure.NVIC_IRQChannel = GPIO_IRQn[GPIO_PinSource];
  if(GPIO_PinSource > 4)
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
  else
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 13;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  //enable IRQ channel
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  //update NVIC registers
  NVIC_Init(&NVIC_InitStructure);
}

void detachInterrupt(uint32_t pin)
{
	uint8_t GPIO_PinSource = 0;	//variable to hold the pin number
	uint8_t PinNumber;				//temp variable to calculate the pin number

	uint16_t gpio_pin = g_APinDescription[pin].ulPin;

	//Clear the pending interrupt flag for that interrupt pin
	EXTI_ClearITPendingBit(gpio_pin);

	//EXTI structure to init EXT
	EXTI_InitTypeDef EXTI_InitStructure;

	//Find out the pin number from the mask
	PinNumber = gpio_pin;
	PinNumber = PinNumber >> 1;
	while(PinNumber)
	{
		PinNumber = PinNumber >> 1;
		GPIO_PinSource++;
	}

  //Select the appropriate EXTI line
  EXTI_InitStructure.EXTI_Line = gpio_pin;
  //disable that EXTI line
  EXTI_InitStructure.EXTI_LineCmd = DISABLE;
  //send values to registers
  EXTI_Init(&EXTI_InitStructure);

	//unregister the user's handler
	callbacksEXTI[GPIO_PinSource] = NULL;
  
}

#ifdef __cplusplus
extern "C" {
#endif


// void interrupts(void){
// 	//Only enable the interrupts that are exposed to the user
// 	NVIC_EnableIRQ(EXTI0_IRQn);
// 	NVIC_EnableIRQ(EXTI1_IRQn);
// 	NVIC_EnableIRQ(EXTI2_IRQn);
// 	NVIC_EnableIRQ(EXTI3_IRQn);
// 	NVIC_EnableIRQ(EXTI4_IRQn);
// 	NVIC_EnableIRQ(EXTI9_5_IRQn);
// 	NVIC_EnableIRQ(EXTI15_10_IRQn);
// }

// void noInterrupts(void){
// 	NVIC_DisableIRQ(EXTI0_IRQn);
// 	NVIC_DisableIRQ(EXTI1_IRQn);
// 	NVIC_DisableIRQ(EXTI2_IRQn);
// 	NVIC_DisableIRQ(EXTI3_IRQn);
// 	NVIC_DisableIRQ(EXTI4_IRQn);
// 	NVIC_DisableIRQ(EXTI9_5_IRQn);
// 	NVIC_DisableIRQ(EXTI15_10_IRQn);
// }

/* interrupt handler for PA0,PB0,PC0,PD0,PE0 */
void EXTI0_IRQHandler      (void) {

  digitalWrite(72,HIGH);

	if (EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		/* Clear the EXTI line pending bit */
		EXTI_ClearITPendingBit(EXTI_Line0);

		if(NULL != callbacksEXTI[0])
		{
      callbacksEXTI[0]();
		}
	}
}

/* interrupt handler for PA1,PB1,PC1,PD1,PE1 */
void EXTI1_IRQHandler    (void) {

  digitalWrite(72,HIGH);

	if (EXTI_GetITStatus(EXTI_Line1) != RESET)
	{
		/* Clear the EXTI line pending bit */
		EXTI_ClearITPendingBit(EXTI_Line1);

		if(NULL != callbacksEXTI[1])
		{
      callbacksEXTI[1]();
		}
	}
}

/* interrupt handler for PA2,PB2,PC2,PD2,PE2 */
void EXTI2_IRQHandler    (void) {

  digitalWrite(72,HIGH);

	if (EXTI_GetITStatus(EXTI_Line2) != RESET)
	{
		/* Clear the EXTI line pending bit */
		EXTI_ClearITPendingBit(EXTI_Line2);

		if(NULL != callbacksEXTI[2])
		{
      callbacksEXTI[2]();
		}
	}
}

/* interrupt handler for PA3,PB3,PC3,PD3,PE3 */
void EXTI3_IRQHandler    (void) {

  digitalWrite(72,HIGH);

	if (EXTI_GetITStatus(EXTI_Line3) != RESET)
	{
		/* Clear the EXTI line pending bit */
		EXTI_ClearITPendingBit(EXTI_Line3);

		if(NULL != callbacksEXTI[3])
		{
      callbacksEXTI[3]();
		}
	}
}

/* interrupt handler for PA4,PB4,PC4,PD4,PE4 */
void EXTI4_IRQHandler    (void) {

  digitalWrite(72,HIGH);

	if (EXTI_GetITStatus(EXTI_Line4) != RESET)
	{
		/* Clear the EXTI line pending bit */
		EXTI_ClearITPendingBit(EXTI_Line4);

		if(NULL != callbacksEXTI[4])
		{
      callbacksEXTI[4]();
		}
	}
}

/* interrupt handler for PA5~9,PB5~9,PC5~9,PD5~9,PE5~9 */
void EXTI9_5_IRQHandler  (void) {

  digitalWrite(72,HIGH);

	if (EXTI_GetITStatus(EXTI_Line5) != RESET)
	{
		/* Clear the EXTI line pending bit */
		EXTI_ClearITPendingBit(EXTI_Line5);

		if(NULL != callbacksEXTI[5])
		{
      callbacksEXTI[5]();
		}
	}

	if (EXTI_GetITStatus(EXTI_Line6) != RESET)
	{
		/* Clear the EXTI line pending bit */
		EXTI_ClearITPendingBit(EXTI_Line6);

		if(NULL != callbacksEXTI[6])
		{
      callbacksEXTI[6]();
		}
	}

	if (EXTI_GetITStatus(EXTI_Line7) != RESET)
	{
		/* Clear the EXTI line pending bit */
		EXTI_ClearITPendingBit(EXTI_Line7);

		if(NULL != callbacksEXTI[7])
		{
      callbacksEXTI[7]();
		}
	}

	if (EXTI_GetITStatus(EXTI_Line8) != RESET)
	{
		/* Clear the EXTI line pending bit */
		EXTI_ClearITPendingBit(EXTI_Line8);

		if(NULL != callbacksEXTI[8])
		{
      callbacksEXTI[8]();
		}
	}

	if (EXTI_GetITStatus(EXTI_Line9) != RESET)
	{
		/* Clear the EXTI line pending bit */
		EXTI_ClearITPendingBit(EXTI_Line9);

		if(NULL != callbacksEXTI[9])
		{
      callbacksEXTI[9]();
		}
	}
}

/* interrupt handler for PA10~15,PB10~15,PC10~15,PD1015,PE10~15 */
void EXTI15_10_IRQHandler(void) {

  digitalWrite(72,HIGH);

	if (EXTI_GetITStatus(EXTI_Line10) != RESET)
	{
		/* Clear the EXTI line pending bit */
		EXTI_ClearITPendingBit(EXTI_Line10);

		if(NULL != callbacksEXTI[10])
		{
      callbacksEXTI[10]();
		}
	}

	if (EXTI_GetITStatus(EXTI_Line11) != RESET)
	{
		/* Clear the EXTI line pending bit */
		EXTI_ClearITPendingBit(EXTI_Line11);

		if(NULL != callbacksEXTI[11])
		{
      callbacksEXTI[11]();
		}
	}

	if (EXTI_GetITStatus(EXTI_Line12) != RESET)
	{
		/* Clear the EXTI line pending bit */
		EXTI_ClearITPendingBit(EXTI_Line12);

		if(NULL != callbacksEXTI[12])
		{
      callbacksEXTI[12]();
		}
	}

	if (EXTI_GetITStatus(EXTI_Line13) != RESET)
	{
		/* Clear the EXTI line pending bit */
		EXTI_ClearITPendingBit(EXTI_Line13);

		if(NULL != callbacksEXTI[13])
		{
      callbacksEXTI[13]();
		}
	}

	if (EXTI_GetITStatus(EXTI_Line14) != RESET)
	{
		/* Clear the EXTI line pending bit */
		EXTI_ClearITPendingBit(EXTI_Line14);

		if(NULL != callbacksEXTI[14])
		{
      callbacksEXTI[14]();
		}
	}

	if (EXTI_GetITStatus(EXTI_Line15) != RESET)
	{
		/* Clear the EXTI line pending bit */
		EXTI_ClearITPendingBit(EXTI_Line15);

		if(NULL != callbacksEXTI[15])
		{
      callbacksEXTI[15]();
		}
	}
}


#ifdef __cplusplus
}
#endif
