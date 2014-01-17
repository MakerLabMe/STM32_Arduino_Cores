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

#include "variant.h"

/*
 * MakerLab Earth Board pin         |  PORT  | Label
 * ---------------------------------+--------+-------
 *   0                              |  PA10  | "RX0",pwm
 *   1                              |  PA9   | "TX0",pwm
 *   2       SPI3_SCK,I2S3_CK       |  PB3   |
 *   3       SPI3_MISO              |  PB4   |
 *   4  I2C1_SMBA,SPI3_MOSI,I2S3_SD |  PB5   |
 *   5       SPI1_MISO              |  PA6   | adc,pwm
 *   6       SPI1_MOSI              |  PA7   | adc,pwm
 *   7                              |  PB0   | adc,pwm
 *   8                              |  PB1   | adc,pwm
 *   9                              |  PA8   | pwm
 *  10       I2C1_SCL               |  PB6   |pwm
 *  11       I2C1_SDA               |  PB7   |pwm
 *  12       SDIO_D4                |  PB8   |pwm
 *  13       SDIO_D5                |  PB9   |pwm
 *  14                              |  PA0   | "A0"
 *  15                              |  PA1   | "A1"
 *  16       USART2_TX              |  PA2   | "A2",pwm
 *  17       USART2_RX              |  PA3   | "A3",pwm
 *  18       DAC_OUT1,SPI1_NSS      |  PA4   | "DAC0"
 *  19       DAC_OUT2,SPI1_SCK      |  PA5   | "DAC1"
 *  20       I2C2_SDA,USART3_RX     |  PB11  | "SDA"
 *  21       I2C2_SCL,USART3_TX     |  PB10  | "SCL"
 *  22       SPI2_MISO              |  PB14  | "SPI_MISO"
 *  23       SPI2_SCK,I2S2_CK       |  PB13  | "SPI_SCK"
 *  24       SPI2_MOSI,I2S2_SD      |  PB15  | "SPI_MOSI"
 *  25  SPI2_NSS,I2S2_WS,I2C2_SMBA  |  PB12  |
 *  26                              |  PB2   |BOOT1,pulldown with 10k resistor
 *  27       SPI3_NSS,I2S3_WS       |  PA15  |

 *
 * USB pin                          |  PORT
 * ----------------                 +--------
 *  28       USBDM                  |  PA11
 *  29       USBDP                  |  PA12
 */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Pins descriptions
 */
extern const PinDescription g_APinDescription[]=
{
  // 0 .. 53 - Digital pins
  // ----------------------
  // 0/1 - UART (Serial)
  { GPIOA, GPIO_Pin_10,  RCC_APB2Periph_GPIOA, NONE, TIM1, TIM_Channel_3 }, // URXD1,TIM1_CH3,pwm
  { GPIOA,  GPIO_Pin_9,  RCC_APB2Periph_GPIOA, NONE, TIM1, TIM_Channel_2 }, // UTXD1,TIM1_CH2,pwm

  //2,SPI3_SCK
  { GPIOB, GPIO_Pin_3,   RCC_APB2Periph_GPIOB, NONE, NULL, NONE }, // 
  //3,SPI3_MISO
  { GPIOB, GPIO_Pin_4,   RCC_APB2Periph_GPIOB, NONE, NULL, NONE }, // 
  //4,SPI3_MOSI
  { GPIOB, GPIO_Pin_5,   RCC_APB2Periph_GPIOB, NONE, NULL, NONE }, // 
  //5(SPI1_MISO, adc,pwm),6(SPI1_MOSI, adc,pwm)
  { GPIOA, GPIO_Pin_6,  RCC_APB2Periph_GPIOA, ADC_Channel_6, TIM3, TIM_Channel_1}, // TIM3_CH1,pwm
  { GPIOA, GPIO_Pin_7,  RCC_APB2Periph_GPIOA, ADC_Channel_7, TIM3, TIM_Channel_2}, // TIM3_CH2,pwm
  // 7/8 - 
  { GPIOB, GPIO_Pin_0,  RCC_APB2Periph_GPIOB, ADC_Channel_8, TIM3, TIM_Channel_3}, // TIM3_CH3,pwm
  { GPIOB, GPIO_Pin_1,  RCC_APB2Periph_GPIOB, ADC_Channel_9, TIM3, TIM_Channel_4}, // TIM3_CH4,pwm

  //9,pwm
  { GPIOA, GPIO_Pin_8,   RCC_APB2Periph_GPIOA, NONE, TIM1, TIM_Channel_1 }, // pin 9,TIM1_CH1,pwm
  //10(SCL,pwm),11(SDA,pwm)
  { GPIOB, GPIO_Pin_6,   RCC_APB2Periph_GPIOB, NONE, TIM4, TIM_Channel_1 }, // 10,TIM4_CH1,pwm
  { GPIOB, GPIO_Pin_7,   RCC_APB2Periph_GPIOB, NONE, TIM4, TIM_Channel_2 }, // 11,TIM4_CH2,pwm
  // 12,13
  { GPIOB, GPIO_Pin_8,   RCC_APB2Periph_GPIOB, NONE, TIM4, TIM_Channel_3 }, // 12,TIM4_CH3,pwm
  { GPIOB, GPIO_Pin_9,   RCC_APB2Periph_GPIOB, NONE, TIM4, TIM_Channel_4 }, // 13,TIM4_CH4,pwm
  
  // 14 .. 19 - Analog pins
  // 14
  { GPIOA, GPIO_Pin_0,  RCC_APB2Periph_GPIOA, ADC_Channel_0, TIM5, TIM_Channel_1 }, // AD0,TIM5_CH1,pwm
  { GPIOA, GPIO_Pin_1,  RCC_APB2Periph_GPIOA, ADC_Channel_1, TIM2, TIM_Channel_2 }, // AD1,TIM2_CH2/TIM5_CH2,pwm
  //16,pwm
  { GPIOA, GPIO_Pin_2,  RCC_APB2Periph_GPIOA, ADC_Channel_2, TIM2, TIM_Channel_3 }, // AD2,TIM2_CH3/TIM5_CH3,pwm
  //17,pwm
  { GPIOA, GPIO_Pin_3,  RCC_APB2Periph_GPIOA, ADC_Channel_3, TIM2, TIM_Channel_4 }, // AD3,TIM2_CH4/TIM5_CH4,pwm
  // 18/19 - DAC0/DAC1
  { GPIOA, GPIO_Pin_4,  RCC_APB2Periph_GPIOA, ADC_Channel_4, NULL, NONE}, // AD4,DAC1
  { GPIOA, GPIO_Pin_5,  RCC_APB2Periph_GPIOA, ADC_Channel_5, NULL, NONE}, // AD5,DAC2

  // 20 SDA,21 SCL1
  { GPIOB, GPIO_Pin_11,  RCC_APB2Periph_GPIOB, NONE, NULL, NONE }, // PIN 20
  { GPIOB, GPIO_Pin_10,  RCC_APB2Periph_GPIOB, NONE, NULL, NONE }, // PIN 21
  
  // 22,SPI2_MISO
  { GPIOB, GPIO_Pin_14,  RCC_APB2Periph_GPIOB, NONE, NULL, NONE }, // PIN 22
  // 23,SPI2_SCK
  { GPIOB, GPIO_Pin_13,  RCC_APB2Periph_GPIOB, NONE, NULL, NONE }, // PIN 23
  // 24,SPI2_MOSI
  { GPIOB, GPIO_Pin_15,  RCC_APB2Periph_GPIOB, NONE, NULL, NONE }, // PIN 24
  
  // 25
  { GPIOB, GPIO_Pin_12,  RCC_APB2Periph_GPIOB, NONE, NULL, NONE }, // PIN 25
  //26,BOOT1, pulldown withe 10k resistor
  { GPIOB, GPIO_Pin_2,   RCC_APB2Periph_GPIOB, NONE, NULL, NONE }, //BOOT1,PIN 26
  //27
  { GPIOA, GPIO_Pin_15,  RCC_APB2Periph_GPIOA, NONE, NULL, NONE}, //PIN 27

  //28,USBDM(D-)
  { GPIOA, GPIO_Pin_11,   RCC_APB2Periph_GPIOA, NONE, NULL, NONE }, // 
  //29,USBDP(D+)
  { GPIOA, GPIO_Pin_12,   RCC_APB2Periph_GPIOA, NONE, NULL, NONE }, // 

  
  // END
  //{ NULL, 0, 0, PIO_NOT_A_PIN, PIo_default, 0, nO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }
} ;

#ifdef __cplusplus
}
#endif

/*
 * UART objects
 */
//RingBuffer rx_buffer1;

//UARTClass Serial(UART, UART_IRQn, id_uart, &rx_buffer1);
//void serialEvent() __attribute__((weak));
//void serialEvent() { }
//
//// IT handlers
//void UART_Handler(void)
//{
//  //Serial.IrqHandler();
//}

// ----------------------------------------------------------------------------
/*
 * USART objects
 */
RingBuffer rx_buffer1;
RingBuffer rx_buffer2;
RingBuffer rx_buffer3;
//RingBuffer rx_buffer4;

USARTClass Serial(USART1, USART1_IRQn, id_serial, &rx_buffer1);
void serialEvent() __attribute__((weak));
void serialEvent() { }
USARTClass Serial1(USART2, USART2_IRQn, id_serial1, &rx_buffer2);
void serialEvent1() __attribute__((weak));
void serialEvent1() { }
USARTClass Serial2(USART3, USART3_IRQn, id_serial2, &rx_buffer3);
void serialEvent2() __attribute__((weak));
void serialEvent2() { }
//USARTClass Serial3(USART3, USART3_IRQn, id_serial3, &rx_buffer4);
//void serialEvent3() __attribute__((weak));
//void serialEvent3() { }

// IT handlers
void USART1_IRQHandler(void) 
{
  Serial.IrqHandler();//USART1 must be Serial,for usart flash programming.
}

//void UART4_IRQHandler(void) 
//{
//  Serial1.IrqHandler();
//}

void USART2_IRQHandler(void) 
{
  Serial1.IrqHandler();
}

void USART3_IRQHandler(void) 
{
  Serial2.IrqHandler();
}

// ----------------------------------------------------------------------------

void serialEventRun(void)
{
  if (Serial.available()) serialEvent();
  if (Serial1.available()) serialEvent1();
  if (Serial2.available()) serialEvent2();
  //if (Serial3.available()) serialEvent3();
}

// ----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

void __libc_init_array(void);

void init( void )
{
  SystemInit();

  // Set Systick to 1ms interval, common to all SAM3 variants
  if (SysTick_Config(SystemCoreClock / 1000))
  {
    // Capture error
    while (true);
  }
	/* Configure the SysTick Handler Priority: Preemption priority and subpriority */
	NVIC_SetPriority(SysTick_IRQn, 15);	

  // Disable watchdog
  //WDT_Disable(WDT);

  // Initialize C library
  __libc_init_array();

  // default 13pin led will off.
  pinMode(13,OUTPUT);
  digitalWrite(13, LOW);

  //disable JTAG-DP,release pin 2(PB3),3(PB4),19(PA15)
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO , ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
  //remap Timer4
  //GPIO_PinRemapConfig(GPIO_Remap_TIM4,ENABLE);
  //remap USART3
  //GPIO_PinRemapConfig(GPIO_FullRemap_USART3,ENABLE);
  //remap USART2
  //GPIO_PinRemapConfig(GPIO_Remap_USART2,ENABLE);
  //remap CAN1,to PB8,PB9
  GPIO_PinRemapConfig(GPIO_Remap1_CAN1,ENABLE);

  // Initialize Analog Controller

	ADC_InitTypeDef ADC_InitStructure;

	// ADCCLK = PCLK2/4
	// RCC_ADCCLKConfig(RCC_PCLK2_Div4);
	// ADCCLK = PCLK2/6 = 72/6 = 12MHz
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);

	// Enable ADC1 clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	ADC_DeInit(ADC1);

	// ADC1 Configuration
	// ADC1 operate independently
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	// Disable the scan conversion so we do one at a time
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	// Don't do continuous conversions - do them on demand
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	// Start conversion by software, not an external trigger
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	// Conversions are 12 bit - put them in the lower 12 bits of the result
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	// Say how many channels would be used by the sequencer
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	// Now do the setup
	ADC_Init(ADC1, &ADC_InitStructure);

	// Enable ADC1
	ADC_Cmd(ADC1, ENABLE);

	// Enable ADC1 reset calibration register
	ADC_ResetCalibration(ADC1);

	// Check the end of ADC1 reset calibration register
	while(ADC_GetResetCalibrationStatus(ADC1));

	// Start ADC1 calibration
	ADC_StartCalibration(ADC1);

	// Check the end of ADC1 calibration
	while(ADC_GetCalibrationStatus(ADC1));

  // Initialize analogOutput module
  analogOutputInit();

	/* Configure the NVIC Preemption Priority Bits */
	/* 4 bits for pre-emption priority(0-15 PreemptionPriority) and 0 bits for subpriority(0 SubPriority) */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
}

#ifdef __cplusplus
}
#endif

