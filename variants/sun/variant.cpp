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
 * MakerLab Sun Board pin           |  PORT  | Label
 * ---------------------------------+--------+-------
 *   0                              |  PA10  | "RX0",pwm
 *   1                              |  PA9   | "TX0",pwm
 *   2                              |  PD10  |
 *   3                              |  PD11  |
 *   4       TIM4_CH1(remap)        |  PD12  |pwm
 *   5       TIM4_CH2(remap)        |  PD13  |pwm
 *   6       TIM4_CH3(remap)        |  PD14  |pwm
 *   7       TIM4_CH4(remap)        |  PD15  |pwm
 *   8       I2S2_MCK               |  PC6   |pwm
 *   9       I2S3_MCK               |  PC7   |pwm
 *  10       SDIO_D0                |  PC8   |pwm
 *  11       SDIO_D1                |  PC9   |pwm
 *  12                              |  PE0   | 
 *  13                              |  PE1   | LED AMBER "L"
 *  14       USART4_TX,SDIO_D2      |  PC10  | 
 *  15       USART4_RX,SDIO_D3      |  PC11  | 
 *  16       USART2_TX(remap)       |  PD5   |
 *  17       USART2_RX(remap)       |  PD6   |
 *  18       USART3_TX(remap)       |  PD8   |
 *  19       USART3_RX(remap)       |  PD9   |
 *  20       I2C1_SDA               |  PB7   |pwm
 *  21       I2C1_SCL               |  PB6   |pwm
 *  22                              |  PA8   |
 *  23       SPI3_NSS,I2S3_WS       |  PA15  |
 *  24       USART5_TX,SDIO_CK      |  PC12  | 
 *  25       USART5_RX,SDIO_CMD     |  PD2   | 
 *  26                              |  PD3   |
 *  27                              |  PD4   |
 *  28                              |  PD7   |
 *  29       SPI3_SCK,I2S3_CK       |  PB3   |
 *  30       SPI3_MISO              |  PB4   |
 *  31       USBDP(D+)              |  PA12  | shared with native usb
 *  32       USBDM(D-)              |  PA11  | shared with native usb
 *  33  I2C1_SMBA,SPI3_MOSI,I2S3_SD |  PB5   |
 *  34       SDIO_D4                |  PB8   |
 *  35       SDIO_D5                |  PB9   |
 *  36  SPI2_NSS,I2S2_WS,I2C2_SMBA  |  PB12  |
 *  37                              |  PE15  |
 *  38                              |  PE14  |
 *  39                              |  PE13  |
 *  40                              |  PE12  |
 *  41                              |  PE11  |
 *  42                              |  PE10  |
 *  43                              |  PE9   |
 *  44                              |  PE8   |
 *  45                              |  PE7   |
 *  46                              |  PB2   |BOOT1,pulldown with 10k resistor
 *  47                              |  PE6   |
 *  48                              |  PE5   |
 *  49                              |  PE4   | 
 *  50                              |  PE3   |
 *  51                              |  PE2   |
 *  52       SPI1_MOSI              |  PA7   | adc
 *  53       SPI1_MISO              |  PA6   | adc
 *  54                              |  PC0   | "A0"
 *  55                              |  PC1   | "A1"
 *  56                              |  PC2   | "A2"
 *  57                              |  PC3   | "A3"
 *  58                              |  PA0   | "A4"
 *  69                              |  PA1   | "A5"
 *  60       USART2_TX              |  PA2   | "A6",pwm
 *  61       USART2_RX              |  PA3   | "A7",pwm
 *  62                              |  PC4   | "A8"
 *  63                              |  PC5   | "A9"
 *  64                              |  PB0   | "A10",pwm
 *  65                              |  PB1   | "A11",pwm
 *  66       DAC_OUT1,SPI1_NSS      |  PA4   | "DAC0"
 *  67       DAC_OUT2,SPI1_SCK      |  PA5   | "DAC1"
 *  68       CAN1_RX(remap)         |  PD0   | 
 *  69       CAN1_TX(remap)         |  PD1   |
 *  70       I2C2_SDA,USART3_RX     |  PB11  |
 *  71       I2C2_SCL,USART3_TX     |  PB10  |
 *  72       SPI2_MISO              |  PB14  |
 *  73       SPI2_MOSI,I2S2_SD      |  PB15  |
 *  74       SPI2_SCK,I2S2_CK       |  PB13  |

 *  75                              |  PC14  | 32.768khz in
 *  76                              |  PC15  | 32.768khz out
 *
 * USB pin                          |  PORT
 * ----------------                 +--------
 *  32       USBDM                  |  PA11
 *  31       USBDP                  |  PA12
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

  // 2
  { GPIOD, GPIO_Pin_10,  RCC_APB2Periph_GPIOD, NONE, NULL, NONE }, // PIN 34
  { GPIOD, GPIO_Pin_11,  RCC_APB2Periph_GPIOD, NONE, NULL, NONE }, // PIN 33

  //4~7 pwm
  { GPIOD, GPIO_Pin_12,  RCC_APB2Periph_GPIOD, NONE, TIM4, TIM_Channel_1 }, // PIN 4,pwm
  { GPIOD, GPIO_Pin_13,  RCC_APB2Periph_GPIOD, NONE, TIM4, TIM_Channel_2 }, // PIN 5,pwm
  { GPIOD, GPIO_Pin_14,  RCC_APB2Periph_GPIOD, NONE, TIM4, TIM_Channel_3 }, // PIN 6,pwm
  { GPIOD, GPIO_Pin_15,  RCC_APB2Periph_GPIOD, NONE, TIM4, TIM_Channel_4 }, // PIN 7,pwm

  //8~11 pwm
  { GPIOC, GPIO_Pin_6,   RCC_APB2Periph_GPIOC, NONE, TIM8, TIM_Channel_1 }, // PIN 28,TIM8_CH1,pwm
  { GPIOC, GPIO_Pin_7,   RCC_APB2Periph_GPIOC, NONE, TIM8, TIM_Channel_2 }, // PIN 27,TIM8_CH2,pwm
  { GPIOC, GPIO_Pin_8,   RCC_APB2Periph_GPIOC, NONE, TIM8, TIM_Channel_3 }, // PIN 26,TIM8_CH3,pwm
  { GPIOC, GPIO_Pin_9,   RCC_APB2Periph_GPIOC, NONE, TIM8, TIM_Channel_4 }, // PIN 25,TIM8_CH4,pwm

  //12
  { GPIOE, GPIO_Pin_0,   RCC_APB2Periph_GPIOE, NONE, NULL, NONE}, // 
  // 13 - AMBER LED
  { GPIOE, GPIO_Pin_1,   RCC_APB2Periph_GPIOE, NONE, NULL, NONE}, // 

  //14(USART4_TX),15(USARt4_RX)
  { GPIOC, GPIO_Pin_10,  RCC_APB2Periph_GPIOC, NONE, NULL, NONE}, // 
  { GPIOC, GPIO_Pin_11,  RCC_APB2Periph_GPIOC, NONE, NULL, NONE }, // 

  //16(USART2_TX remap),17(USART2_RX remap)
  { GPIOD, GPIO_Pin_5,   RCC_APB2Periph_GPIOD, NONE, NULL, NONE }, // 
  { GPIOD, GPIO_Pin_6,   RCC_APB2Periph_GPIOD, NONE, NULL, NONE }, // 

  //18(USART3_TX remap),19(USART3_RX remap)
  { GPIOD, GPIO_Pin_8,   RCC_APB2Periph_GPIOD, NONE, NULL, NONE }, // PIN 36
  { GPIOD, GPIO_Pin_9,   RCC_APB2Periph_GPIOD, NONE, NULL, NONE }, // PIN 35

  //20(SDA),21(SCL)
  { GPIOB, GPIO_Pin_7,   RCC_APB2Periph_GPIOB, NONE, NULL, NONE }, // 
  { GPIOB, GPIO_Pin_6,   RCC_APB2Periph_GPIOB, NONE, NULL, NONE }, // 

  //22,23,24,25
  { GPIOA, GPIO_Pin_8,   RCC_APB2Periph_GPIOA, NONE, TIM1, TIM_Channel_1 }, // PIN 24,TIM1_CH1,pwm
  { GPIOA, GPIO_Pin_15,  RCC_APB2Periph_GPIOA, NONE, NULL, NONE}, //
  { GPIOC, GPIO_Pin_12,  RCC_APB2Periph_GPIOC, NONE, NULL, NONE }, // 
  { GPIOD, GPIO_Pin_2,   RCC_APB2Periph_GPIOD, NONE, NULL, NONE }, // 

  //26,27,28
  { GPIOD, GPIO_Pin_3,   RCC_APB2Periph_GPIOD, NONE, NULL, NONE }, // 
  { GPIOD, GPIO_Pin_4,   RCC_APB2Periph_GPIOD, NONE, NULL, NONE }, // 
  { GPIOD, GPIO_Pin_7,   RCC_APB2Periph_GPIOD, NONE, NULL, NONE }, // 
  //29,SPI3_SCK
  { GPIOB, GPIO_Pin_3,   RCC_APB2Periph_GPIOB, NONE, NULL, NONE }, // 
  //30,SPI3_MISO
  { GPIOB, GPIO_Pin_4,   RCC_APB2Periph_GPIOB, NONE, NULL, NONE }, // 

  //31,USBDP(D+)
  { GPIOA, GPIO_Pin_12,   RCC_APB2Periph_GPIOA, NONE, NULL, NONE }, // 
  //32,USBDM(D-)
  { GPIOA, GPIO_Pin_11,   RCC_APB2Periph_GPIOA, NONE, NULL, NONE }, // 

  // 33,SPI3_MOSI
  { GPIOB, GPIO_Pin_5,   RCC_APB2Periph_GPIOB, NONE, NULL, NONE }, // 
  // 34,35
  { GPIOB, GPIO_Pin_8,   RCC_APB2Periph_GPIOB, NONE, NULL, NONE }, // 
  { GPIOB, GPIO_Pin_9,   RCC_APB2Periph_GPIOB, NONE, NULL, NONE }, // 
  
  // 36
  { GPIOB, GPIO_Pin_12,  RCC_APB2Periph_GPIOB, NONE, NULL, NONE }, // PIN 40

  //37
  { GPIOE, GPIO_Pin_15,  RCC_APB2Periph_GPIOE, NONE, NULL, NONE }, // PIN 53
  { GPIOE, GPIO_Pin_14,  RCC_APB2Periph_GPIOE, NONE, NULL, NONE }, // PIN 52
  { GPIOE, GPIO_Pin_13,  RCC_APB2Periph_GPIOE, NONE, NULL, NONE }, // PIN 51
  { GPIOE, GPIO_Pin_12,  RCC_APB2Periph_GPIOE, NONE, NULL, NONE }, // PIN 50
  { GPIOE, GPIO_Pin_11,  RCC_APB2Periph_GPIOE, NONE, NULL, NONE }, // PIN 49
  { GPIOE, GPIO_Pin_10,  RCC_APB2Periph_GPIOE, NONE, NULL, NONE }, // PIN 48
  
  //43
  { GPIOE, GPIO_Pin_9,   RCC_APB2Periph_GPIOE, NONE, NULL, NONE }, // PIN 47
  { GPIOE, GPIO_Pin_8,   RCC_APB2Periph_GPIOE, NONE, NULL, NONE }, // PIN 46
  { GPIOE, GPIO_Pin_7,   RCC_APB2Periph_GPIOE, NONE, NULL, NONE }, // PIN 45

  //46,BOOT1, pulldown withe 10k resistor
  { GPIOB, GPIO_Pin_2,   RCC_APB2Periph_GPIOB, NONE, NULL, NONE }, //BOOT1

  //47,48
  { GPIOE, GPIO_Pin_6,   RCC_APB2Periph_GPIOE, NONE, NULL, NONE }, // PIN 44
  { GPIOE, GPIO_Pin_5,   RCC_APB2Periph_GPIOE, NONE, NULL, NONE }, // PIN 43
  // 49,50,51
  { GPIOE, GPIO_Pin_4,   RCC_APB2Periph_GPIOE, NONE, NULL, NONE }, // LED AMBER RXL
  { GPIOE, GPIO_Pin_3,   RCC_APB2Periph_GPIOE, NONE, NULL, NONE }, // PIN 22
  { GPIOE, GPIO_Pin_2,   RCC_APB2Periph_GPIOE, NONE, NULL, NONE }, // PIN 23

  //52(SPI1_MOSI, adc),53(SPI1_MISO, adc)
  { GPIOA, GPIO_Pin_7,  RCC_APB2Periph_GPIOA, ADC_Channel_7, TIM3, TIM_Channel_2}, // AD11,TIM3_CH2,pwm
  { GPIOA, GPIO_Pin_6,  RCC_APB2Periph_GPIOA, ADC_Channel_6, TIM3, TIM_Channel_1}, // AD10,TIM3_CH1,pwm
  
  // 54 .. 65 - Analog pins
  // ----------------------
  { GPIOC, GPIO_Pin_0,  RCC_APB2Periph_GPIOC, ADC_Channel_10, NULL ,NONE }, // AD0
  { GPIOC, GPIO_Pin_1,  RCC_APB2Periph_GPIOC, ADC_Channel_11, NULL ,NONE }, // AD1
  { GPIOC, GPIO_Pin_2,  RCC_APB2Periph_GPIOC, ADC_Channel_12, NULL ,NONE }, // AD2
  { GPIOC, GPIO_Pin_3,  RCC_APB2Periph_GPIOC, ADC_Channel_13, NULL ,NONE }, // AD3

  // 58
  { GPIOA, GPIO_Pin_0,  RCC_APB2Periph_GPIOA, ADC_Channel_0, TIM5, TIM_Channel_1 }, // AD4,TIM5_CH1,pwm
  { GPIOA, GPIO_Pin_1,  RCC_APB2Periph_GPIOA, ADC_Channel_1, TIM2, TIM_Channel_2 }, // AD5,TIM2_CH2/TIM5_CH2,pwm
  //60,pwm
  { GPIOA, GPIO_Pin_2,  RCC_APB2Periph_GPIOA, ADC_Channel_2, TIM2, TIM_Channel_3 }, // AD6,TIM2_CH3/TIM5_CH3,pwm
  //61,pwm
  { GPIOA, GPIO_Pin_3,  RCC_APB2Periph_GPIOA, ADC_Channel_3, TIM2, TIM_Channel_4 }, // AD7,TIM2_CH4/TIM5_CH4,pwm

  //62,63
  { GPIOC, GPIO_Pin_4,  RCC_APB2Periph_GPIOC, ADC_Channel_14,NULL, NONE }, // AD12
  { GPIOC, GPIO_Pin_5,  RCC_APB2Periph_GPIOC, ADC_Channel_15,NULL, NONE }, // AD13
  
  // 64/65 - AD10/AD11
  { GPIOB, GPIO_Pin_0,  RCC_APB2Periph_GPIOB, ADC_Channel_8, TIM3, TIM_Channel_3}, // AD14,TIM3_CH3,pwm
  { GPIOB, GPIO_Pin_1,  RCC_APB2Periph_GPIOB, ADC_Channel_9, TIM3, TIM_Channel_4}, // AD15,TIM3_CH4,pwm

  // 66/67 - DAC0/DAC1
  { GPIOA, GPIO_Pin_4,  RCC_APB2Periph_GPIOA, ADC_Channel_4, NULL, NONE}, // AD8,DAC1
  { GPIOA, GPIO_Pin_5,  RCC_APB2Periph_GPIOA, ADC_Channel_5, NULL, NONE}, // AD9,DAC2

  // 68/69 ,CAN1_RX/CAN1_TX
  { GPIOD, GPIO_Pin_0,   RCC_APB2Periph_GPIOD, NONE, NULL, NONE }, // 
  { GPIOD, GPIO_Pin_1,   RCC_APB2Periph_GPIOD, NONE, NULL, NONE }, // 

  // 70 SDA1,71 SCL1
  { GPIOB, GPIO_Pin_11,  RCC_APB2Periph_GPIOB, NONE, NULL, NONE }, // PIN 41
  { GPIOB, GPIO_Pin_10,  RCC_APB2Periph_GPIOB, NONE, NULL, NONE }, // PIN 42

  // 72,SPI2_MISO
  { GPIOB, GPIO_Pin_14,  RCC_APB2Periph_GPIOB, NONE, NULL, NONE }, // PIN 38
  // 73,SPI2_MOSI
  { GPIOB, GPIO_Pin_15,  RCC_APB2Periph_GPIOB, NONE, NULL, NONE }, // PIN 37
  // 74,SPI2_SCK
  { GPIOB, GPIO_Pin_13,  RCC_APB2Periph_GPIOB, NONE, NULL, NONE }, // PIN 39

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
RingBuffer rx_buffer4;

USARTClass Serial(USART1, USART1_IRQn, id_serial, &rx_buffer1);
void serialEvent() __attribute__((weak));
void serialEvent() { }
USARTClass Serial1(UART4, UART4_IRQn, id_serial1, &rx_buffer2);
void serialEvent1() __attribute__((weak));
void serialEvent1() { }
USARTClass Serial2(USART2, USART2_IRQn, id_serial2, &rx_buffer3);
void serialEvent2() __attribute__((weak));
void serialEvent2() { }
USARTClass Serial3(USART3, USART3_IRQn, id_serial3, &rx_buffer4);
void serialEvent3() __attribute__((weak));
void serialEvent3() { }

// IT handlers
void USART1_IRQHandler(void) 
{
  Serial.IrqHandler();//USART1 must be Serial,for usart flash programming.
}

void UART4_IRQHandler(void) 
{
  Serial1.IrqHandler();
}

void USART2_IRQHandler(void) 
{
  Serial2.IrqHandler();
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
  if (Serial3.available()) serialEvent3();
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

  /*
  // Enable parallel access on PIo output data registers
  PIOA->PIO_OWER = 0xFFFFFFFF;
  PIOB->PIO_OWER = 0xFFFFFFFF;
  PIOC->PIO_OWER = 0xFFFFFFFF;
  PIOD->PIO_OWER = 0xFFFFFFFF;

  // Initialize Serial port U(S)Art pins
  PIO_Configure(
    g_APinDescription[PINS_UART].pport,
    g_APinDescription[PINS_UART].ulpintype,
    g_APinDescription[PINS_UART].ulpin,
    g_APinDescription[PINS_UART].ulpinconfiguration);
  digitalWrite(0, HIGH); // Enable pullup for rx0
  PIO_Configure(
    g_APinDescription[PINS_USART0].pport,
    g_APinDescription[PINS_USART0].ulpintype,
    g_APinDescription[PINS_USART0].ulpin,
    g_APinDescription[PINS_USART0].ulpinconfiguration);
  PIO_Configure(
    g_APinDescription[PINS_USART1].pport,
    g_APinDescription[PINS_USART1].ulpintype,
    g_APinDescription[PINS_USART1].ulpin,
    g_APinDescription[PINS_USART1].ulpinconfiguration);
  PIO_Configure(
    g_APinDescription[PINS_USART3].pport,
    g_APinDescription[PINS_USART3].ulpintype,
    g_APinDescription[PINS_USART3].ulpin,
    g_APinDescription[PINS_USART3].ulpinconfiguration);

  // Initialize USB pins
  PIO_Configure(
    g_APinDescription[PINS_USB].pport,
    g_APinDescription[PINS_USB].ulpintype,
    g_APinDescription[PINS_USB].ulpin,
    g_APinDescription[PINS_USB].ulpinconfiguration);

  // Initialize CAN pins
  PIO_Configure(
    g_APinDescription[PINS_CAN0].pport,
    g_APinDescription[PINS_CAN0].ulpintype,
    g_APinDescription[PINS_CAN0].ulpin,
    g_APinDescription[PINS_CAN0].ulpinconfiguration);
  PIO_Configure(
    g_APinDescription[PINS_CAN1].pport,
    g_APinDescription[PINS_CAN1].ulpintype,
    g_APinDescription[PINS_CAN1].ulPin,
    g_APinDescription[PINS_CAN1].ulPinConfiguration);
*/

  //disable JTAG-DP,release pin 29(PB3),30(PB4),23(PA15)
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO , ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
  //remap Timer4
  GPIO_PinRemapConfig(GPIO_Remap_TIM4,ENABLE);
  //remap USART3
  GPIO_PinRemapConfig(GPIO_FullRemap_USART3,ENABLE);
  //remap USART2
  GPIO_PinRemapConfig(GPIO_Remap_USART2,ENABLE);
  //remap CAN1,to PD0,PD1
  GPIO_PinRemapConfig(GPIO_Remap2_CAN1,ENABLE);

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

