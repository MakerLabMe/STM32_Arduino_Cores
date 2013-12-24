/*
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
 * DUE Board pin                    |  PORT  | Label
 * ---------------------------------+--------+-------
 *   0                              |  PA10  | "RX0",pwm
 *   1                              |  PA9   | "TX0",pwm
 *   2                              |  PD4   |
 *   3       USART2_TX(remap)       |  PD5   |
 *   4       USART2_RX(remap)       |  PD6   |
 *   5                              |  PD7   |
 *   6       SPI3_SCK,I2S3_CK       |  PB3   |
 *   7       SPI3_MISO              |  PB4   |
 *   8  I2C1_SMBA,SPI3_MOSI,I2S3_SD |  PB5   |
 *   9       I2C1_SCL               |  PB6   |pwm
 *  10       I2C1_SDA               |  PB7   |pwm
 *  11       SDIO_D4                |  PB8   |pwm
 *  12       SDIO_D5                |  PB9   |pwm
 *  13                              |  PE0   | not LED 
 *  14                              |  PD3   |
 *  15       USART5_RX,SDIO_CMD     |  PD2   | 
 *  16       CAN1_TX(remap)         |  PD1   |
 *  17       CAN1_RX(remap)         |  PD0   | 
 *  18       USART5_TX,SDIO_CK      |  PC12  | 
 *  19       USART4_RX,SDIO_D3      |  PC11  | 
 *  20       USART4_TX,SDIO_D2      |  PC10  | 
 *  21       SPI3_NSS,I2S3_WS       |  PA15  |
 *  22                              |  PE3   |
 *  23                              |  PE2   |
 *  24                              |  PA8   |
 *  25       SDIO_D1                |  PC9   |pwm
 *  26       SDIO_D0                |  PC8   |pwm
 *  27       I2S3_MCK               |  PC7   |pwm
 *  28       I2S2_MCK               |  PC6   |pwm
 *  29       TIM4_CH4(remap)        |  PD15  |
 *  30       TIM4_CH3(remap)        |  PD14  |
 *  31       TIM4_CH2(remap)        |  PD13  |
 *  32       TIM4_CH1(remap)        |  PD12  |
 *  33                              |  PD11  |
 *  34                              |  PD10  |
 *  35       USART3_RX(remap)       |  PD9   |
 *  36       USART3_TX(remap)       |  PD8   |
 *  37       SPI2_MOSI,I2S2_SD      |  PB15  |
 *  38       SPI2_MISO              |  PB14  |
 *  39       SPI2_SCK,I2S2_CK       |  PB13  |
 *  40  SPI2_NSS,I2S2_WS,I2C2_SMBA  |  PB12  |
 *  41       I2C2_SDA,USART3_RX     |  PB11  |
 *  42       I2C2_SCL,USART3_TX     |  PB10  |
 *  43                              |  PE5   |
 *  44                              |  PE6   |
 *  45                              |  PE7   |
 *  46                              |  PE8   |
 *  47                              |  PE9   |
 *  48                              |  PE10  |
 *  49                              |  PE11  |
 *  50                              |  PE12  |
 *  51                              |  PE13  |
 *  52                              |  PE14  |
 *  53                              |  PE15  |
 *  54                              |  PC0   | "A0"
 *  55                              |  PC1   | "A1"
 *  56                              |  PC2   | "A2"
 *  57                              |  PC3   | "A3"
 *  58                              |  PA0   | "A4"
 *  69                              |  PA1   | "A5"
 *  60       USART2_TX              |  PA2   | "A6",pwm
 *  61       USART2_RX              |  PA3   | "A7",pwm
 *  62       DAC_OUT1,SPI1_NSS      |  PA4   | "A8"
 *  63       DAC_OUT2,SPI1_SCK      |  PA5   | "A9"
 *  64       SPI1_MISO              |  PA6   | "A10"
 *  65       SPI1_MOSI              |  PA7   | "A11"
 *  66                              |  PC4   | "A12"
 *  67                              |  PC5   | "A13"
 *  68                              |  PB0   | "A14",pwm
 *  69                              |  PB1   | "A15",pwm
 *  70                              |  PC14  | "SDA1"
 *  71                              |  PC15  | "SCL1"
 *  72                              |  PE4   | LED AMBER "L"
 *  73                              |  PB2   |BOOT1
not support
           *  73                              |  PA21  | LED AMBER "TX"
           *  74                        MISO  |  PA25  |
           *  75                        MOSI  |  PA26  |
           *  76                        SCLK  |  PA27  |
           *  77                        NPCS0 |  PA28  |
           *  78                        NPCS3 |  PB23  | unconnected!
 *
 * USB pin                          |  PORT
 * ----------------                 +--------
 *  USBDM                           |  PA11
 *  USBDP                           |  PA12
 *  USBDIS                          |  PC13
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
  { GPIOD, GPIO_Pin_4,   RCC_APB2Periph_GPIOD, NONE, NULL, NONE }, // 
  { GPIOD, GPIO_Pin_5,   RCC_APB2Periph_GPIOD, NONE, NULL, NONE }, // 
  { GPIOD, GPIO_Pin_6,   RCC_APB2Periph_GPIOD, NONE, NULL, NONE }, // 

  // 5
  { GPIOD, GPIO_Pin_7,   RCC_APB2Periph_GPIOD, NONE, NULL, NONE }, // 
  { GPIOB, GPIO_Pin_3,   RCC_APB2Periph_GPIOB, NONE, NULL, NONE }, // 
  { GPIOB, GPIO_Pin_4,   RCC_APB2Periph_GPIOB, NONE, NULL, NONE }, // 
  { GPIOB, GPIO_Pin_5,   RCC_APB2Periph_GPIOB, NONE, NULL, NONE }, // 
  { GPIOB, GPIO_Pin_6,   RCC_APB2Periph_GPIOB, NONE, TIM4, TIM_Channel_1 }, // 9,TIM4_CH1,pwm
  // 10
  { GPIOB, GPIO_Pin_7,   RCC_APB2Periph_GPIOB, NONE, TIM4, TIM_Channel_2 }, // 10,TIM4_CH2,pwm
  { GPIOB, GPIO_Pin_8,   RCC_APB2Periph_GPIOB, NONE, TIM4, TIM_Channel_3 }, // 11,TIM4_CH3,pwm
  { GPIOB, GPIO_Pin_9,   RCC_APB2Periph_GPIOB, NONE, TIM4, TIM_Channel_4 }, // 12,TIM4_CH4,pwm

  // 13 - AMBER LED
  { GPIOE, GPIO_Pin_0,   RCC_APB2Periph_GPIOE, NONE, NULL, NONE}, // 

  // 14/15 
  { GPIOD, GPIO_Pin_3,   RCC_APB2Periph_GPIOD, NONE, NULL, NONE }, // 
  { GPIOD, GPIO_Pin_2,   RCC_APB2Periph_GPIOD, NONE, NULL, NONE }, // 

  // 16/17 
  { GPIOD, GPIO_Pin_1,   RCC_APB2Periph_GPIOD, NONE, NULL, NONE }, // 
  { GPIOD, GPIO_Pin_0,   RCC_APB2Periph_GPIOD, NONE, NULL, NONE }, // 

  // 18/19 
  { GPIOC, GPIO_Pin_12,  RCC_APB2Periph_GPIOC, NONE, NULL, NONE }, // 
  { GPIOC, GPIO_Pin_11,  RCC_APB2Periph_GPIOC, NONE, NULL, NONE }, // 

  // 20/21 
  { GPIOC, GPIO_Pin_10,  RCC_APB2Periph_GPIOC, NONE, NULL, NONE}, // 
  { GPIOA, GPIO_Pin_15,  RCC_APB2Periph_GPIOA, NONE, NULL, NONE}, //

  // 22
  { GPIOE, GPIO_Pin_3,   RCC_APB2Periph_GPIOE, NONE, NULL, NONE }, // PIN 22
  { GPIOE, GPIO_Pin_2,   RCC_APB2Periph_GPIOE, NONE, NULL, NONE }, // PIN 23
  { GPIOA, GPIO_Pin_8,   RCC_APB2Periph_GPIOA, NONE, TIM1, TIM_Channel_1 }, // PIN 24,TIM1_CH1,pwm
  { GPIOC, GPIO_Pin_9,   RCC_APB2Periph_GPIOC, NONE, TIM8, TIM_Channel_4 }, // PIN 25,TIM8_CH4,pwm

  // 26
  { GPIOC, GPIO_Pin_8,   RCC_APB2Periph_GPIOC, NONE, TIM8, TIM_Channel_3 }, // PIN 26,TIM8_CH3,pwm
  { GPIOC, GPIO_Pin_7,   RCC_APB2Periph_GPIOC, NONE, TIM8, TIM_Channel_2 }, // PIN 27,TIM8_CH2,pwm
  { GPIOC, GPIO_Pin_6,   RCC_APB2Periph_GPIOC, NONE, TIM8, TIM_Channel_1 }, // PIN 28,TIM8_CH1,pwm
  { GPIOD, GPIO_Pin_15,  RCC_APB2Periph_GPIOD, NONE, NULL, NONE }, // PIN 29

  // 30
  { GPIOD, GPIO_Pin_14,  RCC_APB2Periph_GPIOD, NONE, NULL, NONE }, // PIN 30
  { GPIOD, GPIO_Pin_13,  RCC_APB2Periph_GPIOD, NONE, NULL, NONE }, // PIN 31
  { GPIOD, GPIO_Pin_12,  RCC_APB2Periph_GPIOD, NONE, NULL, NONE }, // PIN 32
  { GPIOD, GPIO_Pin_11,  RCC_APB2Periph_GPIOD, NONE, NULL, NONE }, // PIN 33

  // 34
  { GPIOD, GPIO_Pin_10,  RCC_APB2Periph_GPIOD, NONE, NULL, NONE }, // PIN 34
  { GPIOD, GPIO_Pin_9,   RCC_APB2Periph_GPIOD, NONE, NULL, NONE }, // PIN 35
  { GPIOD, GPIO_Pin_8,   RCC_APB2Periph_GPIOD, NONE, NULL, NONE }, // PIN 36
  { GPIOB, GPIO_Pin_15,  RCC_APB2Periph_GPIOB, NONE, NULL, NONE }, // PIN 37

  // 38
  { GPIOB, GPIO_Pin_14,  RCC_APB2Periph_GPIOB, NONE, NULL, NONE }, // PIN 38
  { GPIOB, GPIO_Pin_13,  RCC_APB2Periph_GPIOB, NONE, NULL, NONE }, // PIN 39
  { GPIOB, GPIO_Pin_12,  RCC_APB2Periph_GPIOB, NONE, NULL, NONE }, // PIN 40
  { GPIOB, GPIO_Pin_11,  RCC_APB2Periph_GPIOB, NONE, NULL, NONE }, // PIN 41

  // 42
  { GPIOB, GPIO_Pin_10,  RCC_APB2Periph_GPIOB, NONE, NULL, NONE }, // PIN 42
  { GPIOE, GPIO_Pin_5,   RCC_APB2Periph_GPIOE, NONE, NULL, NONE }, // PIN 43
  { GPIOE, GPIO_Pin_6,   RCC_APB2Periph_GPIOE, NONE, NULL, NONE }, // PIN 44
  { GPIOE, GPIO_Pin_7,   RCC_APB2Periph_GPIOE, NONE, NULL, NONE }, // PIN 45

  // 46
  { GPIOE, GPIO_Pin_8,   RCC_APB2Periph_GPIOE, NONE, NULL, NONE }, // PIN 46
  { GPIOE, GPIO_Pin_9,   RCC_APB2Periph_GPIOE, NONE, NULL, NONE }, // PIN 47
  { GPIOE, GPIO_Pin_10,  RCC_APB2Periph_GPIOE, NONE, NULL, NONE }, // PIN 48
  { GPIOE, GPIO_Pin_11,  RCC_APB2Periph_GPIOE, NONE, NULL, NONE }, // PIN 49

  // 50
  { GPIOE, GPIO_Pin_12,  RCC_APB2Periph_GPIOE, NONE, NULL, NONE }, // PIN 50
  { GPIOE, GPIO_Pin_13,  RCC_APB2Periph_GPIOE, NONE, NULL, NONE }, // PIN 51
  { GPIOE, GPIO_Pin_14,  RCC_APB2Periph_GPIOE, NONE, NULL, NONE }, // PIN 52
  { GPIOE, GPIO_Pin_15,  RCC_APB2Periph_GPIOE, NONE, NULL, NONE }, // PIN 53


  // 54 .. 65 - Analog pins
  // ----------------------
  { GPIOC, GPIO_Pin_0,  RCC_APB2Periph_GPIOC, ADC_Channel_10, NULL ,NONE }, // AD0
  { GPIOC, GPIO_Pin_1,  RCC_APB2Periph_GPIOC, ADC_Channel_11, NULL ,NONE }, // AD1
  { GPIOC, GPIO_Pin_2,  RCC_APB2Periph_GPIOC, ADC_Channel_12, NULL ,NONE }, // AD2
  { GPIOC, GPIO_Pin_3,  RCC_APB2Periph_GPIOC, ADC_Channel_13, NULL ,NONE }, // AD3
  // 58
  { GPIOA, GPIO_Pin_0,  RCC_APB2Periph_GPIOA, ADC_Channel_0, TIM5, TIM_Channel_1 }, // AD4,TIM5_CH1,pwm
  { GPIOA, GPIO_Pin_1,  RCC_APB2Periph_GPIOA, ADC_Channel_1, TIM2, TIM_Channel_2 }, // AD5,TIM2_CH2/TIM5_CH2,pwm
  { GPIOA, GPIO_Pin_2,  RCC_APB2Periph_GPIOA, ADC_Channel_2, TIM2, TIM_Channel_3 }, // AD6,TIM2_CH3/TIM5_CH3,pwm
  { GPIOA, GPIO_Pin_3,  RCC_APB2Periph_GPIOA, ADC_Channel_3, TIM2, TIM_Channel_4 }, // AD7,TIM2_CH4/TIM5_CH4,pwm
  // 62
  { GPIOA, GPIO_Pin_4,  RCC_APB2Periph_GPIOA, ADC_Channel_4, NULL, NONE}, // AD8,DAC1
  { GPIOA, GPIO_Pin_5,  RCC_APB2Periph_GPIOA, ADC_Channel_5, NULL, NONE}, // AD9,DAC2
  { GPIOA, GPIO_Pin_6,  RCC_APB2Periph_GPIOA, ADC_Channel_6, TIM3, TIM_Channel_1}, // AD10,TIM3_CH1,pwm
  { GPIOA, GPIO_Pin_7,  RCC_APB2Periph_GPIOA, ADC_Channel_7, TIM3, TIM_Channel_2}, // AD11,TIM3_CH2,pwm

  // 66/67 - DAC0/DAC1
  { GPIOC, GPIO_Pin_4,  RCC_APB2Periph_GPIOC, ADC_Channel_14,NULL, NONE }, // AD12
  { GPIOC, GPIO_Pin_5,  RCC_APB2Periph_GPIOC, ADC_Channel_15,NULL, NONE }, // AD13

  // 68/69 - AD14/AD15
  { GPIOB, GPIO_Pin_0,  RCC_APB2Periph_GPIOB, ADC_Channel_8, TIM3, TIM_Channel_3}, // AD14,TIM3_CH3,pwm
  { GPIOB, GPIO_Pin_1,  RCC_APB2Periph_GPIOB, ADC_Channel_9, TIM3, TIM_Channel_4}, // AD15,TIM3_CH4,pwm

  // 70/71 - TWI0
  { GPIOC, GPIO_Pin_14,  RCC_APB2Periph_GPIOC, NONE, NULL, NONE }, // TWD0 - SDA1
  { GPIOC, GPIO_Pin_15,  RCC_APB2Periph_GPIOC, NONE, NULL, NONE }, // TWCK0 - SCL1

  // 72/73 - LEDs
  { GPIOE, GPIO_Pin_4,   RCC_APB2Periph_GPIOE, NONE, NULL, NONE }, // LED AMBER RXL
  { GPIOB, GPIO_Pin_2,   RCC_APB2Periph_GPIOB, NONE, NULL, NONE }, //BOOT1
  /*
  { PIOA, PIO_PA21,          ID_Pioa, pio_output_0, PIO_DEFAULT}, // LED AMBER TXL

  // 74/75/76 - SPI
  { PIOA, PIO_PA25A_SPI0_MISO,ID_pioa,pio_periph_A, PIO_DEFAULT}, // MISO
  { PIOA, PIO_PA26A_SPI0_MOSI,ID_pioa,pio_periph_A, PIO_DEFAULT}, // MOSI
  { PIOA, PIO_PA27A_SPI0_SPCK,ID_pioa,pio_periph_A, PIO_DEFAULT}, // SPCK

  // 77 - SPI CS0
  { PIOA, PIO_PA28A_SPI0_NPCS0,ID_pioa,pio_peripH_A,PIO_DEFAULT}, // NPCS0

  // 78 - SPI CS3 (unconnected)
  { PIOB, PIO_PB23B_SPI0_NPCS3,ID_piob,pio_peripH_B,PIO_DEFAULT}, // NPCS3

  // 79 .. 84 - "All pins" masks

  // 79 - TWI0 all pins
  { PIOA, PIO_PA17A_TWD0|PIO_PA18a_twck0, id_pioA, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },
  // 80 - TWI1 all pins
  { PIOB, PIO_PB12A_TWD1|PIO_PB13a_twck1, id_pioB, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },
  // 81 - UART (Serial) all pins
  { PIOA, PIO_PA8A_URXD|PIO_PA9A_utxd, id_pioa, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },
  // 82 - USART0 (Serial1) all pins
  { PIOA, PIO_PA11A_TXD0|PIO_PA10a_rxd0, id_pioa, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },
  // 83 - USART1 (Serial2) all pins
  { PIOA, PIO_PA13A_TXD1|PIO_PA12a_rxd1, id_pioa, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },
  // 84 - USART3 (Serial3) all pins
  { PIOD, PIO_PD4B_TXD3|PIO_PD5B_rxd3, id_piod, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },

  // 85 - USB
  { PIOB, PIO_PB11A_UOTGID|PIO_PB10a_uotgvbof, iD_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // ID - VBOF

  // 86 - SPI CS2
  { PIOB, PIO_PB21B_SPI0_NPCS2, Id_piob, pio_perIPH_B, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // NPCS2

  // 87 - SPI CS1
  { PIOA, PIO_PA29A_SPI0_NPCS1, Id_pioa, pio_perIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // NPCS1

  // 88/89 - CANRX1/CANTX1 (same physical pin for 66/53)
  { PIOB, PIO_PB15A_CANRX1,     Id_piob, pio_perIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // CANRX1
  { PIOB, PIO_PB14A_CANTX1,     Id_piob, pio_perIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // CANTX1

  // 90 .. 91 - "All CAN pins" masks
  // 90 - CAN0 all pins
  { PIOA, PIO_PA1A_CANRX0|PIO_PA0a_cantx0, id_piOA, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC,  NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER },
  // 91 - CAN1 all pins
  { PIOB, PIO_PB15A_CANRX1|PIO_PB14a_cantx1, id_PIOB, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER },

  // END
  { NULL, 0, 0, PIO_NOT_A_PIN, PIo_default, 0, nO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }
  */
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
RingBuffer rx_buffer2;
RingBuffer rx_buffer3;
RingBuffer rx_buffer4;

USARTClass Serial(USART1, USART1_IRQn, id_usart1, &rx_buffer2);
void serialEvent1() __attribute__((weak));
void serialEvent1() { }
USARTClass Serial2(USART2, USART2_IRQn, id_usart2, &rx_buffer3);
void serialEvent2() __attribute__((weak));
void serialEvent2() { }
USARTClass Serial3(USART3, USART3_IRQn, id_usart3, &rx_buffer4);
void serialEvent3() __attribute__((weak));
void serialEvent3() { }

// IT handlers
void USART1_IRQHandler(void) 
{
  Serial.IrqHandler();//USART1 must be Serial,for usart flash programming.
}

void USART2_IRQHandler(void) 
{
  Serial2.IrqHandler();
}

void USART3_IRQHandler(void) 
{
  Serial3.IrqHandler();
}

// ----------------------------------------------------------------------------

void serialEventRun(void)
{
  //if (Serial.available()) serialEvent();
  if (Serial.available()) serialEvent1();
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

  // Disable pull-up on every pin
  for (int i = 0; i < PINS_COUNT; i++)
	  digitalWrite(i, LOW);

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

  //disable JTAG-DP,release pin 6(PB3),7(PB4),21(PA15)
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
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

