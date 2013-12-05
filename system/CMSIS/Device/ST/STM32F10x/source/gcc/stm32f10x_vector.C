/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************
* File Name          : stm32f10x_vector.c
* Author             : MCD Application Team
* Version            : V1.0
* Date               : 10/08/2007
* Description        : This file contains the vector table for STM32F10x.
*                      After Reset the Cortex-M3 processor is in Thread mode,
*                      priority is Privileged, and the Stack is set to Main.
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_conf.h"
#include "stm32f10x.h"
#include "stm32_it.h"
#include "core_cm3.h"

/* Private typedef -----------------------------------------------------------*/
typedef void( *intfunc )( void );
typedef union { intfunc __fun; void * __ptr; } intvec_elem;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#define BootRAM (void *) 0xF108F85F /* workaround for booting from RAM */

//extern const LIB_Interface LIB_Offsets;

#if defined(__IAR_SYSTEMS_ICC__)

# if (__VER__ < 500)
#  define __iar_program_start __program_start
# endif

#pragma language=extended
#pragma segment="CSTACK"

void __iar_program_start( void );

#pragma location = ".intvec"
/* STM32F10x Vector Table entries */
const intvec_elem __vector_table[] =
{
  { .__ptr = __sfe( "CSTACK" ) },
  __iar_program_start,

#else

void Reset_Handler( void );
void *_estack;

const intvec_elem __vector_table[] __attribute__ ((section(".isr_vector"))) =
{
  { .__ptr = &_estack },
  Reset_Handler,

#endif /* IAR */
  NMI_Handler      ,
  HardFault_Handler,
  MemManage_Handler,
  BusFault_Handler ,
  UsageFault_Handler,
  0, 0, 0, 0,            /* Reserved */ 
  SVC_Handler,
  DebugMon_Handler,
  0,                      /* Reserved */
  PendSV_Handler,
  SysTickHandler,
  WWDG_IRQHandler,
  PVD_IRQHandler,
  TAMPER_IRQHandler,
  RTC_IRQHandler,
  FLASH_IRQHandler,
  RCC_IRQHandler,
  EXTI0_IRQHandler,
  EXTI1_IRQHandler,
  EXTI2_IRQHandler,
  EXTI3_IRQHandler,
  EXTI4_IRQHandler,
  DMAChannel1_IRQHandler,
  DMAChannel2_IRQHandler,
  DMAChannel3_IRQHandler,
  DMAChannel4_IRQHandler,
  DMAChannel5_IRQHandler,
  DMAChannel6_IRQHandler,
  DMAChannel7_IRQHandler,
  ADC_IRQHandler,
  USB_HP_CAN_TX_IRQHandler,
  USB_LP_CAN_RX0_IRQHandler,
  CAN_RX1_IRQHandler,
  CAN_SCE_IRQHandler,
  EXTI9_5_IRQHandler,
  TIM1_BRK_IRQHandler,
  TIM1_UP_IRQHandler,
  TIM1_TRG_COM_IRQHandler,
  TIM1_CC_IRQHandler,
  TIM2_IRQHandler,
  TIM3_IRQHandler,
  TIM4_IRQHandler,
  I2C1_EV_IRQHandler,
  I2C1_ER_IRQHandler,
  I2C2_EV_IRQHandler,
  I2C2_ER_IRQHandler,
  SPI1_IRQHandler,
  SPI2_IRQHandler,
  USART1_IRQHandler,
  USART2_IRQHandler,
  USART3_IRQHandler,
  EXTI15_10_IRQHandler,
  RTCAlarm_IRQHandler,
  USBWakeUp_IRQHandler,
  TIM8_BRK_IRQHandler       ,
  TIM8_UP_IRQHandler        ,
  TIM8_TRG_COM_IRQHandler   ,
  TIM8_CC_IRQHandler        ,
  ADC3_IRQHandler           ,
  FSMC_IRQHandler           ,
  SDIO_IRQHandler           ,
  TIM5_IRQHandler           ,
  SPI3_IRQHandler           ,
  UART4_IRQHandler          ,
  UART5_IRQHandler          ,
  TIM6_IRQHandler           ,
  TIM7_IRQHandler           ,
  DMA2_Channel1_IRQHandler  ,
  DMA2_Channel2_IRQHandler  ,
  DMA2_Channel3_IRQHandler  ,
  DMA2_Channel4_5_IRQHandler,
  0, 0, 0, 0, 0, 0, 0,
  {.__ptr = BootRAM},
};

/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/

