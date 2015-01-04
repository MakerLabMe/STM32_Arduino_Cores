/*
  Copyright (c) 2014 MakerLab.me & Andy Sze(andy.sze.mail@gmail.com)  All right reserved.
  Copyright (c) 2012 Arduino.  All right reserved.

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
#include "Reset.h"

#ifdef __cplusplus
extern "C" {
#endif

static void __halt() {
	// Halts
	while (1)
		;
}

extern void svcHook(void);
extern void pendSVHook(void);
extern int sysTickHook(void);

/* Cortex-M3/M4 core handlers */
void NMI_Handler       (void) __attribute__ ((weak, alias("__halt")));
void HardFault_Handler (void) __attribute__ ((weak, alias("__halt")));
void MemManage_Handler (void) __attribute__ ((weak, alias("__halt")));
void BusFault_Handler  (void) __attribute__ ((weak, alias("__halt")));
void UsageFault_Handler(void) __attribute__ ((weak, alias("__halt")));
void DebugMon_Handler  (void) __attribute__ ((weak, alias("__halt")));
void SVC_Handler       (void) { svcHook(); }
void PendSV_Handler    (void) {	pendSVHook(); }

void SysTick_Handler(void)
{
	if (sysTickHook())
		return;

	tickReset();

	// Increment tick count each ms
	TimeTick_Increment();
}

/* Peripherals handlers */
void WWDG_IRQHandler                 (void) __attribute__ ((weak, alias("__halt")));
void PVD_IRQHandler                  (void) __attribute__ ((weak, alias("__halt")));

#if defined(STM32F10X_HD) || defined(STM32F10X_MD) //for F10x handlers
void TAMPER_IRQHandler               (void) __attribute__ ((weak, alias("__halt")));
void RTC_IRQHandler                  (void) __attribute__ ((weak, alias("__halt")));
void DMA1_Channel1_IRQHandler        (void) __attribute__ ((weak, alias("__halt")));
void DMA1_Channel2_IRQHandler        (void) __attribute__ ((weak, alias("__halt")));
void DMA1_Channel3_IRQHandler        (void) __attribute__ ((weak, alias("__halt")));
void DMA1_Channel4_IRQHandler        (void) __attribute__ ((weak, alias("__halt")));
void DMA1_Channel5_IRQHandler        (void) __attribute__ ((weak, alias("__halt")));
void DMA1_Channel6_IRQHandler        (void) __attribute__ ((weak, alias("__halt")));
void DMA1_Channel7_IRQHandler        (void) __attribute__ ((weak, alias("__halt")));
void ADC1_2_IRQHandler               (void) __attribute__ ((weak, alias("__halt")));
void USB_HP_CAN1_TX_IRQHandler       (void) __attribute__ ((weak, alias("__halt")));
void USB_LP_CAN1_RX0_IRQHandler      (void) __attribute__ ((weak, alias("__halt")));
void TIM1_BRK_TIM15_IRQHandler       (void) __attribute__ ((weak, alias("__halt")));
void TIM1_UP_TIM16_IRQHandler        (void) __attribute__ ((weak, alias("__halt")));
void TIM1_TRG_COM_TIM17_IRQHandler   (void) __attribute__ ((weak, alias("__halt")));
void RTCAlarm_IRQHandler             (void) __attribute__ ((weak, alias("__halt")));
void USBWakeUp_IRQHandler            (void) __attribute__ ((weak, alias("__halt")));

#ifdef STM32F10X_HD //only for High desinity
void TIM8_BRK_IRQHandler             (void) __attribute__ ((weak, alias("__halt")));
void TIM8_UP_IRQHandler              (void) __attribute__ ((weak, alias("__halt")));
void TIM8_TRG_COM_IRQHandler         (void) __attribute__ ((weak, alias("__halt")));
void TIM8_CC_IRQHandler              (void) __attribute__ ((weak, alias("__halt")));
void ADC3_IRQHandler                 (void) __attribute__ ((weak, alias("__halt")));
void FSMC_IRQHandler                 (void) __attribute__ ((weak, alias("__halt")));
void SDIO_IRQHandler                 (void) __attribute__ ((weak, alias("__halt")));
void TIM5_IRQHandler                 (void) __attribute__ ((weak, alias("__halt")));
void SPI3_IRQHandler                 (void) __attribute__ ((weak, alias("__halt")));
void UART5_IRQHandler                (void) __attribute__ ((weak, alias("__halt")));
void TIM6_IRQHandler                 (void) __attribute__ ((weak, alias("__halt")));
void TIM7_IRQHandler                 (void) __attribute__ ((weak, alias("__halt")));
void DMA2_Channel1_IRQHandler        (void) __attribute__ ((weak, alias("__halt")));
void DMA2_Channel2_IRQHandler        (void) __attribute__ ((weak, alias("__halt")));
void DMA2_Channel3_IRQHandler        (void) __attribute__ ((weak, alias("__halt")));
void DMA2_Channel4_5_IRQHandler      (void) __attribute__ ((weak, alias("__halt")));
#endif //STM32F10X_HD


#elif defined (STM32F40_41xxx)     //for F4 handlers
void TAMP_STAMP_IRQHandler               (void) __attribute__ ((weak, alias("__halt")));
void RTC_WKUP_IRQHandler                  (void) __attribute__ ((weak, alias("__halt")));
void DMA1_Stream0_IRQHandler         ( void ) __attribute__ ((weak, alias("__halt")));
void DMA1_Stream1_IRQHandler         ( void ) __attribute__ ((weak, alias("__halt")));
void DMA1_Stream2_IRQHandler         ( void ) __attribute__ ((weak, alias("__halt")));
void DMA1_Stream3_IRQHandler         ( void ) __attribute__ ((weak, alias("__halt")));
void DMA1_Stream4_IRQHandler         ( void ) __attribute__ ((weak, alias("__halt")));
void DMA1_Stream5_IRQHandler         ( void ) __attribute__ ((weak, alias("__halt")));
void DMA1_Stream6_IRQHandler         ( void ) __attribute__ ((weak, alias("__halt")));
void ADC_IRQHandler                  ( void ) __attribute__ ((weak, alias("__halt")));
void CAN1_TX_IRQHandler              ( void ) __attribute__ ((weak, alias("__halt")));
void CAN1_RX0_IRQHandler             ( void ) __attribute__ ((weak, alias("__halt")));
void TIM1_BRK_TIM9_IRQHandler       (void) __attribute__ ((weak, alias("__halt")));
void TIM1_UP_TIM10_IRQHandler        (void) __attribute__ ((weak, alias("__halt")));
void TIM1_TRG_COM_TIM11_IRQHandler   (void) __attribute__ ((weak, alias("__halt")));
void RTC_Alarm_IRQHandler             (void) __attribute__ ((weak, alias("__halt")));
void OTG_FS_WKUP_IRQHandler            (void) __attribute__ ((weak, alias("__halt")));

void TIM8_BRK_TIM12_IRQHandler        ( void ) __attribute__ ((weak, alias("__halt")));
void TIM8_UP_TIM13_IRQHandler         ( void ) __attribute__ ((weak, alias("__halt")));
void TIM8_TRG_COM_TIM14_IRQHandler    ( void ) __attribute__ ((weak, alias("__halt")));
void TIM8_CC_IRQHandler               ( void ) __attribute__ ((weak, alias("__halt")));
void DMA1_Stream7_IRQHandler          ( void ) __attribute__ ((weak, alias("__halt")));
void FSMC_IRQHandler                  ( void ) __attribute__ ((weak, alias("__halt")));
void SDIO_IRQHandler                  ( void ) __attribute__ ((weak, alias("__halt")));
void TIM5_IRQHandler                  ( void ) __attribute__ ((weak, alias("__halt")));
void SPI3_IRQHandler                  ( void ) __attribute__ ((weak, alias("__halt")));
void UART4_IRQHandler                 ( void ) __attribute__ ((weak, alias("__halt")));
void UART5_IRQHandler                 ( void ) __attribute__ ((weak, alias("__halt")));
void TIM6_DAC_IRQHandler              ( void ) __attribute__ ((weak, alias("__halt")));
void TIM7_IRQHandler                  ( void ) __attribute__ ((weak, alias("__halt")));

void DMA2_Stream0_IRQHandler          ( void ) __attribute__ ((weak, alias("__halt")));
void DMA2_Stream1_IRQHandler          ( void ) __attribute__ ((weak, alias("__halt")));
void DMA2_Stream2_IRQHandler          ( void ) __attribute__ ((weak, alias("__halt")));
void DMA2_Stream3_IRQHandler          ( void ) __attribute__ ((weak, alias("__halt")));
void DMA2_Stream4_IRQHandler          ( void ) __attribute__ ((weak, alias("__halt")));

void  ETH_IRQHandler           (void) __attribute__ ((weak, alias("__halt")));         /* Ethernet                     */                   
void  ETH_WKUP_IRQHandler      (void) __attribute__ ((weak, alias("__halt")));         /* Ethernet Wakeup through EXTI line */                     
void  CAN2_TX_IRQHandler       (void) __attribute__ ((weak, alias("__halt")));         /* CAN2 TX                      */                          
void  CAN2_RX0_IRQHandler      (void) __attribute__ ((weak, alias("__halt")));         /* CAN2 RX0                     */                          
void  CAN2_RX1_IRQHandler      (void) __attribute__ ((weak, alias("__halt")));         /* CAN2 RX1                     */                          
void  CAN2_SCE_IRQHandler      (void) __attribute__ ((weak, alias("__halt")));         /* CAN2 SCE                     */                          
void  OTG_FS_IRQHandler        (void) __attribute__ ((weak, alias("__halt")));         /* USB OTG FS                   */                   
void  DMA2_Stream5_IRQHandler  (void) __attribute__ ((weak, alias("__halt")));         /* DMA2 Stream 5                */                   
void  DMA2_Stream6_IRQHandler  (void) __attribute__ ((weak, alias("__halt")));         /* DMA2 Stream 6                */                   
void  DMA2_Stream7_IRQHandler  (void) __attribute__ ((weak, alias("__halt")));         /* DMA2 Stream 7                */                   
void  USART6_IRQHandler        (void) __attribute__ ((weak, alias("__halt")));         /* USART6                       */                    
void  I2C3_EV_IRQHandler       (void) __attribute__ ((weak, alias("__halt")));         /* I2C3 event                   */                          
void  I2C3_ER_IRQHandler       (void) __attribute__ ((weak, alias("__halt")));         /* I2C3 error                   */                          
void  OTG_HS_EP1_OUT_IRQHandler(void) __attribute__ ((weak, alias("__halt")));         /* USB OTG HS End Point 1 Out   */                   
void  OTG_HS_EP1_IN_IRQHandler (void) __attribute__ ((weak, alias("__halt")));         /* USB OTG HS End Point 1 In    */                   
void  OTG_HS_WKUP_IRQHandler   (void) __attribute__ ((weak, alias("__halt")));         /* USB OTG HS Wakeup through EXTI */                         
void  OTG_HS_IRQHandler        (void) __attribute__ ((weak, alias("__halt")));         /* USB OTG HS                   */                   
void  DCMI_IRQHandler          (void) __attribute__ ((weak, alias("__halt")));         /* DCMI                         */                   
void  CRYP_IRQHandler          (void) __attribute__ ((weak, alias("__halt")));         /* CRYP crypto                  */                   
void  HASH_RNG_IRQHandler      (void) __attribute__ ((weak, alias("__halt")));         /* Hash and Rng                 */
void  FPU_IRQHandler           (void) __attribute__ ((weak, alias("__halt")));         /* FPU                          */                         

#endif
void FLASH_IRQHandler                (void) __attribute__ ((weak, alias("__halt")));
void RCC_IRQHandler                  (void) __attribute__ ((weak, alias("__halt")));

void CAN1_RX1_IRQHandler             (void) __attribute__ ((weak, alias("__halt")));
void CAN1_SCE_IRQHandler             (void) __attribute__ ((weak, alias("__halt")));
void TIM1_CC_IRQHandler              (void) __attribute__ ((weak, alias("__halt")));
void TIM2_IRQHandler                 (void) __attribute__ ((weak, alias("__halt")));
void TIM3_IRQHandler                 (void) __attribute__ ((weak, alias("__halt")));
void TIM4_IRQHandler                 (void) __attribute__ ((weak, alias("__halt")));
void I2C1_EV_IRQHandler              (void) __attribute__ ((weak, alias("__halt")));
void I2C1_ER_IRQHandler              (void) __attribute__ ((weak, alias("__halt")));
void I2C2_EV_IRQHandler              (void) __attribute__ ((weak, alias("__halt")));
void I2C2_ER_IRQHandler              (void) __attribute__ ((weak, alias("__halt")));
void SPI1_IRQHandler                 (void) __attribute__ ((weak, alias("__halt")));
void SPI2_IRQHandler                 (void) __attribute__ ((weak, alias("__halt")));

#ifdef __cplusplus
}
#endif

