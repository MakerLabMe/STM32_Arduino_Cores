/**
 ******************************************************************************
 * @file    stm32f10x_conf.h
 * @author  Satish Nair, Zachary Crockett and Mohit Bhoite
 * @version V1.0.0
 * @date    21-January-2013
 * @brief   Library configuration file.
 ******************************************************************************
  Copyright (c) 2014 MakerLab.me & Andy Sze(andy.sze.mail@gmail.com).  All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation, either
  version 3 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, see <http://www.gnu.org/licenses/>.
  ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_CONF_H
#define __STM32F10x_CONF_H

/* Includes ------------------------------------------------------------------*/
/* Uncomment the line below to enable peripheral header file inclusion */
#include "stm.h"

#if defined (STM32F10X_MD) || defined (STM32F10X_HD)
#include "include/stm32f10x_adc.h"
#include "include/stm32f10x_bkp.h"
#include "include/stm32f10x_can.h"
#include "include/stm32f10x_crc.h"
#include "include/stm32f10x_dac.h"
#include "include/stm32f10x_dbgmcu.h"
#include "include/stm32f10x_dma.h"
#include "include/stm32f10x_exti.h"
#include "include/stm32f10x_flash.h"
#include "include/stm32f10x_fsmc.h"
#include "include/stm32f10x_gpio.h"
#include "include/stm32f10x_i2c.h" 
#include "include/stm32f10x_iwdg.h"
#include "include/stm32f10x_pwr.h"
#include "include/stm32f10x_rcc.h"
#include "include/stm32f10x_rtc.h"
#include "include/stm32f10x_sdio.h"
#include "include/stm32f10x_spi.h"
#include "include/stm32f10x_tim.h"
#include "include/stm32f10x_usart.h"
#include "include/stm32f10x_wwdg.h"
#include "include/misc.h"   /* High level functions for NVIC and SysTick (add-on to CMSIS functions) */

#elif defined (STM32F40_41xxx)
#include "../libstmf4/stm32f4xx_conf.h"
#endif

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Uncomment the line below to expanse the "assert_param" macro in the 
 Standard Peripheral Library drivers code */
/* #define USE_FULL_ASSERT    1 */

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT

/*******************************************************************************
 * Macro Name     : assert_param
 * Description    : The assert_param macro is used for function's parameters check.
 * Input          : - expr: If expr is false, it calls assert_failed function
 *                    which reports the name of the source file and the source
 *                    line number of the call that failed.
 *                    If expr is true, it returns no value.
 * Return         : None
 *******************************************************************************/
#define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
void assert_failed(uint8_t* file, uint32_t line);
#else
#define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */

#endif /* __STM32F10x_CONF_H */

