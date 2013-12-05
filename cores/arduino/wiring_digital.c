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

#include "Arduino.h"

#ifdef __cplusplus
 extern "C" {
#endif

extern void pinMode( uint32_t ulPin, uint32_t ulMode )
{
	if ( ulPin > PINS_COUNT )
    {
        return ;
    }

  GPIO_TypeDef *gpio_port = g_APinDescription[ulPin].pPort;
  uint16_t gpio_pin = g_APinDescription[ulPin].ulPin;

  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(g_APinDescription[ulPin].ulPeripheral,ENABLE);

  GPIO_InitStructure.GPIO_Pin = gpio_pin;

	switch ( ulMode )
    {
        case INPUT:
          GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        break ;

        case INPUT_PULLUP:
          GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
        break ;

        case INPUT_PULLDOWN:
          GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
        break;

        case OUTPUT:
          GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
          GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        break ;

        case AF_OUTPUT_PUSHPULL:	//Used internally for Alternate Function Output PushPull(TIM, UART, SPI etc)
          GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
          GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        break;

        case AF_OUTPUT_DRAIN:		//Used internally for Alternate Function Output Drain(I2C etc)
          GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
          GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        break;

        case AN_INPUT:				//Used internally for ADC Input
          GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
        break;

        default:
        break ;
    }

  GPIO_Init(gpio_port, &GPIO_InitStructure);
}

extern void digitalWrite( uint32_t ulPin, uint32_t ulVal )
{
  /* Handle */
	if ( ulPin > PINS_COUNT )
  {
    return ;
  }

  if ( ulVal == HIGH )
  {
    g_APinDescription[ulPin].pPort->BSRR = g_APinDescription[ulPin].ulPin;
  }
  else
  {
    g_APinDescription[ulPin].pPort->BRR = g_APinDescription[ulPin].ulPin;
  }
}

extern int digitalRead( uint32_t ulPin )
{
	if ( ulPin > PINS_COUNT )
    {
        return LOW ;
    }

	return GPIO_ReadInputDataBit(g_APinDescription[ulPin].pPort,g_APinDescription[ulPin].ulPin) ;
}

#ifdef __cplusplus
}
#endif

