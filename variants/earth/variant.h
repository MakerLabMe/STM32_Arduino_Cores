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

#ifndef _VARIANT_MAKERLAB_SUN_
#define _VARIANT_MAKERLAB_SUN_

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/** Frequency of the board main oscillator */
#define VARIANT_MAINOSC		8000000

/** Master clock frequency */
#define VARIANT_MCK			72000000

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "Arduino.h"
#ifdef __cplusplus
//#include "UARTClass.h"
#include "USARTClass.h"
#endif

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

/**
 * Libc porting layers
 */
#if defined (  __GNUC__  ) /* GCC CS3 */
#    include <syscalls.h> /** RedHat Newlib minimal stub */
#endif

#define NONE ((uint8_t)0xFF)

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

// Number of pins defined in PinDescription array
#define PINS_COUNT           (30)

// LEDs
#define PIN_LED_13           (13u)
#define PIN_LED_RXL          -1//(72u)
#define PIN_LED_TXL          -1//(73u)
#define PIN_LED              PIN_LED_13
#define PIN_LED2             PIN_LED_RXL
#define PIN_LED3             PIN_LED_TXL

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 2

#define SPI_INTERFACE        SPI2
#define SPI_INTERFACE_ID     -1//ID_SPI0
#define SPI_CHANNELS_NUM -1//4
//NSS use software control
#define PIN_SPI_SS0          -1//(59u)
#define PIN_SPI_SS1          -1//(60u)
#define PIN_SPI_SS2          -1//(61u)
#define PIN_SPI_SS3          -1//(62u)
#define PIN_SPI_MOSI         (24u)
#define PIN_SPI_MISO         (22u)
#define PIN_SPI_SCK          (23u)
#define BOARD_SPI_SS0        -1//(59u)
#define BOARD_SPI_SS1        -1//(60u)
#define BOARD_SPI_SS2        -1//(61u)
#define BOARD_SPI_SS3        -1//PIN_SPI_SS3
#define BOARD_SPI_DEFAULT_SS -1//BOARD_SPI_SS3


#define SPI_INTERFACE1        SPI1
#define PIN_SPI_MOSI1         (6u)
#define PIN_SPI_MISO1         (5u)
#define PIN_SPI_SCK1          (19u)

//#define SPI_INTERFACE2        SPI3
//#define PIN_SPI_MOSI2         (4u)
//#define PIN_SPI_MISO2         (3u)
//#define PIN_SPI_SCK2          (2u)


#define BOARD_PIN_TO_SPI_PIN(x) \
	(x==BOARD_SPI_SS0 ? PIN_SPI_SS0 : \
	(x==BOARD_SPI_SS1 ? PIN_SPI_SS1 : \
	(x==BOARD_SPI_SS2 ? PIN_SPI_SS2 : PIN_SPI_SS3 )))
#define BOARD_PIN_TO_SPI_CHANNEL(x) \
	(x==BOARD_SPI_SS0 ? 0 : \
	(x==BOARD_SPI_SS1 ? 1 : \
	(x==BOARD_SPI_SS2 ? 2 : 3)))

static const uint8_t SS   = BOARD_SPI_SS0;
//static const uint8_t SS1  = BOARD_SPI_SS1;
//static const uint8_t SS2  = BOARD_SPI_SS2;
//static const uint8_t SS3  = BOARD_SPI_SS3;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

static const uint8_t MOSI1 = PIN_SPI_MOSI1;
static const uint8_t MISO1 = PIN_SPI_MISO1;
static const uint8_t SCK1  = PIN_SPI_SCK1;

//static const uint8_t MOSI2 = PIN_SPI_MOSI2;
//static const uint8_t MISO2 = PIN_SPI_MISO2;
//static const uint8_t SCK2  = PIN_SPI_SCK2;

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 2

#define PIN_WIRE_SDA         (20u)
#define PIN_WIRE_SCL         (21u)
#define WIRE_INTERFACE       I2C2
#define WIRE_INTERFACE_ID    ID_TWI1
#define WIRE_ISR_HANDLER     TWI1_Handler

#define PIN_WIRE1_SDA        (11u)
#define PIN_WIRE1_SCL        (10u)
#define WIRE1_INTERFACE      I2C1
#define WIRE1_INTERFACE_ID   ID_TWI0
#define WIRE1_ISR_HANDLER    TWI0_Handler

/*
 * UART/USART Interfaces
 */
// Serial
#define RX               (0u)  //PA10
#define TX               (1u)  //PA9
// Serial1
#define RX0              (17u) //PA3
#define TX0              (16u) //PA2
// Serial2
#define RX1              (20u) //PD6
#define TX1              (21u) //PD5
// Serial3
//#define RX2              (18u) //PD9
//#define TX2              (19u) //PD8

/*
 * USB Interfaces
 */
#define PINS_USB      D_DACC
/*
 * Analog pins
 */
static const uint8_t A0  = 14;
static const uint8_t A1  = 15;
static const uint8_t A2  = 16;
static const uint8_t A3  = 17;
static const uint8_t A4  = 18;
static const uint8_t A5  = 19;
static const uint8_t A6  = 5;
static const uint8_t A7  = 6;
static const uint8_t A8  = 7;
static const uint8_t A9  = 8;
//static const uint8_t CANRX = 68;
//static const uint8_t CANTX = 69;
#define ADC_RESOLUTION		12

#define DACC_RESOLUTION		12
#define DACC_ISR_HANDLER    DACC_Handler
#define DACC_ISR_ID         DACC_IRQn

/*
 * PWM
 */
#define PWM_INTERFACE		PWM
#define PWM_INTERFACE_ID	ID_PWM
#define PWM_FREQUENCY		1000
#define PWM_MAX_DUTY_CYCLE	255
#define PWM_MIN_DUTY_CYCLE	0
#define PWM_RESOLUTION		8

/*
 * TC
 */
#define TC_INTERFACE        TC0
#define TC_INTERFACE_ID     ID_TC0
#define TC_FREQUENCY        1000
#define TC_MAX_DUTY_CYCLE   255
#define TC_MIN_DUTY_CYCLE   0
#define TC_RESOLUTION		8

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus

//extern UARTClass Serial;
extern USARTClass Serial;
extern USARTClass Serial1;
//extern USARTClass Serial2;
//extern USARTClass Serial3;

#endif

#endif /* _VARIANT_MAKERLAB_SUN_ */

