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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "USARTClass.h"
#include "Arduino.h"

// Constructors ////////////////////////////////////////////////////////////////

USARTClass::USARTClass( USART_TypeDef* pUsart, IRQn_Type dwIrq, uint32_t dwId, RingBuffer* pRx_buffer )
{
  _rx_buffer = pRx_buffer ;

  _pUsart=pUsart ;
  _dwIrq=dwIrq ;
  _dwId=dwId ;
}

// Public Methods //////////////////////////////////////////////////////////////

void USARTClass::begin( const uint32_t dwBaudRate )
{

  //Serial
  if(_dwId == id_usart1)
  {
    // AFIO clock enable
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

    // Enable USART Clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    // Configure USART Rx as input floating
    pinMode(RX, INPUT);

    // Configure USART Tx as alternate function push-pull
    pinMode(TX, AF_OUTPUT_PUSHPULL);
  }
  else if(_dwId == id_usart4)//Serial1
  {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    pinMode(RX0, INPUT);
    pinMode(TX0, AF_OUTPUT_PUSHPULL);
  }
  else if(_dwId == id_usart2)//Serial2
  {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    pinMode(RX1, INPUT);
    pinMode(TX1, AF_OUTPUT_PUSHPULL);
  }
  else if(_dwId == id_usart3)//Serial3
  {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    pinMode(RX2, INPUT);
    pinMode(TX2, AF_OUTPUT_PUSHPULL);
  }
	// USART default configuration
	// USART configured as follow:
	// - BaudRate = (set baudRate as 9600 baud)
	// - Word Length = 8 Bits
	// - One Stop Bit
	// - No parity
	// - Hardware flow control disabled (RTS and CTS signals)
	// - Receive and transmit enabled
	USART_InitStructure.USART_BaudRate = dwBaudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	// Configure USART
	USART_Init(_pUsart, &USART_InitStructure);

  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */  
  //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  
  /* Enable the USARTy Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = _dwIrq;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = _dwId;		
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable USART Receive interrupts */
  USART_ITConfig(_pUsart, USART_IT_RXNE, ENABLE);
  // Enable UART interrupt in NVIC
  NVIC_EnableIRQ( _dwIrq ) ;

	// Enable the USART
	USART_Cmd(_pUsart, ENABLE);
}

void USARTClass::end( void )
{
  // clear any received data
  _rx_buffer->_iHead = _rx_buffer->_iTail ;

  // Disable UART interrupt in NVIC
  NVIC_DisableIRQ( _dwIrq ) ;

  // Wait for any outstanding data to be sent
  flush();
  
  USART_Cmd(_pUsart, DISABLE);

  if(_dwId == id_usart1)
  {
    // Disable USART Clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, DISABLE);
  }
  else if(_dwId == id_usart2)
  {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, DISABLE);
  }
  else if(_dwId == id_usart3)
  {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, DISABLE);
  }
  else if(_dwId == id_usart4)
  {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, DISABLE);
  }
}

int USARTClass::available( void )
{
  return (uint32_t)(SERIAL_BUFFER_SIZE + _rx_buffer->_iHead - _rx_buffer->_iTail) % SERIAL_BUFFER_SIZE ;
}

int USARTClass::peek( void )
{
  if ( _rx_buffer->_iHead == _rx_buffer->_iTail )
    return -1 ;

  return _rx_buffer->_aucBuffer[_rx_buffer->_iTail] ;
}

int USARTClass::read( void )
{
  // if the head isn't ahead of the tail, we don't have any characters
  if ( _rx_buffer->_iHead == _rx_buffer->_iTail )
    return -1 ;

  uint8_t uc = _rx_buffer->_aucBuffer[_rx_buffer->_iTail] ;
  _rx_buffer->_iTail = (unsigned int)(_rx_buffer->_iTail + 1) % SERIAL_BUFFER_SIZE ;
  return uc ;
}

void USARTClass::flush( void )
{
#if 0
  // Wait for transmission to complete
  while ((_pUsart->US_CSR & US_CSR_TXRDY) != US_CSR_TXRDY)
	;
#endif
}

size_t USARTClass::write( const uint8_t uc_data )
{
	// Send one byte from USART
	USART_SendData(_pUsart, uc_data);

	// Loop until USART DR register is empty
	while(USART_GetFlagStatus(_pUsart, USART_FLAG_TXE) == RESET)
	{
	}

	return 1;
}

void USARTClass::IrqHandler( void )
{
  // Did we receive data ?
  if(USART_GetITStatus(_pUsart, USART_IT_RXNE) != RESET)
  {
    /* Read one byte from the receive data register */
    uint8_t RxBuffer;
    RxBuffer = USART_ReceiveData(_pUsart);
    _rx_buffer->store_char( RxBuffer ) ;
  }

#if 0
  // Acknowledge errors
  if ((status & US_CSR_OVRE) == US_CSR_OVRE ||
		  (status & US_CSR_FRAME) == US_CSR_FRAME)
  {
	// TODO: error reporting outside ISR
    _pUsart->US_CR |= US_CR_RSTSTA;
  }
#endif

}

