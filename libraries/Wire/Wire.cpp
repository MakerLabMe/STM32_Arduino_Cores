/*
 * TwoWire.h - TWI/I2C library for MakerLabBoard(Arduino compatible)
 * Copyright (c) 2014 MakerLab.me & Andy Sze(andy.sze.mail@gmail.com) 
 * ported to stm32
 * Copyright (c) 2011 Cristian Maglie <c.maglie@bug.st>.
 * All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

extern "C" {
#include <string.h>
}

#include "Wire.h"

static inline bool TWI_FailedAcknowledge(I2C_TypeDef *pTwi) {
#if 0
	return pTwi->TWI_SR & TWI_SR_NACK;
#endif
}

static inline bool TWI_WaitTransferComplete(I2C_TypeDef *_twi, uint32_t _timeout) {
#if 0
	while (!TWI_TransferComplete(_twi)) {
		if (TWI_FailedAcknowledge(_twi))
			return false;
		if (--_timeout == 0)
			return false;
	}
#endif
	return true;
}

static inline bool TWI_WaitByteSent(I2C_TypeDef *_twi, uint32_t _timeout) {
#if 0
	while (!TWI_ByteSent(_twi)) {
		if (TWI_FailedAcknowledge(_twi))
			return false;
		if (--_timeout == 0)
			return false;
	}
#endif
	return true;
}

static inline bool TWI_WaitByteReceived(I2C_TypeDef *_twi, uint32_t _timeout) {
#if 0
	while (!TWI_ByteReceived(_twi)) {
		if (TWI_FailedAcknowledge(_twi))
			return false;
		if (--_timeout == 0)
			return false;
	}
#endif
	return true;
}

static inline bool TWI_STATUS_SVREAD(uint32_t status) {
#if 0
	return (status & TWI_SR_SVREAD) == TWI_SR_SVREAD;
#endif
}

static inline bool TWI_STATUS_SVACC(uint32_t status) {
#if 0
	return (status & TWI_SR_SVACC) == TWI_SR_SVACC;
#endif
}

static inline bool TWI_STATUS_GACC(uint32_t status) {
#if 0
	return (status & TWI_SR_GACC) == TWI_SR_GACC;
#endif
}

static inline bool TWI_STATUS_EOSACC(uint32_t status) {
#if 0
	return (status & TWI_SR_EOSACC) == TWI_SR_EOSACC;
#endif
}

static inline bool TWI_STATUS_NACK(uint32_t status) {
#if 0
	return (status & TWI_SR_NACK) == TWI_SR_NACK;
#endif
}

TwoWire::TwoWire(I2C_TypeDef *_twi, void(*_beginCb)(void)) :
	twi(_twi), rxBufferIndex(0), rxBufferLength(0), txAddress(0),
			txBufferLength(0), srvBufferIndex(0), srvBufferLength(0), status(
					UNINITIALIZED), onBeginCallback(_beginCb) {
	// Empty
}

void TwoWire::begin(void) {
	if (onBeginCallback)
		onBeginCallback();

  if(twi==I2C1)
  {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    pinMode(PIN_WIRE_SCL, AF_OUTPUT_DRAIN);
    pinMode(PIN_WIRE_SDA, AF_OUTPUT_DRAIN);
  }else
  {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

    pinMode(PIN_WIRE1_SCL, AF_OUTPUT_DRAIN);
    pinMode(PIN_WIRE1_SDA, AF_OUTPUT_DRAIN);
  }


	I2C_DeInit(twi);

	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	//	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 100000;
	I2C_Init(twi, &I2C_InitStructure);

	//	I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF, ENABLE);
	//
	//	NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
	//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 12;
	//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//	NVIC_Init(&NVIC_InitStructure);

	I2C_Cmd(twi, ENABLE);

	status = MASTER_IDLE;
#if 0

	// Disable PDC channel
	twi->TWI_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;

	TWI_ConfigureMaster(twi, TWI_CLOCK, VARIANT_MCK);
	status = MASTER_IDLE;
#endif
}

void TwoWire::begin(uint8_t address) {
	if (onBeginCallback)
		onBeginCallback();
	I2C_InitStructure.I2C_OwnAddress1 = address;
  begin();
#if 0

	// Disable PDC channel
	twi->TWI_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;

	TWI_ConfigureSlave(twi, address);
	status = SLAVE_IDLE;
	TWI_EnableIt(twi, TWI_IER_SVACC);
	//| TWI_IER_RXRDY | TWI_IER_TXRDY	| TWI_IER_TXCOMP);
#endif
}

void TwoWire::begin(int address) {
	begin((uint8_t) address);
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop) {
	if (quantity > BUFFER_LENGTH)
		quantity = BUFFER_LENGTH;

  uint32_t _millis;

  //START
  I2C_GenerateSTART(twi,ENABLE);

  _millis = millis();
	while(!I2C_CheckEvent(twi, I2C_EVENT_MASTER_MODE_SELECT))
	{
		if(EVENT_TIMEOUT < (millis() - _millis)) return 0;
	}

	/* Send Slave address for read */
	I2C_Send7bitAddress(twi, address, I2C_Direction_Receiver);

	_millis = millis();
	while(!I2C_CheckEvent(twi, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
	{
		if(RECV_TIMEOUT< (millis() - _millis)) return 0;
	}

	/* perform blocking read into buffer */
	uint8_t *pBuffer = rxBuffer;
	uint8_t numByteToRead = quantity;
	uint8_t bytesRead = 0;

	/* While there is data to be read */
	while(numByteToRead)
	{
		if(numByteToRead == 1 && sendStop == true)
		{
			/* Disable Acknowledgement */
			I2C_AcknowledgeConfig(twi, DISABLE);

			/* Send STOP Condition */
			I2C_GenerateSTOP(twi, ENABLE);
		}

		if(I2C_CheckEvent(twi, I2C_EVENT_MASTER_BYTE_RECEIVED))
		{
			/* Read a byte from the Slave */
			*pBuffer = I2C_ReceiveData(twi);

			bytesRead++;

			/* Point to the next location where the byte read will be saved */
			pBuffer++;

			/* Decrement the read bytes counter */
			numByteToRead--;
		}
	}

	/* Enable Acknowledgement to be ready for another reception */
	I2C_AcknowledgeConfig(twi, ENABLE);

	// set rx buffer iterator vars
	rxBufferIndex = 0;
	rxBufferLength = bytesRead;

	return bytesRead;
#if 0
	// perform blocking read into buffer
	int readed = 0;
	TWI_StartRead(twi, address, 0, 0);
	do {
		// Stop condition must be set during the reception of last byte
		if (readed + 1 == quantity)
			TWI_SendSTOPCondition( twi);

		TWI_WaitByteReceived(twi, RECV_TIMEOUT);
		rxBuffer[readed++] = TWI_ReadByte(twi);
	} while (readed < quantity);
	TWI_WaitTransferComplete(twi, RECV_TIMEOUT);

	// set rx buffer iterator vars
	rxBufferIndex = 0;
	rxBufferLength = readed;

	return readed;
#endif
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity) {
	return requestFrom((uint8_t) address, (uint8_t) quantity, (uint8_t) true);
}

uint8_t TwoWire::requestFrom(int address, int quantity) {
	return requestFrom((uint8_t) address, (uint8_t) quantity, (uint8_t) true);
}

uint8_t TwoWire::requestFrom(int address, int quantity, int sendStop) {
	return requestFrom((uint8_t) address, (uint8_t) quantity, (uint8_t) sendStop);
}

void TwoWire::beginTransmission(uint8_t address) {
	status = MASTER_SEND;

	// save address of target and empty buffer
	txAddress = address;
	txBufferLength = 0;
}

void TwoWire::beginTransmission(int address) {
	beginTransmission((uint8_t) address);
}

//
//	Originally, 'endTransmission' was an f(void) function.
//	It has been modified to take one parameter indicating
//	whether or not a STOP should be performed on the bus.
//	Calling endTransmission(false) allows a sketch to
//	perform a repeated start.
//
//	WARNING: Nothing in the library keeps track of whether
//	the bus tenure has been properly ended with a STOP. It
//	is very possible to leave the bus in a hung state if
//	no call to endTransmission(true) is made. Some I2C
//	devices will behave oddly if they do not see a STOP.
//
uint8_t TwoWire::endTransmission(uint8_t sendStop) {
	// transmit buffer (blocking)
	uint32_t _millis;

	/* Send START condition */
	I2C_GenerateSTART(twi, ENABLE);

	_millis = millis();
	while(!I2C_CheckEvent(twi, I2C_EVENT_MASTER_MODE_SELECT))
	{
		if(XMIT_TIMEOUT < (millis() - _millis)) return 4;
	}

	/* Send Slave address for write */
	I2C_Send7bitAddress(twi, txAddress, I2C_Direction_Transmitter);

	_millis = millis();
	while(!I2C_CheckEvent(twi, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		if(XMIT_TIMEOUT < (millis() - _millis)) return 4;
	}

	uint8_t *pBuffer = txBuffer;
	uint8_t NumByteToWrite = txBufferLength;

	/* While there is data to be written */
	while(NumByteToWrite--)
	{
		/* Send the current byte to slave */
		I2C_SendData(twi, *pBuffer);

		/* Point to the next byte to be written */
		pBuffer++;

		_millis = millis();
		while (!I2C_CheckEvent(twi, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		{
			if(XMIT_TIMEOUT < (millis() - _millis)) return 4;
		}
	}

	if(sendStop == true)
	{
		/* Send STOP condition */
		I2C_GenerateSTOP(twi, ENABLE);
	}

	// reset tx buffer iterator vars
	txBufferIndex = 0;
	txBufferLength = 0;

	status = MASTER_IDLE;

	return 0;
#if 0
	TWI_StartWrite(twi, txAddress, 0, 0, txBuffer[0]);
	TWI_WaitByteSent(twi, XMIT_TIMEOUT);
	int sent = 1;
	while (sent < txBufferLength) {
		TWI_WriteByte(twi, txBuffer[sent++]);
		TWI_WaitByteSent(twi, XMIT_TIMEOUT);
	}
	TWI_Stop( twi);
	TWI_WaitTransferComplete(twi, XMIT_TIMEOUT);

	// empty buffer
	txBufferLength = 0;

	status = MASTER_IDLE;
	return sent;
#endif
}

//	This provides backwards compatibility with the original
//	definition, and expected behaviour, of endTransmission
//
uint8_t TwoWire::endTransmission(void)
{
	return endTransmission(true);
}

size_t TwoWire::write(uint8_t data) {
	if (status == MASTER_SEND) {
		if (txBufferLength >= BUFFER_LENGTH)
			return 0;
		txBuffer[txBufferLength++] = data;
		return 1;
	} else {
		if (srvBufferLength >= BUFFER_LENGTH)
			return 0;
		srvBuffer[srvBufferLength++] = data;
		return 1;
	}
}

size_t TwoWire::write(const uint8_t *data, size_t quantity) {
	if (status == MASTER_SEND) {
		for (size_t i = 0; i < quantity; ++i) {
			if (txBufferLength >= BUFFER_LENGTH)
				return i;
			txBuffer[txBufferLength++] = data[i];
		}
	} else {
		for (size_t i = 0; i < quantity; ++i) {
			if (srvBufferLength >= BUFFER_LENGTH)
				return i;
			srvBuffer[srvBufferLength++] = data[i];
		}
	}
	return quantity;
}

int TwoWire::available(void) {
	return rxBufferLength - rxBufferIndex;
}

int TwoWire::read(void) {
	if (rxBufferIndex < rxBufferLength)
		return rxBuffer[rxBufferIndex++];
	return -1;
}

int TwoWire::peek(void) {
	if (rxBufferIndex < rxBufferLength)
		return rxBuffer[rxBufferIndex];
	return -1;
}

void TwoWire::flush(void) {
	// Do nothing, use endTransmission(..) to force
	// data transfer.
}

void TwoWire::onReceive(void(*function)(int)) {
	onReceiveCallback = function;
}

void TwoWire::onRequest(void(*function)(void)) {
	onRequestCallback = function;
}

void TwoWire::onService(void) {
#if 0
	// Retrieve interrupt status
	uint32_t sr = TWI_GetStatus(twi);

	if (status == SLAVE_IDLE && TWI_STATUS_SVACC(sr)) {
		TWI_DisableIt(twi, TWI_IDR_SVACC);
		TWI_EnableIt(twi, TWI_IER_RXRDY | TWI_IER_GACC | TWI_IER_NACK
				| TWI_IER_EOSACC | TWI_IER_SCL_WS | TWI_IER_TXCOMP);

		srvBufferLength = 0;
		srvBufferIndex = 0;

		// Detect if we should go into RECV or SEND status
		// SVREAD==1 means *master* reading -> SLAVE_SEND
		if (!TWI_STATUS_SVREAD(sr)) {
			status = SLAVE_RECV;
		} else {
			status = SLAVE_SEND;

			// Alert calling program to generate a response ASAP
			if (onRequestCallback)
				onRequestCallback();
			else
				// create a default 1-byte response
				write((uint8_t) 0);
		}
	}

	if (status != SLAVE_IDLE) {
		if (TWI_STATUS_TXCOMP(sr) && TWI_STATUS_EOSACC(sr)) {
			if (status == SLAVE_RECV && onReceiveCallback) {
				// Copy data into rxBuffer
				// (allows to receive another packet while the
				// user program reads actual data)
				for (uint8_t i = 0; i < srvBufferLength; ++i)
					rxBuffer[i] = srvBuffer[i];
				rxBufferIndex = 0;
				rxBufferLength = srvBufferLength;

				// Alert calling program
				onReceiveCallback( rxBufferLength);
			}

			// Transfer completed
			TWI_EnableIt(twi, TWI_SR_SVACC);
			TWI_DisableIt(twi, TWI_IDR_RXRDY | TWI_IDR_GACC | TWI_IDR_NACK
					| TWI_IDR_EOSACC | TWI_IDR_SCL_WS | TWI_IER_TXCOMP);
			status = SLAVE_IDLE;
		}
	}

	if (status == SLAVE_RECV) {
		if (TWI_STATUS_RXRDY(sr)) {
			if (srvBufferLength < BUFFER_LENGTH)
				srvBuffer[srvBufferLength++] = TWI_ReadByte(twi);
		}
	}

	if (status == SLAVE_SEND) {
		if (TWI_STATUS_TXRDY(sr) && !TWI_STATUS_NACK(sr)) {
			uint8_t c = 'x';
			if (srvBufferIndex < srvBufferLength)
				c = srvBuffer[srvBufferIndex++];
			TWI_WriteByte(twi, c);
		}
	}
#endif
}

#if WIRE_INTERFACES_COUNT > 0
static void Wire_Init(void) {
#if 0
	pmc_enable_periph_clk(WIRE_INTERFACE_ID);
	PIO_Configure(
			g_APinDescription[PIN_WIRE_SDA].pPort,
			g_APinDescription[PIN_WIRE_SDA].ulPinType,
			g_APinDescription[PIN_WIRE_SDA].ulPin,
			g_APinDescription[PIN_WIRE_SDA].ulPinConfiguration);
	PIO_Configure(
			g_APinDescription[PIN_WIRE_SCL].pPort,
			g_APinDescription[PIN_WIRE_SCL].ulPinType,
			g_APinDescription[PIN_WIRE_SCL].ulPin,
			g_APinDescription[PIN_WIRE_SCL].ulPinConfiguration);

	NVIC_DisableIRQ(TWI1_IRQn);
	NVIC_ClearPendingIRQ(TWI1_IRQn);
	NVIC_SetPriority(TWI1_IRQn, 0);
	NVIC_EnableIRQ(TWI1_IRQn);
#endif
}

TwoWire Wire = TwoWire(WIRE_INTERFACE, Wire_Init);

void WIRE_ISR_HANDLER(void) {
	Wire.onService();
}
#endif

#if WIRE_INTERFACES_COUNT > 1
static void Wire1_Init(void) {
#if 0
	pmc_enable_periph_clk(WIRE1_INTERFACE_ID);
	PIO_Configure(
			g_APinDescription[PIN_WIRE1_SDA].pPort,
			g_APinDescription[PIN_WIRE1_SDA].ulPinType,
			g_APinDescription[PIN_WIRE1_SDA].ulPin,
			g_APinDescription[PIN_WIRE1_SDA].ulPinConfiguration);
	PIO_Configure(
			g_APinDescription[PIN_WIRE1_SCL].pPort,
			g_APinDescription[PIN_WIRE1_SCL].ulPinType,
			g_APinDescription[PIN_WIRE1_SCL].ulPin,
			g_APinDescription[PIN_WIRE1_SCL].ulPinConfiguration);

	NVIC_DisableIRQ(TWI0_IRQn);
	NVIC_ClearPendingIRQ(TWI0_IRQn);
	NVIC_SetPriority(TWI0_IRQn, 0);
	NVIC_EnableIRQ(TWI0_IRQn);
#endif
}

TwoWire Wire1 = TwoWire(WIRE1_INTERFACE, Wire1_Init);

void WIRE1_ISR_HANDLER(void) {
	Wire1.onService();
}
#endif
