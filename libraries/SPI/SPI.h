/*
 * Copyright (c) 2014 MakerLab.me & Andy Sze(andy.sze.mail@gmail.com) 
 * ported to stm32
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#ifndef _SPI_H_INCLUDED
#define _SPI_H_INCLUDED

#include "variant.h"
#include <stdio.h>

#define SPI_MODE0 0x02
#define SPI_MODE1 0x00
#define SPI_MODE2 0x03
#define SPI_MODE3 0x01

#define SPI_CLOCK_DIV2     SPI_BaudRatePrescaler_2  
#define SPI_CLOCK_DIV4     SPI_BaudRatePrescaler_4  
#define SPI_CLOCK_DIV8     SPI_BaudRatePrescaler_8  
#define SPI_CLOCK_DIV16    SPI_BaudRatePrescaler_16 
#define SPI_CLOCK_DIV32    SPI_BaudRatePrescaler_32 
#define SPI_CLOCK_DIV64    SPI_BaudRatePrescaler_64 
#define SPI_CLOCK_DIV128   SPI_BaudRatePrescaler_128
#define SPI_CLOCK_DIV256   SPI_BaudRatePrescaler_256

class SPIClass {
  public:
	SPIClass(SPI_TypeDef *_spi, void(*_initCb)(void));

	//byte transfer(uint8_t _data, SPITransferMode _mode = SPI_LAST) { return transfer(BOARD_SPI_DEFAULT_SS, _data, _mode); }
	//byte transfer(byte _channel, uint8_t _data, SPITransferMode _mode = SPI_LAST);
  byte transfer(uint8_t _data);

	// SPI Configuration methods

	void attachInterrupt(void);
	void detachInterrupt(void);

	void begin(void);
	void end(void);

	// Attach/Detach pin to/from SPI controller
	//void begin(uint8_t _pin);
	//void end(uint8_t _pin);

	// These methods sets a parameter on a single pin
	void setBitOrder( uint16_t );
	void setDataMode( uint8_t);
	void setClockDivider( uint8_t);

  private:
	void init();

	SPI_TypeDef *spi;
	//uint32_t id;
	uint16_t bitOrder;
	uint32_t divider;
	uint32_t mode;
	void (*initCb)(void);
	bool initialized;
  SPI_InitTypeDef SPI_InitStructure;
};

#if SPI_INTERFACES_COUNT > 0
extern SPIClass SPI;
#if SPI_INTERFACES_COUNT > 1
extern SPIClass SPI_1;
#if SPI_INTERFACES_COUNT > 2
extern SPIClass SPI_2;
#endif
#endif
#endif

#endif
