#
#  Copyright (c) 2014 MakerLab.me && Andy Sze(andy.sze.mail@gmail.com).  All right reserved.
#  Copyright (c) 2011 Arduino.  All right reserved.
#
#  This library is free software; you can redistribute it and/or
#  modify it under the terms of the GNU Lesser General Public
#  License as published by the Free Software Foundation; either
#  version 2.1 of the License, or (at your option) any later version.
#
#  This library is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#  See the GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public
#  License along with this library; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#

# Makefile for compiling libchip

SUBMAKE_OPTIONS=--no-builtin-rules --no-builtin-variables --no-print-directory

#-------------------------------------------------------------------------------
# Rules
#-------------------------------------------------------------------------------

# libsam_sam3s4c_gcc_rel.a libsam_sam3u4e_gcc_rel.a libsam_sam3x8e_gcc_rel.a libsam_sam3x8h_gcc_rel.a
#all: libsam_sam3s4c_gcc_dbg.a libsam_sam3u4e_gcc_dbg.a libsam_sam3x8e_gcc_dbg.a libsam_sam3x8h_gcc_dbg.a arduino_due_x
all: STM32F4Discovery libstm_stm32f4_25Mhz_gcc_dbg.a libstm_stm32f4_25Mhz_gcc_rel.a libstm_stm32f4_8Mhz_gcc_dbg.a libstm_stm32f4_8Mhz_gcc_rel.a 

.PHONY: STM32F4Discovery
STM32F4Discovery:
	@echo ------------------------------------------------------------------------------------
	@echo --- Making $@
	@echo ---
	@$(MAKE) CHIP=STM32F40_41xxx  $(SUBMAKE_OPTIONS) OUTPUT_BIN=../../../variants/F4Discovery -f stm32.mk
	@echo ------------------------------------------------------------------------------------

.PHONY: libstm_stm32f4_8Mhz_gcc_dbg.a
libstm_stm32f4_8Mhz_gcc_dbg.a:
	@echo ------------------------------------------------------------------------------------
	@echo --- Making $@
	@echo ---
	@$(MAKE) CHIP=STM32F40_41xxx   DEBUG=1 $(SUBMAKE_OPTIONS) OUTPUT_BIN=../../../variants/F4Discovery -f stm32.mk
	@echo ------------------------------------------------------------------------------------

.PHONY: libstm_stm32f4_8Mhz_gcc_rel.a
libstm_stm32f4_8Mhz_gcc_rel.a:
	@echo ------------------------------------------------------------------------------------
	@echo --- Making $@
	@echo ---
	@$(MAKE) CHIP=STM32F40_41xxx  $(SUBMAKE_OPTIONS) OUTPUT_BIN=../../../variants/F4Discovery -f stm32.mk
	@echo ------------------------------------------------------------------------------------

.PHONY: libstm_stm32f4_25Mhz_gcc_dbg.a
libstm_stm32f4_25Mhz_gcc_dbg.a:
	@echo ------------------------------------------------------------------------------------
	@echo --- Making $@
	@echo ---
	@$(MAKE) CHIP=STM32F40_41xxx DEBUG=1 $(SUBMAKE_OPTIONS) OUTPUT_BIN=../../../variants/F4Discovery -f stm32.mk
	@echo ------------------------------------------------------------------------------------

.PHONY: libstm_stm32f4_25Mhz_gcc_rel.a
libstm_stm32f4_25Mhz_gcc_rel.a:
	@echo ------------------------------------------------------------------------------------
	@echo --- Making $@
	@echo ---
	@$(MAKE) CHIP=STM32F40_41xxx $(SUBMAKE_OPTIONS) OUTPUT_BIN=../../../variants/F4Discovery -f stm32.mk
	@echo ------------------------------------------------------------------------------------

.PHONY: clean
clean:
	@echo ------------------------------------------------------------------------------------
	@echo --- Cleaning STM32F4 release and debug
	@$(MAKE) CHIP=STM32F40_41xxx  $(SUBMAKE_OPTIONS) -f stm32.mk $@
	@$(MAKE) CHIP=STM32F40_41xxx  DEBUG=1 $(SUBMAKE_OPTIONS) -f stm32.mk $@
	@echo ------------------------------------------------------------------------------------


