/**************************************************************************//**
 * @file
 * @brief Micro SD card driver for EFM32_G2xx_DK and EFM32_G8xx_DK
 * @author Energy Micro AS
 * @version 1.7.3
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2010 Energy Micro AS, http://www.energymicro.com</b>
 ******************************************************************************
 *
 * This source code is the property of Energy Micro AS. The source and compiled
 * code may only be used on Energy Micro "EFM32" microcontrollers.
 *
 * This copyright notice may not be removed from the source code nor changed.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Energy Micro AS has no
 * obligation to support this Software. Energy Micro AS is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Energy Micro AS will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 *****************************************************************************/
#ifndef __MICROSD_H
#define __MICROSD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "efm32.h"
#include "efm32_gpio.h"
#include "integer.h"

/** USART used for communicating with the MicroSD card */
#define MICROSD_USART            USART1 //changed to USART1: USART0

/** Clock speeds to use when communicating with the MicroSD card. */
#define LOW_SPEED_CLOCK          100000
#define HIGH_SPEED_CLOCK         12000000

/* Definitions for MMC/SDC command */
#define CMD0   (0)			/* GO_IDLE_STATE */
#define CMD1   (1)			/* SEND_OP_COND */
#define ACMD41 (41|0x80)	        /* SEND_OP_COND (SDC) */
#define CMD8   (8)			/* SEND_IF_COND */
#define CMD9   (9)			/* SEND_CSD */
#define CMD10  (10)			/* SEND_CID */
#define CMD12  (12)			/* STOP_TRANSMISSION */
#define ACMD13 (13|0x80)	        /* SD_STATUS (SDC) */
#define CMD16  (16)			/* SET_BLOCKLEN */
#define CMD17  (17)			/* READ_SINGLE_BLOCK */
#define CMD18  (18)			/* READ_MULTIPLE_BLOCK */
#define CMD23  (23)			/* SET_BLOCK_COUNT */
#define ACMD23 (23|0x80)	        /* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24  (24)			/* WRITE_BLOCK */
#define CMD25  (25)			/* WRITE_MULTIPLE_BLOCK */
#define CMD41  (41)			/* SEND_OP_COND (ACMD) */
#define CMD55  (55)			/* APP_CMD */
#define CMD58  (58)			/* READ_OCR */

void MICROSD_init(uint32_t refFreq);

void FCLK_FAST(void);
void FCLK_SLOW(void);

uint8_t xfer_spi(uint8_t data);
uint8_t wait_ready(void);
void deselect(void);
int select(void);
void power_on(void);
void power_off(void);
int rcvr_datablock(BYTE *buff, UINT btr);
int xmit_datablock(const BYTE *buff, BYTE token);
BYTE send_cmd(BYTE cmd,	DWORD arg);

#ifdef __cplusplus
}
#endif

#endif
