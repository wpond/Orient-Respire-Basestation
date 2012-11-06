/**************************************************************************//**
 * @file
 * @brief Wav Player, requires FAT32 formatted micro-SD card with .wav file
 * @details
 *   On some DVK main boards, you need to remove the prototype board for this
 *   example to run successfully.
 * @author Energy Micro AS
 * @version 1.2.2
 * @note
 *   WARNING: Do not attach or use headphones with this example. Use small
 *   loadspeakers with built in amplification, ensuring volume is at an
 *   acceptable level. Exposure to loud noises from any source for extended
 *   periods of time may temporarily or permanently affect your hearing. The
 *   louder the volume sound level, the less time is required before your
 *   hearing could be affected. Hearing damage from loud noise is sometimes
 *   undetectable at first and can have a cumulative effect.
 *
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2011 Energy Micro AS, http://www.energymicro.com</b>
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
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <limits.h>

#include "efm32.h"
#include "efm32_common.h"
#include "efm32_cmu.h"
#include "efm32_emu.h"
#include "efm32_dac.h"
#include "efm32_prs.h"
#include "efm32_timer.h"
#include "efm32_dma.h"
#include "efm32_usart.h"
#include "efm32_gpio.h"
#include "dmactrl.h"
#include "usb.h"

/** Remove this #define if you want to use DAC0 instead of the I2S dac. */
#define USE_I2S

/** Ram buffers
 * BUFFERSIZE should be between 512 and 1024, depending on available ram on efm32
 */
//#define BUFFERSIZE               512
#define BUFFERSIZE               128

/** DMA callback structure */
static DMA_CB_TypeDef DMAcallBackTX;
static DMA_CB_TypeDef DMAcallBackRX;

/* Buffers for DMA transfer, 16 bits are transfered at a time with DMA.
 * The buffers are twice as large as BUFFERSIZE to hold both left and right
 * channel samples. */
static int8_t ramBufferDacData0Stereo[2 * BUFFERSIZE];
static int8_t ramBufferDacData1Stereo[2 * BUFFERSIZE];

static int8_t ramBufferTX0Zeros[1];
static int8_t ramBufferTX1Zeros[1];
static int8_t ramBufferRXZeros[1];
static int8_t ramBufferRXZeros[1];

/** Bytecounter, need to stop DMA when finished reading file */
static uint32_t ByteCounter;

/**************************************************************************//**
 * @brief
 *   Callback function called when the DMA finishes a transfer.
 * @param channel
 *   The DMA channel that finished.
 * @param primary
 *   Primary or Alternate DMA descriptor
 * @param user
 *   User defined pointer (Not used in this example.)
 *****************************************************************************/
void PingPongTransferCompleteTX(unsigned int channel, bool primary, void *user)
{
  (void) channel;              /* Unused parameter */
  (void) user;                 /* Unused parameter */

  /* Stop DMA if bytecounter is equal to datasize or larger */
  bool stop = false;

  /* Refresh the DMA control structure */
  DMA_RefreshPingPong(0,
                      primary,
                      false,
                      NULL,
                      NULL,
                              (2 * BUFFERSIZE) - 1,
                      stop);
}

volatile int rxPackets = 0;

/**************************************************************************//**
 * @brief
 *   Callback function called when the DMA finishes a transfer.
 * @param channel
 *   The DMA channel that finished.
 * @param primary
 *   Primary or Alternate DMA descriptor
 * @param user
 *   User defined pointer (Not used in this example.)
 *****************************************************************************/
void PingPongTransferCompleteRX(unsigned int channel, bool primary, void *user)
{
  (void) channel;              /* Unused parameter */
  (void) user;                 /* Unused parameter */

//  FillBufferFromSDcard((bool) wavHeader.channels, primary);

  /* Stop DMA if bytecounter is equal to datasize or larger */
  bool stop = false;
  int8_t * buffer;

  /* Set buffer pointer correct ram buffer */
  if (primary)
  {
    buffer = ramBufferDacData0Stereo;
  }
  else /* Alternate */
  {
    buffer = ramBufferDacData1Stereo;
  }
  USB_Transmit((uint8_t *)buffer, 256);
  rxPackets++;
  /* Refresh the DMA control structure */
  DMA_RefreshPingPong(1,
                      primary,
                      false,
                      NULL,
                      NULL,
                      (2 * BUFFERSIZE) - 1,
                      stop);
}

/**************************************************************************//**
 * @brief
 *   Setup DMA in ping pong mode
 * @details
 *   The DMA is set up to transfer data from memory to the DAC, triggered by
 *   PRS (which in turn is triggered by the TIMER). When the DMA finishes,
 *   it will trigger the callback (PingPongTransferComplete).
 *****************************************************************************/
void DMA_setup(void)
{
  /* DMA configuration structs */
  DMA_Init_TypeDef       dmaInit;
  DMA_CfgChannel_TypeDef chnlCfg;
  DMA_CfgDescr_TypeDef   descrCfg;
  ramBufferTX0Zeros[0] = 0;
  /* Initializing the DMA */
  dmaInit.hprot        = 0;
  dmaInit.controlBlock = dmaControlBlock;
  DMA_Init(&dmaInit);

  /* Set the interrupt callback routine */
  DMAcallBackTX.cbFunc = PingPongTransferCompleteTX;

  /* Callback doesn't need userpointer */
  DMAcallBackTX.userPtr = NULL;

  /* Setting up channel */
  chnlCfg.highPri   = false; /* Can't use with peripherals */
  chnlCfg.enableInt = true;  /* Interrupt needed when buffers are used */

  chnlCfg.select = DMAREQ_USART2_TXBL;

  chnlCfg.cb = &DMAcallBackTX;
  DMA_CfgChannel(0, &chnlCfg);  // Enable TX DMA Channel
  /* Set the interrupt callback routine */
  DMAcallBackRX.cbFunc = PingPongTransferCompleteRX;

  /* Callback doesn't need userpointer */
  DMAcallBackRX.userPtr = NULL;

  /* Setting up channel */
  chnlCfg.highPri   = false; /* Can't use with peripherals */
  chnlCfg.enableInt = true;  /* Interrupt needed when buffers are used */

  /* channel 0 and 1 will need data at the same time,
   * can use channel 0 as trigger */

  chnlCfg.select = DMAREQ_USART2_RXDATAV;

  chnlCfg.cb = &DMAcallBackRX;
  DMA_CfgChannel(1, &chnlCfg);
  /* Setting up channel descriptor */
  /* Destination is DAC/USART register and doesn't move */
  descrCfg.dstInc = dmaDataIncNone;

  descrCfg.srcInc = dmaDataIncNone;
  descrCfg.size   = dmaDataSize1;

  /* We have time to arbitrate again for each sample */
  descrCfg.arbRate = dmaArbitrate1;
  descrCfg.hprot   = 0;

  /* Configure both primary and secondary descriptor alike */
  DMA_CfgDescr(0, true, &descrCfg);
  DMA_CfgDescr(0, false, &descrCfg);

  /* Setting up channel descriptor */
  /* Destination is DAC/USART register and doesn't move */
  descrCfg.dstInc = dmaDataInc1;

  descrCfg.srcInc = dmaDataIncNone;
  descrCfg.size   = dmaDataSize1;

  /* We have time to arbitrate again for each sample */
  descrCfg.arbRate = dmaArbitrate1;
  descrCfg.hprot   = 0;

  /* Configure both primary and secondary descriptor alike */
  DMA_CfgDescr(1, true, &descrCfg);
  DMA_CfgDescr(1, false, &descrCfg);

  /* Enabling PingPong Transfer*/
  DMA_ActivatePingPong(0,
                       false,
                             (void *) & (USART2->TXDATA),
                       (void *) &ramBufferTX0Zeros,
                       (2 * BUFFERSIZE) - 1,
                       (void *) &(USART2->TXDATA),
                       (void *) &ramBufferTX1Zeros,
                       (2 * BUFFERSIZE) - 1);
  /* Enabling PingPong Transfer*/
  DMA_ActivatePingPong(1,
                       false,
                       (void *) &ramBufferDacData0Stereo,
                             (void *) & (USART2->RXDATA),
                       (2 * BUFFERSIZE) - 1,
                       (void *) &ramBufferDacData1Stereo,
                       (void *) &(USART2->RXDATA),
                       (2 * BUFFERSIZE) - 1);
}

#ifdef USE_I2S
/**************************************************************************//**
 * @brief
 *   Setup USART1 in I2S mode.
 * @details
 *   USART1 is initialized in I2S mode to feed the DS1334 I2S dac.
 *   Baudrate is set based on information from the WAV file header.
 *****************************************************************************/
static void I2S_Setup(void)
{
  USART_InitI2s_TypeDef init = USART_INITI2S_DEFAULT;

  CMU_ClockEnable(cmuClock_USART2, true);

  /* Use location 1: TX  - Pin D0, (RX - Pin D1) */
  /*                 CLK - Pin D2, CS - Pin D3   */

  init.format = usartI2sFormatW32D24;
  init.delay = true;
  // ADMP441_SD
  GPIO_PinModeSet(gpioPortB, 4, gpioModeInput, 0);
  // ADMP441_SCK
  GPIO_PinModeSet(gpioPortB, 5, gpioModePushPull, 0);
  // ADMP441_WS
  GPIO_PinModeSet(gpioPortB, 6, gpioModePushPull, 0);
  // ADMP441_CHIPEN
  GPIO_PinModeSet(gpioPortD,10, gpioModePushPull, 1);

  /* Configure USART for basic I2S operation */
  init.sync.baudrate = 48000 * 64;
  init.sync.enable = usartEnable;
  init.sync.databits = usartDatabits8;
  init.dmaSplit = false;
  init.mono = false;
  USART_InitI2s(USART2, &init);

  /* Enable pins at location 1 */
  USART2->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN |
                  USART_ROUTE_CSPEN |
                  USART_ROUTE_CLKPEN |
                  USART_ROUTE_LOCATION_LOC1;
}
#endif


/**************************************************************************//**
 * @brief
 *   Main function.
 * @details
 *   Configures the DVK for sound output, reads the wav header and fills the data
 *   buffers. After the DAC, DMA, Timer and PRS are set up to perform playback
 *   the mainloop just enters em1 continuously.
 *****************************************************************************/
int EnableMicrophone(void)
{
  int    bytes_read;
  volatile uint16_t data;

  ByteCounter = 0;

  /* Use 48MHZ HFXO as core clock frequency, need high speed for 44.1kHz stereo */
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);


  /* Setup SysTick Timer for 10 msec interrupts  */
//  if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 100))
//  {
//    while (1) ;
//  }


  /* Start clocks */
  CMU_ClockEnable(cmuClock_DMA, true);

  /* Fill both primary and alternate RAM-buffer before start */
  //FillBufferFromSDcard((bool) wavHeader.channels, true);
  //FillBufferFromSDcard((bool) wavHeader.channels, false);

  /* Setup DMA and peripherals */
  DMA_setup();

  /* Setup USART1 as I2S master */
  I2S_Setup();

//    //USART2->TXDATA = 0xFF;
//  while (true) {
//    USART2->TXDOUBLE = 0xFF;
//    while (!(USART2->STATUS & USART_STATUS_TXC)) ;
//    data = USART2->RXDOUBLE;
//    if(data != 0x0000) {
//      ramBufferZeros[0] = data;
//    }
//  }

}
