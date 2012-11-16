/* definitions */
//#define TIME_SERVICE

/* includes */
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "efm32.h"
#include "efm32_chip.h"
#include "efm32_emu.h"
#include "efm32_gpio.h"
#include "efm32_i2c.h"
#include "efm32_usart.h"
#include "efm32_rtc.h"
#include "efm32_cmu.h"
#include "efm32_adc.h"
#include "efm32_timer.h"
#include "efm32_int.h"
#include "efm32_usb.h"

#include "fatfs/src/ff.h"
#include "microsd.h"
#include "USB.h"
#include "led.h"
#include "radio.h"

/* variables */
extern uint8_t usbMessage[32];
extern int usbMessageLen;
volatile int USB_Message = 0;

FATFS Fatfs;
FIL file;
bool fileOpened = false;

uint32_t last_respire_data = 0,
	last_pressure_data = 0;

/* prototypes */
void InitClocks();
void startupLEDs();
void wait(uint32_t ms);
int initFatFS(void);

/* functions */
int initFatFS(void)
{
  MICROSD_init(48000000);
  if (f_mount(0, &Fatfs) != FR_OK)
    return -1;
  return 0;
}
DWORD get_fattime(void)
{
  return (28 << 25) | (2 << 21) | (1 << 16);
}

void wait(uint32_t ms)
{
	
	uint32_t time, 
		clockFreq = CMU_ClockFreqGet(cmuClock_RTC);
	
	while (ms > 0)
	{
		
		time = RTC_CounterGet();
		
		if (16777215 - time < ((double)ms / 1000.0) * clockFreq)
		{
			ms -= (uint32_t)(1000.0 * ((16777215 - time) / (double)clockFreq));
			while (RTC_CounterGet() > time);
		}
		else
		{
			while (RTC_CounterGet() < time + ((double)ms / 1000.0) * clockFreq);
			break;
		}
		
	}
	
}

void startupLEDs()
{
	
	LED_Off(RED);
	LED_Off(BLUE);
	LED_Off(GREEN);
	
	wait(1000);
	
	LED_On(RED);
	LED_On(BLUE);
	LED_On(GREEN);
	
	wait(1000);
	
	LED_Off(RED);
	LED_Off(BLUE);
	LED_Off(GREEN);
	
	wait(1000);
	
}

void InitClocks()
{
	/* Starting LFXO and waiting until it is stable */
  CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
  
  // starting HFXO, wait till stable
  CMU_OscillatorEnable(cmuOsc_HFXO, true, true);
	
	// route HFXO to CPU
	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
	
  /* Routing the LFXO clock to the RTC */
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
  CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);
  
  // disabling the RCs
	CMU_ClockEnable(cmuSelect_HFRCO, false);
	CMU_ClockEnable(cmuSelect_LFRCO, false);

  /* Enabling clock to the interface of the low energy modules */
  CMU_ClockEnable(cmuClock_CORE, true);
  CMU_ClockEnable(cmuClock_CORELE, true);
  
  // enable clock to hf perfs
  CMU_ClockEnable(cmuClock_HFPER, true);
	
	// enable clock to GPIO
	CMU_ClockEnable(cmuClock_GPIO, true);
	
	// enable clock to RTC
	CMU_ClockEnable(cmuClock_RTC, true);
	RTC_Enable(true);
	
	// enable clock to DMA
	CMU_ClockEnable(cmuClock_DMA, true);
	
	// enable clock to USARTs
	CMU_ClockEnable(cmuClock_USART0, true);
	
	// enable timer0
	CMU_ClockEnable(cmuClock_TIMER0, true);
	
}

void GPIO_ODD_IRQHandler()
{
	
	GPIO->IFC = GPIO->IF;
	
}

void GPIO_EVEN_IRQHandler()
{
	
	RADIO_Interrupt();
	
	GPIO->IFC = GPIO->IF;
	
}

void TIMER0_IRQHandler()
{
	
	TIMER_IntClear(TIMER0, TIMER_IF_OF);
	
	uint32_t time = RTC_CounterGet();
	uint32_t time_diff = (time > last_respire_data) ? time - last_respire_data : (16777215 - last_respire_data) + time;
	
	if (time_diff < CMU_ClockFreqGet(cmuClock_RTC) * 3)
	{
		LED_Toggle(GREEN);
		LED_Off(RED);
	}
	else
	{
		LED_Off(GREEN);
		LED_On(RED);
	}
	
}

void LEDTIMER_Init()
{
	
	
  /* Select TIMER0 parameters */  
  TIMER_Init_TypeDef timerInit =
  {
    .enable     = true, 
    .debugRun   = true, 
    .prescale   = timerPrescale1024, 
    .clkSel     = timerClkSelHFPerClk, 
    .fallAction = timerInputActionNone, 
    .riseAction = timerInputActionNone, 
    .mode       = timerModeUp, 
    .dmaClrAct  = false,
    .quadModeX4 = false, 
    .oneShot    = false, 
    .sync       = false, 
  };
  
  /* Enable overflow interrupt */
  TIMER_IntEnable(TIMER0, TIMER_IF_OF);
  
  /* Enable TIMER0 interrupt vector in NVIC */
  NVIC_EnableIRQ(TIMER0_IRQn);

  /* Set TIMER Top value */
  TIMER_TopSet(TIMER0, SystemHFClockGet()/1024);
  
  /* Configure TIMER */
  TIMER_Init(TIMER0, &timerInit);
	
}



int main()
{
	
	// Chip errata
	CHIP_Init();
	
	// ensure core frequency has been updated
	SystemCoreClockUpdate();
	
	// start clocks
	InitClocks();
	
	// init LEDs
	LED_Init();
	
	// show startup LEDs
	startupLEDs();
	
	// enable even interrupts (radio)
	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	
	// init USB
	USB_Setup();
	
	// init radio
	RADIO_Init();
	
	uint8_t i;
	
	#ifdef TIME_SERVICE
	
		uint8_t payload[32];
		RADIO_ConfigTimeService();
		while(1)
		{
			
			if (USB_Message > 0)
			{
				
				for (i = 0; i < 32; i++)
					payload[i] = usbMessage[i];
				usbMessageLen = 0;
				USB_Message = 0;
				
				if (!RADIO_Send(payload))
					LED_On(RED);
				else
					LED_Off(RED);
				RADIO_EnableTX(true);
				LED_Toggle(GREEN);
				
			}
			
		}
	
	#endif
	
	// init timer
	LEDTIMER_Init();
	
	RADIO_ConfigRX();
	RADIO_EnableRX(true);
	
	uint8_t buffer[43];
	uint32_t time;
	
	// init to 0
	for (i = 0; i < 43; i++)
		buffer[i] = 0;
	
	// start microsd card
	if (initFatFS() == -1)
	{
		INT_Disable();
		LED_On(RED);
		LED_On(GREEN);
		LED_On(BLUE);
		while(1);
	}
	
  while (1)
  {
		
		if (RADIO_GetBufferFill() > 0)
		{
			
			// fill from 2nd byte
			RADIO_Recv(buffer+1);
			
			// get current time
			time = RTC_CounterGet();
			
			if (buffer[1] & 0x02 || buffer[1] & 0x01)
				last_respire_data = time;
			else if (buffer[1] & 0x03 || buffer[1] & 0x04)
				last_pressure_data = time;
			
			if (!fileOpened)
			{
				
				if (buffer[1] & 0x01)
				{
					
					char filename[13];
					sprintf(filename,"%8.8i.txt",10300000); // MMDDHHMM
					
					if (f_open(&file, filename, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
					{
						INT_Disable();
						LED_On(GREEN);
						LED_On(BLUE);
						LED_Off(RED);
						while(1);
					}
					
					fileOpened = true;
					
				}
				else
				{
					continue;
				}
				
			}
			
			// set first byte to signal start
			buffer[0] = '*';
			
			if (buffer[1] & 0x01)
			{
			
				// set time
				buffer[28] = (time >> 8) & 0xFF;
				buffer[29] = (time) & 0xFF;
				// clear sync stuff
				for (i = 7; i < 15; i++)
					buffer[i] = 0;
				
      }
      
      // move sequence number
      buffer[41] = ((buffer[1] & 0xF0) >> 4);
      buffer[1] &= 0x0F;
      // move dropped packet count
      buffer[42] = ((buffer[2] & 0xF0) >> 4);
      buffer[2] &= 0x0F;
			
			// transmit
			USB_Transmit(buffer,43);
			
			// store on SD
			uint32_t bytes_written;
			if (f_write(&file, buffer, 43, &bytes_written) != FR_OK) 
			{
				INT_Disable();
				LED_Off(BLUE);
				LED_On(RED);
				LED_On(GREEN);
				while(1);
			}
			
			if (f_sync(&file) != FR_OK || bytes_written < 43)
			{
				INT_Disable();
				LED_On(RED);
				LED_On(BLUE);
				LED_Off(GREEN);
				while(1);
			}
			
		}
		
  }
	
}