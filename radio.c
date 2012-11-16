#include "radio.h"

#include "nRF24L01.h"

#include "efm32_usart.h"
#include "efm32_gpio.h"

#include <stdint.h>
#include <string.h>

#include "led.h"

/* prototypes */
uint8_t readRegister(uint8_t reg);
void writeRegister(uint8_t reg, uint8_t value);

void readBytes(uint8_t reg, uint8_t *payload, uint8_t len);
void writeBytes(uint8_t reg, uint8_t *payload, uint8_t len);

void readPayload(uint8_t *payload, uint8_t len);
void writePayload(uint8_t *payload, uint8_t len);

void readRegisterMulti(uint8_t reg, uint8_t *payload, uint8_t len);
void writeRegisterMulti(uint8_t reg, uint8_t *payload, uint8_t len);


void sendCommand(uint8_t cmd, uint8_t data);
void sendPayload(uint8_t reg, uint8_t bytes, uint8_t *data);

/* variables */
volatile uint8_t bufferCount = 0;

/* functions */
void RADIO_Init()
{
	
	// usart 0 location 2
	
	// enable pins
	GPIO_PinModeSet(NRF_CE_PORT, NRF_CE_PIN, gpioModePushPull, 0);
	GPIO_PinModeSet(NRF_CSN_PORT, NRF_CSN_PIN, gpioModePushPull, 1);
	GPIO_PinModeSet(NRF_RXEN_PORT, NRF_RXEN_PIN, gpioModePushPull, 0);
	GPIO_PinModeSet(NRF_INT_PORT, NRF_INT_PIN, gpioModeInput, 0);
	
	GPIO_PinModeSet(gpioPortC, 11, gpioModePushPull, 1);
	GPIO_PinModeSet(gpioPortC, 10, gpioModeInput, 0);
	GPIO_PinModeSet(gpioPortC, 9, gpioModePushPull, 0);
	
	USART_InitSync_TypeDef usartInit = USART_INITSYNC_DEFAULT;
	
	usartInit.msbf = true;
	usartInit.clockMode = usartClockMode0;
	usartInit.baudrate = 1000000;
	USART_InitSync(NRF_USART, &usartInit);
	NRF_USART->ROUTE = (NRF_USART->ROUTE & ~_USART_ROUTE_LOCATION_MASK) | USART_ROUTE_LOCATION_LOC2;
	NRF_USART->ROUTE |= USART_ROUTE_TXPEN | USART_ROUTE_RXPEN | USART_ROUTE_CLKPEN;
	
	// configure gpio interrupt
	GPIO_IntClear(1 << NRF_INT_PIN);
	writeRegister(NRF_STATUS,0x70);
	GPIO_IntConfig(NRF_INT_PORT,NRF_INT_PIN,false,true,true);
	
	NRF_CE_lo;
	
	// initial config
	writeRegister(NRF_EN_AA,0x3F);
	writeRegister(NRF_EN_RXADDR,0x3F);
	writeRegister(NRF_SETUP_AW,0x03);
	writeRegister(NRF_SETUP_RETR,0x0E);
	writeRegister(NRF_RF_CH,NODE_CH);
	writeRegister(NRF_RF_SETUP,0x0F);
	
	writeRegister(NRF_RX_PW_P0, 32);
	writeRegister(NRF_RX_PW_P1, 32);
	writeRegister(NRF_RX_PW_P2, 32);
	writeRegister(NRF_RX_PW_P3, 32);
	writeRegister(NRF_RX_PW_P4, 32);
	writeRegister(NRF_RX_PW_P5, 32);
	
	uint8_t addr_array[5];
	
	addr_array[0] = 0xE7;
  addr_array[1] = 0xE7;
  addr_array[2] = 0xE7;
  addr_array[3] = 0xE7;
  addr_array[4] = 0xE7;
  writeRegisterMulti(NRF_TX_ADDR, addr_array, 5);
  writeRegisterMulti(NRF_RX_ADDR_P0, addr_array, 5);
  
  addr_array[0] = 0xC2;
  addr_array[1] = 0xC2;
  addr_array[2] = 0xC2;
  addr_array[3] = 0xC2;
  addr_array[4] = 0xC2;
  writeRegisterMulti(NRF_RX_ADDR_P1, addr_array, 5);
  writeRegister(NRF_RX_ADDR_P2, 0xC3);
  writeRegister(NRF_RX_ADDR_P3, 0xC4);
  writeRegister(NRF_RX_ADDR_P4, 0xC5);
  writeRegister(NRF_RX_ADDR_P5, 0xC6);
	
	writeRegister(NRF_DYNPD, 0x00);
	writeRegister(NRF_FEATURE, 0x00);
	
}

void RADIO_ConfigTimeService()
{
	
	writeRegister(NRF_EN_AA,0x00);
	writeRegister(NRF_SETUP_RETR,0x00);
	writeRegister(NRF_RF_CH,0);
	RADIO_ConfigTX();
	
}

void RADIO_ConfigTX()
{

	writeRegister(NRF_CONFIG,0x4E);

	sendCommand(NRF_FLUSH_TX, NRF_NOP);
	sendCommand(NRF_FLUSH_RX, NRF_NOP);
	writeRegister(NRF_STATUS,0x70);
	
	bufferCount = 0;
	
}

void RADIO_ConfigRX()
{
	
	writeRegister(NRF_CONFIG,0x3F);

	sendCommand(NRF_FLUSH_TX, NRF_NOP);
	sendCommand(NRF_FLUSH_RX, NRF_NOP);
	writeRegister(NRF_STATUS,0x70);
	
	bufferCount = 0;
	
}

void RADIO_EnableRX(bool enable)
{
	
	if (enable)
	{
		NRF_RXEN_hi;
		NRF_CE_hi;
	}
	else
	{
		NRF_RXEN_lo;
		NRF_CE_lo;
	}
	
}

void RADIO_EnableTX(bool enable)
{
	
	if (enable)
	{
		NRF_CE_hi;
	}
	else
	{
		NRF_CE_lo;
	}
	
}

bool RADIO_Send(uint8_t *payload)
{
	
	uint8_t fifo_status = readRegister(NRF_FIFO_STATUS);
	
	if (fifo_status & 0x20)
	{
		return false;
	}
	
	bufferCount++;
	
	sendPayload(NRF_W_TX_PAYLOAD,32,payload);
	
	return true;
	
}

bool RADIO_Recv(uint8_t *payload)
{
	
	uint8_t fifo_status = readRegister(NRF_FIFO_STATUS);
	
	if (fifo_status & 0x01)
	{
		return false;
	}
	
	bufferCount--;
	
	receivePayload(NRF_R_RX_PAYLOAD, payload, 32);
	return true;
	
}

uint8_t readByte(uint8_t cmd)
{
  volatile uint8_t blah;
  NRF_CSN_lo;
  while((NRF_USART->STATUS & USART_STATUS_RXDATAV)) {
    blah = NRF_USART->RXDATA;
  }
  while(!(NRF_USART->STATUS & USART_STATUS_TXBL));
  NRF_USART->TXDATA = cmd;
  while(!(NRF_USART->STATUS & USART_STATUS_TXC));
  USART_Rx(NRF_USART);
  NRF_USART->TXDATA = 0x00;
  while (!(NRF_USART->STATUS & USART_STATUS_TXC)) ;
  NRF_CSN_hi;
  return USART_Rx(NRF_USART);
}

uint8_t readRegister(uint8_t reg)
{
	return readByte(reg | NRF_R_REGISTER);
}

void writeRegister(uint8_t reg, uint8_t data)
{
  sendCommand((reg | NRF_W_REGISTER), data);
}

void writeRegisterMulti(uint8_t reg, uint8_t *data, uint8_t bytes)
{
  sendPayload((NRF_W_REGISTER | reg), bytes, data);
}

void sendPayload(uint8_t reg, uint8_t bytes, uint8_t *data)
{
  NRF_CSN_lo;
  
  volatile uint8_t blah;
  while(!(NRF_USART->STATUS & USART_STATUS_TXBL));
  NRF_USART->TXDATA = reg;
  while(!(NRF_USART->STATUS & USART_STATUS_TXC)) ;
  blah = NRF_USART->RXDATA;
  int i;
  for (i = 0; i < bytes; i++) {
    while(!(NRF_USART->STATUS & USART_STATUS_TXBL));
	  NRF_USART->TXDATA = data[i];
    while(!(NRF_USART->STATUS & USART_STATUS_TXC)) ;
    blah = NRF_USART->RXDATA;
  }
  
  NRF_CSN_hi;
}

void receivePayload(uint8_t cmd, uint8_t *buf, uint8_t bytes)
{
  NRF_CSN_lo;
  NRF_USART->CMD = USART_CMD_CLEARRX;
  USART_Tx(NRF_USART, cmd);
  USART_Rx(NRF_USART);
  int i;
  for (i = 0; i < bytes; i++) {
	  USART_Tx(NRF_USART, 0xFF);
	  buf[i] = USART_Rx(NRF_USART);
  }
  //while(!(NRF_USART->STATUS & USART_STATUS_TXC)) ;
  NRF_CSN_hi;
}


void sendCommand(uint8_t cmd, uint8_t data)
{
  NRF_CSN_lo;
  while(!(NRF_USART->STATUS & USART_STATUS_TXBL));
  NRF_USART->TXDATA = cmd;
  while(!(NRF_USART->STATUS & USART_STATUS_TXC));
  USART_Rx(NRF_USART);
  NRF_USART->TXDATA = data;
  while (!(NRF_USART->STATUS & USART_STATUS_TXC)) ;
  NRF_CSN_hi;
}  

void RADIO_Interrupt()
{
	
	if (GPIO_IntGet() & (1 << NRF_INT_PIN))
	{
		
		// handle interrupt
		uint8_t status = readRegister(NRF_STATUS);
		
		// tx
		if (status & 0x20)
		{
			
			bufferCount--;
			
			// if nothing left to sendk
			if (bufferCount == 0)
			{
				RADIO_EnableTX(false);
			}
			
		}
		
		// rx
		if (status & 0x40)
		{
			
			bufferCount++;
			
		}
		
		writeRegister(NRF_STATUS,0x70);
		GPIO_IntClear((1 << NRF_INT_PIN));
		
	}
	
}

uint8_t RADIO_GetBufferFill()
{
	
		return bufferCount;
		
}
