#include "main.h"
#include "stm32f4xx_dma.h"

#if 0
extern uint32_t pUartLine;
extern uint32_t pUartSymbol;
extern uint8_t uartString[2][NMEA_MAX_LEN_STRING];


void USART1_IRQHandler () {
	if( USART_GetITStatus(USART1, USART_IT_RXNE)) {
		uint16_t symbol = USART_ReceiveData(USART1);
		if ((uint8_t)symbol == '\n') {
			uartString[pUartLine][pUartSymbol] = (uint8_t)symbol;
			pUartSymbol++;
			uartString[pUartLine][pUartSymbol] = 0;
			if (pUartLine)
				pUartLine = 0;
			else
				pUartLine = 1;
			pUartSymbol = 0;
		} else {
			uartString[pUartLine][pUartSymbol] =(uint8_t)symbol;
			if (pUartSymbol < NMEA_MAX_LEN_STRING)
				pUartSymbol++;
			else
				pUartSymbol = 0;
		}
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}

void DMA2_Stream2_IRQHandler() {
}

#endif
