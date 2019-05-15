#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "main.h"
#include <math.h>
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

uint8_t uartString[2][NMEA_MAX_LEN_STRING] = {0};
uint32_t pUartLine = 0;
uint32_t pUartSymbol = 0;

void main () {

	geoPointString geoPoint;
	geoPointVector geoVector;
	geoPoint.lat_dir[0] = 0;
	geoPoint.lon_dir[0] = 0;

	geoPoint.lat[0] = 0;
	geoPoint.lon[0] = 0;
	geoVector.magVar[0] = 0;
	geoVector.speedKnt[0] = 0;

    uint32_t pNmeaEolTmp = 0;
    ErrorStatus error;
    RCC_DeInit();
    // HSE Enable
    RCC->APB2ENR |= RCC_APB2ENR_LTDCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI5EN;

    RCC_HSEConfig(RCC_HSE_ON);
    error = RCC_WaitForHSEStartUp();
    FLASH->ACR |= FLASH_ACR_LATENCY_5WS;

    // Main Pll config
    RCC_PLLConfig (RCC_PLLSource_HSE, 8, 360, 2, 7);
    RCC_PLLCmd(ENABLE);

    // System Clock Mux select
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_1) {}

    // PLLSAI config
    RCC_PLLSAIConfig(192,4,4);
    //RCC->DCKCFGR |= PLL
    RCC_LTDCCLKDivConfig(0x20000);
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV4;
    RCC_PLLSAICmd(ENABLE);

//  RCC_MCO1Config(RCC_MCO1Source_PLLCLK,RCC_MCO1Div_1);
    while ((RCC->CR & RCC_CR_PLLSAIRDY) == 0) {}

    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOJEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOKEN;

    LCD_Init();
    LCD_LayerInit();
    LTDC_Cmd(ENABLE);
    LCD_SetLayer(LCD_FOREGROUND_LAYER);
    LCD_Clear(LCD_COLOR_WHITE);
    LCD_SetTransparency(200);
    LTDC_ReloadConfig(LTDC_IMReload);
    LCD_Clear(LCD_COLOR_BLACK);


    UART1_Init();
//  USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
    USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
//  DMA_UART1_Init();
//  DMA_ITConfig(DMA2_Stream2,DMA_IT_TC,ENABLE);

    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
//  NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);


    uint16_t line = 0;
    uint32_t pUartLineTmp = 0;
    double speed = 0.0;
    while (1) {
    	if (pUartLineTmp != pUartLine) {
    		getGpsPointString(uartString[pUartLineTmp], &geoPoint, &geoVector);
    		LCD_DisplayStringLine(LCD_LINE_0,geoPoint.lat);
    		LCD_DisplayStringLine(LCD_LINE_1,geoPoint.lon);
    		LCD_DisplayChar(LCD_LINE_0,(20*8),geoPoint.lat_dir[0]);
    		LCD_DisplayChar(LCD_LINE_1,(20*8),geoPoint.lon_dir[0]);
    		LCD_DisplayStringLine(LCD_LINE_3,geoVector.speedKnt);
    		LCD_DisplayStringLine(LCD_LINE_4,geoVector.magVar);

    		pUartLineTmp = pUartLine;

    		speed = atof(geoVector.speedKnt);
    		speed = speed * 1.852;
    		char speedStr[31] = {0};
    		sprintf(speedStr,"Speed is %.2f",speed);
    		LCD_DisplayStringLine(LCD_LINE_5,speedStr);

    	}
    }
}

void UART1_Init() {

    USART_InitTypeDef usart1Init;
    GPIO_InitTypeDef GPIO_Init_Structure;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    GPIO_Init_Structure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init_Structure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init_Structure.GPIO_Speed = GPIO_Fast_Speed;
    // WRX_Pin
    GPIO_Init_Structure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_PinAFConfig(GPIOA,9,GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA,10,GPIO_AF_USART1);
    GPIO_Init(GPIOA, &GPIO_Init_Structure);

    usart1Init.USART_BaudRate = 9600;
    usart1Init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart1Init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usart1Init.USART_Parity = USART_Parity_No;
    usart1Init.USART_StopBits = USART_StopBits_1;
    usart1Init.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART1, &usart1Init);
    //USART1->BRR = ;
    USART_Cmd(USART1, ENABLE);
}

void DMA_UART1_Init () {
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    DMA_InitTypeDef DmaInitTypeDef;
    DMA_StructInit(&DmaInitTypeDef);
    DmaInitTypeDef.DMA_BufferSize = (NMEA_MAX_LEN_STRING);
    DmaInitTypeDef.DMA_Channel = DMA_Channel_4;
    DmaInitTypeDef.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DmaInitTypeDef.DMA_PeripheralBaseAddr = ((uint32_t)(&USART1->DR));
    DmaInitTypeDef.DMA_Memory0BaseAddr = (uint32_t)&uartString[0];
    DmaInitTypeDef.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//  DmaInitTypeDef.DMA_Mode = DMA_Mode_Circular;
    DmaInitTypeDef.DMA_MemoryInc = DMA_MemoryInc_Enable;


    DMA_DeInit(DMA2_Stream2);
    DMA_Init(DMA2_Stream2, & DmaInitTypeDef);
//  DMA_DoubleBufferModeConfig(DMA2_Stream2, (uint32_t)&uartString[1], DMA_Memory_0);
//  DMA_DoubleBufferModeCmd(DMA2_Stream2,ENABLE);
    DMA_Cmd(DMA2_Stream2,ENABLE);

}



#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  while (1)
  {}
}
#endif
