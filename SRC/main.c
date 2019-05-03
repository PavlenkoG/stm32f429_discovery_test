/*
 * main.c
 *
 *  Created on: 23.04.2019
 *      Author: grpa
 */

#include "main.h"
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#define DISPLAY_WIDTH           ((uint16_t)240)
#define DISPLAY_HEIGHT          ((uint16_t)160)
#define DISPLAY1_WIDTH           ((uint16_t)240)
#define DISPLAY1_HEIGHT          ((uint16_t)320)

typedef enum ColorDisplay {
    RED = 0xF800,
    GREEN = 0x07E0,
    BLUE = 0x001F,
    BLACK = 0x0000,
    WHITE = 0xFFFF
} Color;

void SetPixel (uint16_t setX, uint16_t setY, Color Color, uint16_t* Layer) {
    uint32_t numBuffer = ((setY) * DISPLAY1_WIDTH) + setX;
    Layer[numBuffer] = Color;
}

void main () {
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

    LCD_DisplayStringLine(LCD_LINE_0, (uint8_t *)"  8-bits AHB      ");

    UART1_Init();
    uint16_t rx_data = 0;
    while (1) {
            rx_data = USART_ReceiveData(USART1);
    }
}
#if 0
void main ()
{


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

    GPIO_InitTypeDef GPIO_Init_Structure;


    GPIO_Init_Structure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_6 |
                                GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_Init_Structure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init_Structure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init_Structure.GPIO_Speed = GPIO_Fast_Speed;
    GPIO_PinAFConfig(GPIOA,3,GPIO_AF_LTDC);
    GPIO_PinAFConfig(GPIOA,4,GPIO_AF_LTDC);
    GPIO_PinAFConfig(GPIOA,6,GPIO_AF_LTDC);
    GPIO_PinAFConfig(GPIOA,11,GPIO_AF_LTDC);
    GPIO_PinAFConfig(GPIOA,12,GPIO_AF_LTDC);
    GPIO_Init(GPIOA, &GPIO_Init_Structure);

    /* LTDC pins configuraiton: PB8 -- 11 */
    GPIO_Init_Structure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_PinAFConfig(GPIOB,8,GPIO_AF_LTDC);
    GPIO_PinAFConfig(GPIOB,9,GPIO_AF_LTDC);
    GPIO_PinAFConfig(GPIOB,10,GPIO_AF_LTDC);
    GPIO_PinAFConfig(GPIOB,11,GPIO_AF_LTDC);
    GPIO_Init(GPIOB, &GPIO_Init_Structure);


    /* LTDC pins configuraiton: PC6 -- 10 */
    GPIO_Init_Structure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_10;
    GPIO_PinAFConfig(GPIOC,6,GPIO_AF_LTDC);
    GPIO_PinAFConfig(GPIOC,7,GPIO_AF_LTDC);
    GPIO_PinAFConfig(GPIOC,10,GPIO_AF_LTDC);
    GPIO_Init(GPIOC, &GPIO_Init_Structure);

    /* LTDC pins configuraiton: PD3 -- 6 */
    GPIO_Init_Structure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_6;
    GPIO_PinAFConfig(GPIOD,3,GPIO_AF_LTDC);
    GPIO_PinAFConfig(GPIOD,6,GPIO_AF_LTDC);
    GPIO_Init(GPIOD, &GPIO_Init_Structure);

    /* LTDC pins configuraiton: PF10 */
    GPIO_Init_Structure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |GPIO_Pin_11;
    GPIO_Init(GPIOF, &GPIO_Init_Structure);
    GPIO_Init_Structure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init_Structure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init_Structure.GPIO_Speed = GPIO_Medium_Speed;
    GPIO_PinAFConfig(GPIOF,6,GPIO_AF_SPI5);
    GPIO_PinAFConfig(GPIOF,7,GPIO_AF_SPI5);
    GPIO_PinAFConfig(GPIOF,8,GPIO_AF_SPI5);
    GPIO_PinAFConfig(GPIOF,9,GPIO_AF_SPI5);
    GPIO_PinAFConfig(GPIOF,10,GPIO_AF_LTDC);
    GPIO_PinAFConfig(GPIOF,11,GPIO_AF_SPI5);
    GPIO_Init(GPIOF, &GPIO_Init_Structure);

    /* LTDC pins configuraiton: PG6 -- 11 */
    GPIO_Init_Structure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | \
            GPIO_Pin_10| GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_PinAFConfig(GPIOG,6,GPIO_AF_LTDC);
    GPIO_PinAFConfig(GPIOG,7,GPIO_AF_LTDC);
    GPIO_PinAFConfig(GPIOG,10,GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOG,11,GPIO_AF_LTDC);
    GPIO_PinAFConfig(GPIOG,12,GPIO_AF_CAN1);
    GPIO_Init(GPIOG, &GPIO_Init_Structure);

    GPIO_Init_Structure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_PinAFConfig(GPIOB,0,GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOB,1,GPIO_AF_CAN1);
    GPIO_Init(GPIOB, &GPIO_Init_Structure);

    ili9341_Init();
    // LTDC Config
    LTDC_InitTypeDef ltdcHandle;
    LTDC_StructInit(&ltdcHandle);
    // Polarity configuration
    ltdcHandle.LTDC_HSPolarity = LTDC_HSPolarity_AL;
    ltdcHandle.LTDC_VSPolarity = LTDC_VSPolarity_AL;
    ltdcHandle.LTDC_DEPolarity = LTDC_DEPolarity_AL;
    ltdcHandle.LTDC_PCPolarity = LTDC_PCPolarity_IPC;

    ltdcHandle.LTDC_HorizontalSync = 9;
    ltdcHandle.LTDC_VerticalSync = 1;
    ltdcHandle.LTDC_AccumulatedHBP = 29;//3;
    ltdcHandle.LTDC_AccumulatedVBP = 3;
    ltdcHandle.LTDC_AccumulatedActiveH = 323;
    ltdcHandle.LTDC_AccumulatedActiveW = 269;
    ltdcHandle.LTDC_TotalHeigh = 327;//327;
    ltdcHandle.LTDC_TotalWidth = 279;//279;
    ltdcHandle.LTDC_BackgroundBlueValue = 0;
    ltdcHandle.LTDC_BackgroundGreenValue = 0;
    ltdcHandle.LTDC_BackgroundRedValue = 0;

    // Layer Config
    LTDC_Layer_InitTypeDef pLayer1Cfg;
    LTDC_LayerStructInit(&pLayer1Cfg);
    pLayer1Cfg.LTDC_HorizontalStart = 30;
    pLayer1Cfg.LTDC_HorizontalStop = 270;
    pLayer1Cfg.LTDC_VerticalStart = 4;
    pLayer1Cfg.LTDC_VerticalStop = 164;

    pLayer1Cfg.LTDC_PixelFormat = LTDC_Pixelformat_RGB565;
//  pLayer1Cfg.LTDC_CFBStartAdress = (uint32_t)&imageLayer1;//ST_LOGO_1;
    pLayer1Cfg.LTDC_BlendingFactor_1 = LTDC_BlendingFactor1_PAxCA;
    pLayer1Cfg.LTDC_ConstantAlpha = 0xff;
    pLayer1Cfg.LTDC_DefaultColorAlpha = 0;
    pLayer1Cfg.LTDC_DefaultColorRed = 0x0;
    pLayer1Cfg.LTDC_DefaultColorGreen = 0x0;
    pLayer1Cfg.LTDC_DefaultColorBlue = 0x0;
    pLayer1Cfg.LTDC_CFBPitch = 480;//480;
    pLayer1Cfg.LTDC_CFBLineLength = 486;//486;
    pLayer1Cfg.LTDC_CFBLineNumber = 160;

    LTDC_Layer_InitTypeDef pLayer2Cfg;
    LTDC_LayerStructInit(&pLayer2Cfg);
    pLayer2Cfg.LTDC_HorizontalStart = 30;
    pLayer2Cfg.LTDC_HorizontalStop = 270;
    pLayer2Cfg.LTDC_VerticalStart = 4;
    pLayer2Cfg.LTDC_VerticalStop = 324;//327;

    pLayer2Cfg.LTDC_PixelFormat = LTDC_Pixelformat_RGB565;
    pLayer2Cfg.LTDC_CFBStartAdress = (uint32_t)&imageLayer2;
    pLayer2Cfg.LTDC_BlendingFactor_2 = LTDC_BlendingFactor2_PAxCA;
    pLayer2Cfg.LTDC_ConstantAlpha = 0xff;
    pLayer2Cfg.LTDC_DefaultColorAlpha = 0;
    pLayer2Cfg.LTDC_DefaultColorRed = 0x0;
    pLayer2Cfg.LTDC_DefaultColorGreen = 0x0;
    pLayer2Cfg.LTDC_DefaultColorBlue = 0x0;
    pLayer2Cfg.LTDC_CFBPitch = 480;
    pLayer2Cfg.LTDC_CFBLineLength = 486;
    pLayer2Cfg.LTDC_CFBLineNumber = 320;


    LTDC_Init(&ltdcHandle);
//  LTDC_LayerInit(LTDC_Layer1,&pLayer1Cfg);
//  LTDC_LayerCmd(LTDC_Layer1,ENABLE);
    LTDC_LayerInit(LTDC_Layer2,&pLayer2Cfg);
    LTDC_LayerCmd(LTDC_Layer2,ENABLE);
    LTDC_Cmd (ENABLE);
    LTDC_ReloadConfig(LTDC_SRCR_IMR);


//   for (int i = 0; i < 320; i++) {
//         SetPixel(0,i,WHITE, &imageLayer2);
//         imageLayer2[240 * i] = WHITE;
//   }
//  for (int i = 0; i < 240; i++) {
//      SetPixel(i,0,BLUE, &imageLayer1);
//  }

    while (1) {
        for (int j = 0; j < 240; j ++) {
            for (int i = 0; i < 320; i++) {
                //imageLayer2[i*240] = WHITE;
                SetPixel(j,i,WHITE, &imageLayer2);
            }
            LCD_Delay(5);
            for (int i = 0; i < 320; i++) {
                //imageLayer2[i*240] = WHITE;
                SetPixel(j,i,BLACK, &imageLayer2);
            }
        }
    }
}
#endif

void LCD_IO_Init () {

    GPIO_InitTypeDef GPIO_Init_Structure;

    GPIO_Init_Structure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init_Structure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init_Structure.GPIO_Speed = GPIO_Fast_Speed;
    // WRX_Pin
    GPIO_Init_Structure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_12;
    GPIO_Init(GPIOD, &GPIO_Init_Structure);
    // NCX_Pin
    GPIO_Init_Structure.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOC, &GPIO_Init_Structure);

    LCD_CS_LOW();
    LCD_CS_HIGH();
    SPIx_Init();

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
    USART_Cmd(USART1, ENABLE);
}

void SPIx_Init ()
{

    SPI_InitTypeDef spiInit;
    SPI_StructInit(&spiInit);
    spiInit.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
    spiInit.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spiInit.SPI_CPHA = SPI_CPHA_1Edge;
    spiInit.SPI_CPOL = SPI_CPOL_Low;
    spiInit.SPI_DataSize = SPI_DataSize_8b;
    spiInit.SPI_FirstBit = SPI_FirstBit_MSB;
    spiInit.SPI_NSS = SPI_NSS_Soft;
    spiInit.SPI_Mode = SPI_Mode_Master;
    SPI_Init(SPI5,&spiInit);
    SPI_Cmd (SPI5, ENABLE);
}
/**
  * @brief  Writes register value.
  */
void LCD_IO_WriteData(uint16_t RegValue)
{
  /* Set WRX to send data */
  LCD_WRX_HIGH();

  /* Reset LCD control line(/CS) and Send data */
  LCD_CS_LOW();
  SPIx_Write(RegValue);

  /* Deselect: Chip Select high */
  LCD_CS_HIGH();
}

/**
  * @brief  Writes register address.
  */
void LCD_IO_WriteReg(uint8_t Reg)
{
  /* Reset WRX to send command */
  LCD_WRX_LOW();

  /* Reset LCD control line(/CS) and Send command */
  LCD_CS_LOW();
  SPIx_Write(Reg);

  /* Deselect: Chip Select high */
  LCD_CS_HIGH();
}

/**
  * @brief  Reads register value.
  * @param  RegValue Address of the register to read
  * @param  ReadSize Number of bytes to read
  * @retval Content of the register value
  */
uint32_t LCD_IO_ReadData(uint16_t RegValue, uint8_t ReadSize)
{
  uint32_t readvalue = 0;

  /* Select: Chip Select low */
  LCD_CS_LOW();

  /* Reset WRX to send command */
  LCD_WRX_LOW();

  SPIx_Write(RegValue);

  readvalue = SPIx_Read(ReadSize);

  /* Set WRX to send data */
  LCD_WRX_HIGH();

  /* Deselect: Chip Select high */
  LCD_CS_HIGH();

  return readvalue;
}

/**
  * @brief  Wait for loop in ms.
  * @param  Delay in ms.
  */
void LCD_Delay(uint32_t Delay)
{
    for (uint32_t i = 0; i < Delay*10000; i++){} ;
}

/**
  * @brief  Reads 4 bytes from device.
  * @param  ReadSize: Number of bytes to read (max 4 bytes)
  * @retval Value read on the SPI
  */
static uint16_t SPIx_Read(uint8_t ReadSize)
{
  uint16_t readvalue;

  readvalue = SPI_I2S_ReceiveData(SPI5);

  return readvalue;
}

/**
  * @brief  Writes a byte to device.
  * @param  Value: value to be written
  */
static void SPIx_Write(uint16_t Value)
{
    SPI_I2S_SendData(SPI5, Value);
    while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_BSY)) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
    /* Infinite loop /
    / Use GDB to find out why we're here */
    while (1);
    }
#endif
