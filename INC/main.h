#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#include "misc.h"
#include "ili9341.h"
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_sdram.h"
#include "stm32f429i_discovery_lcd.h"
#include "fonts.h"

#define LCD_CS_LOW()        GPIO_WriteBit(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_RESET)
#define LCD_CS_HIGH()       GPIO_WriteBit(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_SET)

#define LCD_WRX_LOW()       GPIO_WriteBit(LCD_WRX_GPIO_PORT, LCD_WRX_PIN, Bit_RESET)
#define LCD_WRX_HIGH()      GPIO_WriteBit(LCD_WRX_GPIO_PORT, LCD_WRX_PIN, Bit_SET)

#define LCD_RDX_LOW()       GPIO_WriteBit(LCD_RDX_GPIO_PORT, LCD_RDX_PIN, Bit_RESET)
#define LCD_RDX_HIGH()      GPIO_WriteBit(LCD_RDX_GPIO_PORT, LCD_RDX_PIN, Bit_SET)

#define LCD_NCS_PIN                             GPIO_Pin_2
#define LCD_NCS_GPIO_PORT                       GPIOC
#define LCD_WRX_PIN                             GPIO_Pin_13
#define LCD_WRX_GPIO_PORT                       GPIOD
#define LCD_RDX_PIN                             GPIO_Pin_12
#define LCD_RDX_GPIO_PORT                       GPIOD

void LCD_IO_Init ();
void SPIx_Init ();

void LCD_IO_WriteData(uint16_t RegValue);
void LCD_IO_WriteReg(uint8_t Reg);

uint32_t LCD_IO_ReadData(uint16_t RegValue, uint8_t ReadSize);
void LCD_Delay(uint32_t Delay);
static uint16_t SPIx_Read(uint8_t ReadSize);
static void SPIx_Write(uint16_t Value);

