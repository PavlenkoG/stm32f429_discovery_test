#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#include "misc.h"
#include "ili9341.h"
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_sdram.h"
#include "stm32f429i_discovery_lcd.h"
#include "fonts.h"
#include "geo.h"

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

#define NMEA_MAX_LEN_STRING 84


void UART1_Init();
void DMA_UART1_Init();
uint16_t printNmeaToDisplay (uint16_t line, uint8_t* pNmea);
void GPS_GetCoord(const char* str, char* lat, char* lon);
void GPS_GetDateTime(const char* str, char* date, char* time);
