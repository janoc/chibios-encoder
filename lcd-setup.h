
#ifndef LCD_SETUP_H_
#define LCD_SETUP_H_

#include "ch.h"
#include "hal.h"

#define LCD_CS_PIN GPIOA_PIN4

/*
 * SPI configuration structure.
 * fpclk/32 speed, CPHA=0, CPOL=0, 8bits frames, MSb transmitted first.
 * The slave select line is the pin GPIOE_PIN3 on the port GPIOA.
 */

/*
  Pinout
#define GPIOA_SPI1_SCK              5
#define GPIOA_SPI1_MOSI             7
#define GPIOE_SPI1_CS               3
*/

static const SPIConfig spicfg = {
  NULL,
  GPIOE,                                                        /*port of CS  */
  GPIOE_SPI1_CS,                                                /*pin of CS   */
  SPI_CR1_BR_2 | SPI_CR1_CPOL | SPI_CR1_CPHA,                   /*CR1 register*/
  SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0                    /*CR2 register*/
};


#define LCD_COMMAND 0
#define LCD_DATA 1

void lcdInit(SPIDriver *spip);
void lcdReset(SPIDriver *spip);
void lcdClearDisplay(SPIDriver *spip);
void lcdSetAddr(SPIDriver *spip, uint8_t addr);

void lcdWriteNibble(SPIDriver *spip, uint8_t reg, uint8_t b);
void lcdWriteByte(SPIDriver *spip, uint8_t reg, uint8_t b);
void lcdWriteString(SPIDriver *spip, const char *str);
#endif
