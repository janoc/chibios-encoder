
#include "lcd-setup.h"

//
//HD44780 commands and related options (taken from BusPirate source)
//
#define CMD_CLEARDISPLAY        0b00000001 //82us-1.64ms

#define CMD_RETURNHOME          0b00000010 //40us-1.64ms

#define CMD_ENTRYMODESET        0b00000100 //40us
#define INCREMENT 0b10
#define DECREMENT 0b00
#define DISPLAYSHIFTON 0b1
#define DISPLAYSHIFTOFF 0

#define CMD_DISPLAYCONTROL      0b00001000 //40us
#define DISPLAYON 0b100
#define DISPLAYOFF 0
#define CURSERON 0b10
#define CURSEROFF 0
#define BLINKON 0b1
#define BLINKOFF 0

#define CMD_CURSERDISPLAYSHIFT 0b00010000 //40us
#define DISPLAYSHIFT 0b1000
#define CURSERMOVE 0
#define SHIFTRIGHT 0b100
#define SHIFTLEFT 0

#define CMD_FUNCTIONSET         0b00100000 //40us
#define DATAWIDTH8 0b10000
#define DATAWIDTH4 0
#define DISPLAYLINES2 0b1000
#define DISPLAYLINES1 0
#define FONT5X10 0b100
#define FONT5X7 0
#define MODULE24X4 0b1

#define CMD_SETCGRAMADDR        0b01000000 //40us
//6bit character generator RAM address

#define CMD_SETDDRAMADDR        0b10000000 //40us
//7bit display data RAM address

void lcdInit(SPIDriver *spip)
{
    // reset the display
    lcdReset(spip);

    // clear the display and set the cursor home
    lcdClearDisplay(spip);
}

void lcdReset(SPIDriver *spip)
{
    // clear I/O pins in the shift register
    uint8_t pkt = 0;    

    spiAcquireBus(spip);
    spiStart(spip, &spicfg);
    spiSelect(spip);
    spiSend(spip, 1, &pkt);
    spiUnselect(spip);
    spiStop(&SPID1);
    spiReleaseBus(&SPID1);
    
    chThdSleepMicroseconds(500);

    // write 0x03 3 times
    lcdWriteNibble(spip, LCD_COMMAND, 0x03);
    lcdWriteNibble(spip, LCD_COMMAND, 0x03);
    lcdWriteNibble(spip, LCD_COMMAND, 0x03);
    chThdSleepMilliseconds(50); // wait for the reset to complete

    // enable 4bit mode
    lcdWriteNibble(spip, LCD_COMMAND, 0x02);
    chThdSleepMilliseconds(160); // wait for the op to complete

    // 4bit mode, two lines (the 1), 5x7 font
    lcdWriteByte(spip, LCD_COMMAND, (CMD_FUNCTIONSET + DATAWIDTH4 + FONT5X7 + DISPLAYLINES2));
    chThdSleepMilliseconds(1); // wait for the op to complete 

    // enable display & cursor
    lcdWriteByte(spip, LCD_COMMAND, (CMD_DISPLAYCONTROL + DISPLAYON + CURSERON));
    chThdSleepMilliseconds(1); // wait for the op to complete
    
}

void lcdClearDisplay(SPIDriver *spip)
{
    lcdWriteByte(spip, LCD_COMMAND, CMD_CLEARDISPLAY);
    chThdSleepMilliseconds(2); // wait for the op to complete
}

void lcdSetAddr(SPIDriver *spip, uint8_t addr)
{
    lcdWriteByte(spip, LCD_COMMAND, CMD_SETDDRAMADDR | addr);
}

void lcdWriteString(SPIDriver *spip, const char *str)
{
    const char *c = str;

    while(*c != 0)
    {
        lcdWriteByte(spip, LCD_DATA, *c);
        ++c;
    }
}

void lcdWriteByte(SPIDriver *spip, uint8_t reg, uint8_t b)
{
    lcdWriteNibble(spip, reg, b >> 4);
    lcdWriteNibble(spip, reg, b & 0x0F);
}

void lcdWriteNibble(SPIDriver *spip, uint8_t reg, uint8_t b)
{
#define LED_BIT 1
#define RS_BIT  2    // 0 - command, 1 - data
#define RW_BIT  4    // 0 - write, 1 - read
#define EN_BIT  8    // latch
    
    uint8_t pkt;

    // first write - set the 4 LCD reg bits to the desired value
    pkt = ((b & 0x0F) << 4) | (reg ? RS_BIT : 0) | LED_BIT;

    spiAcquireBus(spip);

    spiStart(spip, &spicfg);
    spiSelect(spip);
    spiSend(spip, 1, &pkt);
    spiUnselect(spip);
    spiStop(spip);
    chThdSleepMicroseconds(500);

    // second write - keep 4 LCD reg bits, raise EN pin
    pkt |= EN_BIT;
    spiStart(spip, &spicfg);    
    spiSelect(spip);
    spiSend(spip, 1, &pkt);
    spiUnselect(spip);
    spiStop(spip);
    chThdSleepMicroseconds(500);

    // third write - lower the EN pin, latch to the display
    pkt &= (~EN_BIT);
    spiStart(spip, &spicfg);        
    spiSelect(spip);
    spiSend(spip, 1, &pkt);
    spiUnselect(spip);
    spiStop(spip);
    chThdSleepMicroseconds(500);

    spiReleaseBus(spip);
}
















