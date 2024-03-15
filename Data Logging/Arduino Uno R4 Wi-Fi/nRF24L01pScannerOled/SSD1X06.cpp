/*  TWI (I2C)  code to drive 128x64 monochrome oled display module 
 *  ceptimus.  September 2016
 */
#include "SSD1X06.h"
#include <Wire.h>

void cmd(uint8_t c) {
  Wire.beginTransmission(SSD1X06_I2C_ADDRESS);
  Wire.write(0);
  Wire.write(c);
  Wire.endTransmission();
}

void SSD1X06::fillDisplay(uint8_t c) { // fill whole display using character c
  for (uint8_t row = 0; row < SSD1X06_CHARHEIGHT; row++) {
    for (uint8_t x = 0; x < SSD1X06_LCDWIDTH; x += 6) {
      displayChar6x8(row, x, c);
    }
  }
}

void SSD1X06::start(void) {
  Wire.begin();
  Wire.setClock(400000L);
  
  // initial setup commands for the display
#ifdef SSD1306  
  cmd(SSD1X06_SETMULTIPLEX); cmd(0x3F);
  cmd(SSD1X06_SETDISPLAYOFFSET); cmd(0x00);
  cmd(SSD1X06_SETSTARTLINE);
  cmd(SSD1X06_SEGREMAP | 0x01);
  cmd(SSD1X06_COMSCANDEC);
  cmd(SSD1X06_SETCOMPINS); cmd(0x12);
  cmd(SSD1X06_SETCONTRAST); cmd(0xFF);
  cmd(SSD1X06_DISPLAYALLON_RESUME);
  cmd(SSD1X06_NORMALDISPLAY);
  cmd(SSD1X06_SETDISPLAYCLOCKDIV); cmd(0x80);
  cmd(SSD1X06_CHARGEPUMP); cmd(0x14);
  cmd(SSD1X06_SETPRECHARGE); cmd(0x22);
  cmd(SSD1X06_MEMORYMODE); cmd(0x01);
  cmd(SSD1X06_SETVCOMDETECT); cmd(0x20);
  cmd(SSD1X06_DISPLAYON);
#endif

#ifdef SSD1106  
  cmd(0xAE);    /*display off*/

  cmd(0x02);    /*set lower column address*/
  cmd(0x10);    /*set higher column address*/

  cmd(0x40);    /*set display start line*/

  cmd(0xB0);    /*set page address*/

  cmd(0x81);    /*contract control*/
  cmd(0x80);    /*128*/

  cmd(0xA1);    /*set segment remap*/

  cmd(0xA6);    /*normal / reverse*/

  cmd(0xA8);    /*multiplex ratio*/
  cmd(0x3F);    /*duty = 1/32*/

  cmd(0xad);    /*set charge pump enable*/
  cmd(0x8b);     /*external VCC   */

  cmd(0x30);    /*0X30---0X33  set VPP   9V liangdu!!!!*/

  cmd(0xC8);    /*Com scan direction*/

  cmd(0xD3);    /*set display offset*/
  cmd(0x00);   /*   0x20  */

  cmd(0xD5);    /*set osc division*/
  cmd(0x80);

  cmd(0xD9);    /*set pre-charge period*/
  cmd(0x1f);    /*0x22*/

  cmd(0xDA);    /*set COM pins*/
  cmd(0x12);

  cmd(0xdb);    /*set vcomh*/
  cmd(0x40);

  cmd(0xAF);    /*display ON*/

#endif

}
 
void SSD1X06::displayString6x8(uint8_t row, uint8_t x, const char *s, uint8_t rvsField) {
  if (row > SSD1X06_CHARHEIGHT - 1) {
    row = SSD1X06_CHARHEIGHT - 1;
  }
  if (rvsField) { rvsField = 0x80; }
  
  while (uint8_t c = *s++) {
    displayChar6x8(row, x, c ^ rvsField);
    x += 6;
  }
  
}

void SSD1X06::displayString6x8(uint8_t row, uint8_t x, const __FlashStringHelper *s, uint8_t rvsField) {
  if (rvsField) { rvsField = 0x80; }
  uint8_t PROGMEM *p = (uint8_t PROGMEM *)s;
  while (uint8_t c = pgm_read_byte_near(p++)) {
    displayChar6x8(row, x, c ^ rvsField);
    x += 6;
  }
}

void SSD1X06::displayByte(uint8_t row, uint8_t x, uint8_t b) {
#ifdef SSD1306
  cmd(SSD1X06_PAGEADDR);
  cmd(row);
  cmd(row);
  cmd(SSD1X06_COLUMNADDR);
  cmd(x);
  cmd(x < SSD1X06_LCDWIDTH - 6 ? x + 5 : SSD1X06_LCDWIDTH - 1);
#endif

#ifdef SSD1106
  cmd(0xB0 + row);
  x += 2;
  cmd(x & 0x0F);
  cmd(((x >> 4) & 0x0F) | 0x10);
#endif

  Wire.beginTransmission(SSD1X06_I2C_ADDRESS);
  Wire.write(0x40);
  Wire.write(b);
  Wire.endTransmission();
}

void SSD1X06::displayChar6x8(uint8_t row, uint8_t x, uint8_t c) {
  uint8_t xorMask = c & 0x80 ? 0xFF : 0x00;  // if MSB set display character reverse field

#ifdef SSD1306
  cmd(SSD1X06_PAGEADDR);
  cmd(row);
  cmd(row);
  cmd(SSD1X06_COLUMNADDR);
  cmd(x);
  cmd(x < SSD1X06_LCDWIDTH - 6 ? x + 5 : SSD1X06_LCDWIDTH - 1);
#endif

#ifdef SSD1106
  cmd(0xB0 + row);
  x += 2;
  cmd(x & 0x0F);
  cmd(((x >> 4) & 0x0F) | 0x10);
#endif

  Wire.beginTransmission(SSD1X06_I2C_ADDRESS);
  Wire.write(0x40);
  Wire.write(xorMask);
 
  uint16_t p = ((c & 0x7F) - 32) * 5;
  for (uint16_t i = 2; i < 7; i++ ) {
    Wire.write(pgm_read_byte_near(font + p++) ^ xorMask);
  }
  Wire.endTransmission();
}


