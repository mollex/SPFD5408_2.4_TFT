#ifndef _SPFD5408TFT_MLX_H_
#define _SPFD5408TFT_MLX_H_   

#include "Adafruit_GFX.h"

/***************************************************************************************
**                         Section 6: Colour enumeration
***************************************************************************************/
// Default color definitions
#define TFT_BLACK       0x0000      /*   0,   0,   0 */
#define TFT_NAVY        0x000F      /*   0,   0, 128 */
#define TFT_DARKGREEN   0x03E0      /*   0, 128,   0 */
#define TFT_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define TFT_MAROON      0x7800      /* 128,   0,   0 */
#define TFT_PURPLE      0x780F      /* 128,   0, 128 */
#define TFT_OLIVE       0x7BE0      /* 128, 128,   0 */
#define TFT_LIGHTGREY   0xD69A      /* 211, 211, 211 */
#define TFT_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define TFT_BLUE        0x001F      /*   0,   0, 255 */
#define TFT_GREEN       0x07E0      /*   0, 255,   0 */
#define TFT_CYAN        0x07FF      /*   0, 255, 255 */
#define TFT_RED         0xF800      /* 255,   0,   0 */
#define TFT_MAGENTA     0xF81F      /* 255,   0, 255 */
#define TFT_YELLOW      0xFFE0      /* 255, 255,   0 */
#define TFT_WHITE       0xFFFF      /* 255, 255, 255 */
#define TFT_ORANGE      0xFDA0      /* 255, 180,   0 */
#define TFT_GREENYELLOW 0xB7E0      /* 180, 255,   0 */
#define TFT_PINK        0xFE19      /* 255, 192, 203 */ //Lighter pink, was 0xFC9F
#define TFT_BROWN       0x9A60      /* 150,  75,   0 */
#define TFT_GOLD        0xFEA0      /* 255, 215,   0 */
#define TFT_SILVER      0xC618      /* 192, 192, 192 */
#define TFT_SKYBLUE     0x867D      /* 135, 206, 235 */
#define TFT_VIOLET      0x915C      /* 180,  46, 226 */


/*
 *   Support:  Arduino Uno, Duemilanove, etc.
 */
class SPFD5408TFT : public Adafruit_GFX {

public:

	SPFD5408TFT(int CS = A3, int RS = A2, int WR = A1, int RD = A0, int RST = A4);

	void     reset(void);                                     
	void     begin(uint16_t ID = 0x9320);
	virtual void     drawPixel(int16_t x, int16_t y, uint16_t color);  
	void     WriteCmdData(uint16_t cmd, uint16_t dat);                
	void     pushCommand(uint16_t cmd, uint8_t* block, int8_t N);
	uint16_t color565(uint8_t r, uint8_t g, uint8_t b) { return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3); }
	uint16_t readID(void);
	virtual void     fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
	virtual void     drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) { fillRect(x, y, 1, h, color); }
	virtual void     drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) { fillRect(x, y, w, 1, color); }
	virtual void     fillScreen(uint16_t color) { fillRect(0, 0, _width, _height, color); }
	virtual void     setRotation(uint8_t r);
	virtual void     invertDisplay(boolean i);

	uint16_t readReg(uint16_t reg);
	uint32_t readReg32(uint16_t reg);
	int16_t  readGRAM(int16_t x, int16_t y, uint16_t* block, int16_t w, int16_t h);
	uint16_t readPixel(int16_t x, int16_t y) { uint16_t color; readGRAM(x, y, &color, 1, 1); return color; }
	void     setAddrWindow(int16_t x, int16_t y, int16_t x1, int16_t y1);
	void     pushColors(uint16_t color);
	void     pushColors(uint16_t* block, int16_t n, bool first);
	void     pushColors(uint8_t* block, int16_t n, bool first);
	void     pushColors(const uint8_t* block, int16_t n, bool first);
	void     vertScroll(int16_t top, int16_t scrollines, int16_t offset);
	uint8_t flag_write_bmp;
	void startWrite();

private:
	uint16_t _lcd_ID, _lcd_rev, _lcd_madctl, _lcd_drivOut, _MC, _MP, _MW, _SC, _EC, _SP, _EP;
	uint16_t  _lcd_xor, _lcd_capable;
};

#endif
