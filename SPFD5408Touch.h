// Touch screen library with X Y and Z (pressure) readings as well
// as oversampling to avoid 'bouncing'
// (c) ladyada / adafruit
// Code under MIT License

#ifndef _SPFD5408Touch_MLX_H_
#define _SPFD5408Touch_MLX_H_   

#include <stdint.h>

class SPFD5408TouchPoint {
public:
	SPFD5408TouchPoint(void);
	SPFD5408TouchPoint(int16_t x, int16_t y, int16_t z);

	bool operator==(SPFD5408TouchPoint);
	bool operator!=(SPFD5408TouchPoint);

	int16_t x, y, z;
};

class SPFD5408TouchScreen {
public:
	SPFD5408TouchScreen(uint8_t xp, uint8_t yp, uint8_t xm, uint8_t ym);
	SPFD5408TouchScreen(uint8_t xp, uint8_t yp, uint8_t xm, uint8_t ym, uint16_t rx);

	bool isTouching(void);
	uint16_t pressure(void);
	int readTouchY();
	int readTouchX();
	SPFD5408TouchPoint getPoint();
	int16_t pressureThreshhold;

private:
	uint8_t _yp, _ym, _xm, _xp;
	uint16_t _rxplate;
};

#endif

