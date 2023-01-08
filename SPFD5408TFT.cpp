
#include "SPFD5408TFT.h"

/********************************* _pin_magic_ ***********************************************/
#include <avr/io.h>

 //Support:  Arduino Uno, Duemilanove, etc.
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined (__AVR_ATmega328__) || defined(__AVR_ATmega8__)

#define RD_PORT PORTC
#define RD_PIN  0
#define WR_PORT PORTC
#define WR_PIN  1
#define CD_PORT PORTC
#define CD_PIN  2
#define CS_PORT PORTC
#define CS_PIN  3
#define RESET_PORT PORTC
#define RESET_PIN  4

#define DMASK         0x03
#define NMASK         ~DMASK

// Write 8-bit value to LCD data lines
#define write_8(x) { PORTD = (PORTD & B00101111) | ((x) & B11010000); \
                     PORTB = (PORTB & B11010000) | ((x) & B00101111);} // STROBEs are defined later
#define read_8() (PIND & B11010000) | (PINB & B00101111)
// These set the PORT directions as required before the write and read
// operations.  Because write operations are much more common than reads,
// the data-reading functions in the library code set the PORT(s) to
// input before a read, and restore them back to the write state before
// returning.  This avoids having to set it for output inside every
// drawing method.  The default state has them initialized for writes.
#define setWriteDir() { DDRD |=  B11010000; DDRB |=  B00101111; }
#define setReadDir()  { DDRD &= ~B11010000; DDRB &= ~B00101111; }

#define write8(x)     { write_8(x); WR_STROBE; }
#define write16(x)    { uint8_t h = (x)>>8, l = x; write8(h); write8(l); }
#define READ_8(dst)   { RD_STROBE; dst = read_8(); RD_IDLE; }
#define READ_16(dst)  { uint8_t hi; READ_8(hi); READ_8(dst); dst |= (hi << 8); }

#define PIN_LOW(p, b)        (p) &= ~(1<<(b))
#define PIN_HIGH(p, b)       (p) |= (1<<(b))
#define PIN_OUTPUT(p, b)     *(&p-1) |= (1<<(b))

#else
    #error MCU unsupported
#endif  

#define RD_ACTIVE  PIN_LOW(RD_PORT, RD_PIN)
#define RD_IDLE    PIN_HIGH(RD_PORT, RD_PIN)
#define RD_OUTPUT  PIN_OUTPUT(RD_PORT, RD_PIN)
#define WR_ACTIVE  PIN_LOW(WR_PORT, WR_PIN)
#define WR_IDLE    PIN_HIGH(WR_PORT, WR_PIN)
#define WR_OUTPUT  PIN_OUTPUT(WR_PORT, WR_PIN)
#define CD_COMMAND PIN_LOW(CD_PORT, CD_PIN)
#define CD_DATA    PIN_HIGH(CD_PORT, CD_PIN)
#define CD_OUTPUT  PIN_OUTPUT(CD_PORT, CD_PIN)
#define CS_ACTIVE  PIN_LOW(CS_PORT, CS_PIN)
#define CS_IDLE    PIN_HIGH(CS_PORT, CS_PIN)
#define CS_OUTPUT  PIN_OUTPUT(CS_PORT, CS_PIN)
#define RESET_ACTIVE  PIN_LOW(RESET_PORT, RESET_PIN)
#define RESET_IDLE    PIN_HIGH(RESET_PORT, RESET_PIN)
#define RESET_OUTPUT  PIN_OUTPUT(RESET_PORT, RESET_PIN)

// General macros.   IOCLR registers are 1 cycle when optimised.
#define WR_STROBE { WR_ACTIVE; WR_IDLE; }       //PWLW=TWRL=50ns
#define RD_STROBE RD_IDLE, RD_ACTIVE, RD_ACTIVE, RD_ACTIVE      //PWLR=TRDL=150ns, tDDR=100ns
#define CTL_INIT()   { RD_OUTPUT; WR_OUTPUT; CD_OUTPUT; CS_OUTPUT; }

#define WriteCmd(x)  { CD_COMMAND; write16(x); }
#define WriteData(x) { CD_DATA; write16(x); }

/********************************* _settings_ ***********************************************/

#define USING_16BIT_BUS 0
#define SUPPORT_9320


#define wait_ms(ms)  delay(ms)
#define MIPI_DCS_REV1   (1<<0)
#define AUTO_READINC    (1<<1)
#define READ_BGR        (1<<2)
#define READ_LOWHIGH    (1<<3)
#define READ_24BITS     (1<<4)
#define XSA_XEA_16BIT   (1<<5)
#define READ_NODUMMY    (1<<6)
#define INVERT_GS       (1<<8)
#define INVERT_SS       (1<<9)
#define MV_AXIS         (1<<10)
#define INVERT_RGB      (1<<11)
#define REV_SCREEN      (1<<12)
#define FLIP_VERT       (1<<13)
#define FLIP_HORIZ      (1<<14)

/********************************* _functions_ ***********************************************/

SPFD5408TFT::SPFD5408TFT(int CS, int RS, int WR, int RD, int RST) : Adafruit_GFX(240, 320)
{
    // we can not access GPIO pins until AHB has been enabled.
}

static uint8_t done_reset, is8347;

void SPFD5408TFT::reset(void)
{
    done_reset = 1;
    setWriteDir();
    CTL_INIT();
    CS_IDLE;
    RD_IDLE;
    WR_IDLE;
#ifdef USE_OPENSMART_SHIELD_PINOUT
    digitalWrite(5, LOW);
    delay(200);
    digitalWrite(5, HIGH);
#else
    RESET_ACTIVE;
    delay(2);
    RESET_IDLE;
#endif
    WriteCmdData(0xB0, 0x0000);   //R61520 needs this to read ID
}

void SPFD5408TFT::WriteCmdData(uint16_t cmd, uint16_t dat)
{
    CS_ACTIVE;
    WriteCmd(cmd);
    WriteData(dat);
    CS_IDLE;
}

static void WriteCmdParamN(uint16_t cmd, int8_t N, uint8_t* block)
{
    CS_ACTIVE;
    WriteCmd(cmd);
    while (N-- > 0) {
        uint8_t u8 = *block++;
        CD_DATA;
        write8(u8);
        if (N && is8347) {
            cmd++;
            WriteCmd(cmd);
        }
    }

    CS_IDLE;
}

static inline void WriteCmdParam4(uint8_t cmd, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4)
{
    uint8_t d[4];
    d[0] = d1, d[1] = d2, d[2] = d3, d[3] = d4;
    WriteCmdParamN(cmd, 4, d);
}

//#define WriteCmdParam4(cmd, d1, d2, d3, d4) {uint8_t d[4];d[0] = d1, d[1] = d2, d[2] = d3, d[3] = d4;WriteCmdParamN(cmd, 4, d);}
void SPFD5408TFT::pushCommand(uint16_t cmd, uint8_t* block, int8_t N) { WriteCmdParamN(cmd, N, block); }

static uint16_t read16bits(void)
{
    uint16_t ret;
    uint8_t lo;
#if USING_16BIT_BUS
    READ_16(ret);               //single strobe to read whole bus
    if (ret > 255)              //ID might say 0x00D3
        return ret;
#else
    READ_8(ret);
#endif
    //all MIPI_DCS_REV1 style params are 8-bit
    READ_8(lo);
    return (ret << 8) | lo;
}

static uint32_t readReg40(uint16_t reg)
{
    uint16_t h, m, l;
    CS_ACTIVE;
    WriteCmd(reg);
    setReadDir();
    CD_DATA;
    h = read16bits();
    m = read16bits();
    l = read16bits();
    RD_IDLE;
    CS_IDLE;
    setWriteDir();
    return ((uint32_t)h << 24) | (m << 8) | (l >> 8);
}

uint16_t SPFD5408TFT::readReg(uint16_t reg)
{
    uint16_t ret;
    uint8_t lo;
    if (!done_reset)
        reset();
    CS_ACTIVE;
    WriteCmd(reg);
    setReadDir();
    CD_DATA;
    //    READ_16(ret);
    ret = read16bits();
    RD_IDLE;
    CS_IDLE;
    setWriteDir();
    return ret;
}

uint32_t SPFD5408TFT::readReg32(uint16_t reg)
{
    uint16_t h, l;
    CS_ACTIVE;
    WriteCmd(reg);
    setReadDir();
    CD_DATA;
    h = read16bits();
    l = read16bits();
    RD_IDLE;
    CS_IDLE;
    setWriteDir();
    return ((uint32_t)h << 16) | (l);
}

uint16_t SPFD5408TFT::readID(void)
{
    uint16_t ret, ret2;
    uint8_t msb;
    ret = readReg(0);           //forces a reset() if called before begin()
    if (ret == 0x5408)          //the SPFD5408 fails the 0xD3D3 test.
        return 0x5408;
    if (ret == 0x0089 || ret == 0x8989)
        return 0x1289;
    ret = readReg(0x67);        //HX8347-A
    if (ret == 0x4747)
        return 0x8347;
    ret = readReg32(0xA1);      //SSD1963: [01 57 61 01]
    if (ret == 0x6101)
        return 0x1963;
    ret = readReg40(0xBF);
    //HX8357B: [xx 01 62 83 57 FF] unsupported
    //R61581:  [xx 01 22 15 81]    unsupported
    return ret;

    if (ret == 0x9481)          //ILI9481: [xx 02 04 94 81 FF]
        return 0x9481;
    if (ret == 0x1511)          //?R61511: [xx 02 04 15 11] not tested yet
        return 0x1511;
    if (ret == 0x1520)          //?R61520: [xx 01 22 15 20]
        return 0x1520;
    if (ret == 0x1400)          //?RM68140:[xx FF 68 14 00] not tested yet
        return 0x6814;
    ret = readReg40(0xEF);      //ILI9327: [xx 02 04 93 27 FF] 

    if (ret == 0x9327)
        return 0x9327;
    ret = readReg32(0x04);      //ST7789V: [85 85 52] 
    if (ret == 0x8000)          //HX8357-D
        return 0x8357;
    if (ret == 0x8552)
        return 0x7789;
    ret = readReg32(0xD3);      //for ILI9488, 9486, 9340, 9341
    msb = ret >> 8;
    if (msb == 0x93 || msb == 0x94)
        return ret;             //0x9488, 9486, 9340, 9341
    if (ret == 0x00D3 || ret == 0xD3D3)
        return ret;             //16-bit write-only bus
/*
    msb = 0x12;                 //read 3rd,4th byte.  does not work in parallel
    pushCommand(0xD9, &msb, 1);
    ret2 = readReg(0xD3);
    msb = 0x13;
    pushCommand(0xD9, &msb, 1);
    ret = (ret2 << 8) | readReg(0xD3);
//	if (ret2 == 0x93)
        return ret2;
*/
/*  ret = readReg(0x93);//HX8340
  if (ret != 0 ) return ret;*/
    return readReg(0);          //0154, 7783, 9320, 9325, 9335, B505, B509
}

// independent cursor and window registers.   S6D0154, ST7781 increments.  ILI92320/5 do not.  
int16_t SPFD5408TFT::readGRAM(int16_t x, int16_t y, uint16_t* block, int16_t w, int16_t h)
{
    uint16_t ret, dummy, _MR = _MW;
    int16_t n = w * h, row = 0, col = 0;
    uint8_t r, g, b, tmp;
    if (_lcd_capable & MIPI_DCS_REV1)
        _MR = 0x2E;
    setAddrWindow(x, y, x + w - 1, y + h - 1);
    while (n > 0) {
        if (!(_lcd_capable & MIPI_DCS_REV1)) {
            WriteCmdData(_MC, x + col);
            WriteCmdData(_MP, y + row);
        }
        CS_ACTIVE;
        WriteCmd(_MR);
        setReadDir();
        CD_DATA;
        if (_lcd_capable & READ_NODUMMY) {
            ;
        }
        else if ((_lcd_capable & MIPI_DCS_REV1) || _lcd_ID == 0x1289) {
            READ_8(r);
        }
        else {
            READ_16(dummy);
        }
        if (_lcd_ID == 0x1511) READ_8(r);   //extra dummy for R61511
        while (n) {
            if (_lcd_capable & READ_24BITS) {
                READ_8(r);
                READ_8(g);
                READ_8(b);
                if (_lcd_capable & READ_BGR)
                    ret = color565(b, g, r);
                else
                    ret = color565(r, g, b);
            }
            else {
                READ_16(ret);
                if (_lcd_capable & READ_LOWHIGH)
                    ret = (ret >> 8) | (ret << 8);
                if (_lcd_capable & READ_BGR)
                    ret = (ret & 0x07E0) | (ret >> 11) | (ret << 11);
            }
            *block++ = ret;
            n--;
            if (!(_lcd_capable & AUTO_READINC))
                break;
        }
        if (++col >= w) {
            col = 0;
            if (++row >= h)
                row = 0;
        }
        RD_IDLE;
        CS_IDLE;
        setWriteDir();
    }
    if (!(_lcd_capable & MIPI_DCS_REV1))
        setAddrWindow(0, 0, width() - 1, height() - 1);
    return 0;
}

void SPFD5408TFT::setRotation(uint8_t r)
{
    uint16_t GS, SS, ORG, REV = _lcd_rev;
    uint8_t val, d[3];
    rotation = r & 3;           // just perform the operation ourselves on the protected variables
    _width = (rotation & 1) ? HEIGHT : WIDTH;
    _height = (rotation & 1) ? WIDTH : HEIGHT;
    switch (rotation) {
    case 0:                    //PORTRAIT:
        val = 0x48;             //MY=0, MX=1, MV=0, ML=0, BGR=1
        break;
    case 1:                    //LANDSCAPE: 90 degrees
        val = 0x28;             //MY=0, MX=0, MV=1, ML=0, BGR=1
        break;
    case 2:                    //PORTRAIT_REV: 180 degrees
        val = 0x88;             //MY=1, MX=0, MV=0, ML=1, BGR=1
        break;
    case 3:                    //LANDSCAPE_REV: 270 degrees
        val = 0xf8;             //MY=1, MX=1, MV=1, ML=1, BGR=1
        break;
    }
    if (_lcd_capable & INVERT_GS)
        val ^= 0x80;
    if (_lcd_capable & INVERT_SS)
        val ^= 0x40;
    if (_lcd_capable & INVERT_RGB)
        val ^= 0x08;
    if (_lcd_capable & MIPI_DCS_REV1) {
        if (_lcd_ID == 0x6814 || _lcd_ID == 0x9488) {  //.kbv my weird 0x9486 might be 68140
            GS = (val & 0x80) ? (1 << 6) : 0;   //MY
            SS = (val & 0x40) ? (1 << 5) : 0;   //MX
            val &= 0x28;        //keep MV, BGR, MY=0, MX=0, ML=0
            d[0] = 0;
            d[1] = GS | SS | 0x02;      //MY, MX
            d[2] = 0x3B;
            WriteCmdParamN(0xB6, 3, d);
            goto common_MC;
        }
        else if (_lcd_ID == 0x1963 || _lcd_ID == 0x9481 || _lcd_ID == 0x1511) {
            if (val & 0x80)
                val |= 0x01;    //GS
            if ((val & 0x40))
                val |= 0x02;    //SS
            if (_lcd_ID == 0x1963) val &= ~0xC0;
            if (_lcd_ID == 0x9481) val &= ~0xD0;
            if (_lcd_ID == 0x1511) {
                val &= ~0x10;   //remove ML
                val |= 0xC0;    //force penguin 180 rotation
            }
            //            val &= (_lcd_ID == 0x1963) ? ~0xC0 : ~0xD0; //MY=0, MX=0 with ML=0 for ILI9481
            goto common_MC;
        }
        else if (_lcd_ID == 0x1581) {
            if (val & 0x80)
                val |= 0x01;
            GS = (val & 0x80) ? (1 << 2) : 0;   //MY
           // SS = (val & 0x40) ? (1 << 0) : 0;   //MX
            d[0] = 0x12 | GS | SS;
            /*d[1] = 0x3B;
            d[2] = 0x00;
            d[3] = 0x00;
            d[4] = 0x00;
            d[5] = 0x01;
            d[6] = 0x00;
            d[7] = 0x43;*/
            WriteCmdParamN(0xc0, 1, d);
            goto common_MC;
        }
        else if (is8347) {
            _MC = 0x02, _MP = 0x06, _MW = 0x22, _SC = 0x02, _EC = 0x04, _SP = 0x06, _EP = 0x08;
            if (_lcd_ID == 0x5252) {
                val |= 0x02;   //VERT_SCROLLON
                if (val & 0x10) val |= 0x04;   //if (ML) SS=1 kludge mirror in XXX_REV modes
            }
            goto common_BGR;
        }
    common_MC:
        _MC = 0x2A, _MP = 0x2B, _MW = 0x2C, _SC = 0x2A, _EC = 0x2A, _SP = 0x2B, _EP = 0x2B;
    common_BGR:
        WriteCmdParamN(is8347 ? 0x16 : 0x36, 1, &val);
        _lcd_madctl = val;
        //	    if (_lcd_ID	== 0x1963) WriteCmdParamN(0x13, 0, NULL);   //NORMAL mode
    }
    // cope with 9320 variants
    else if (_lcd_ID == 0x65)//HX8352B
    {
        switch (rotation) {
        case 0:                    //PORTRAIT:
            val = 0x08;             //MY=0, MX=1, MV=0, ML=0, BGR=1
            break;
        case 1:                    //LANDSCAPE: 90 degrees
            val = 0x68;             //MY=0, MX=0, MV=1, ML=0, BGR=1
            break;
        case 2:                    //PORTRAIT_REV: 180 degrees
            val = 0xC8;             //MY=1, MX=0, MV=0, ML=1, BGR=1
            break;
        case 3:                    //LANDSCAPE_REV: 270 degrees
            val = 0xa8;             //MY=1, MX=1, MV=1, ML=1, BGR=1
            break;
        }
        _MW = 0x22;
        if (flag_write_bmp) WriteCmdData(0x16, val);
        else WriteCmdData(0x16, 0x08);
        _lcd_madctl = 0x08;
    }
    else {
        switch (_lcd_ID) {
#ifdef SUPPORT_0139
        case 0x0139:
            _SC = 0x46, _EC = 0x46, _SP = 0x48, _EP = 0x47;
            goto common_S6D;
#endif
        case 0x0154:
            _SC = 0x37, _EC = 0x36, _SP = 0x39, _EP = 0x38;
        common_S6D:
            _MC = 0x20, _MP = 0x21, _MW = 0x22;
            GS = (val & 0x80) ? (1 << 9) : 0;
            SS = (val & 0x40) ? (1 << 8) : 0;
            WriteCmdData(0x01, GS | SS | 0x0028);       // set Driver Output Control
            goto common_ORG;
        case 0x7793:
        case 0x9326:
        case 0xB509:
            _MC = 0x200, _MP = 0x201, _MW = 0x202, _SC = 0x210, _EC = 0x211, _SP = 0x212, _EP = 0x213;
            GS = (val & 0x80) ? (1 << 15) : 0;
            uint16_t NL;
            NL = ((HEIGHT / 8) - 1) << 9;
            if (_lcd_ID == 0x9326) NL >>= 1;
            //  WriteCmdData(0x400, GS | NL);
            uint16_t scan;
            if (GS == 0) scan = 0;
            else scan = 0x04;
            WriteCmdData(0x400, GS | NL | scan);
            goto common_SS;
        default:
            _MC = 0x20, _MP = 0x21, _MW = 0x22, _SC = 0x50, _EC = 0x51, _SP = 0x52, _EP = 0x53;
            GS = (val & 0x80) ? (1 << 15) : 0;
            WriteCmdData(0x60, GS | 0x2700);    // Gate Scan Line (0xA700)
        common_SS:
            SS = (val & 0x40) ? (1 << 8) : 0;
            //SS = (val&0x05)<<8;
            WriteCmdData(0x01, SS);     // set Driver Output Control
        common_ORG:
            ORG = (val & 0x20) ? (1 << 3) : 0;
            if (val & 0x08)
                ORG |= 0x1000;  //BGR
            _lcd_madctl = ORG | 0x0030;
            WriteCmdData(0x03, _lcd_madctl);    // set GRAM write direction and BGR=1.
            break;
#ifdef SUPPORT_1289
        case 0x1289:
            _MC = 0x4E, _MP = 0x4F, _MW = 0x22, _SC = 0x44, _EC = 0x44, _SP = 0x45, _EP = 0x46;
            if (rotation & 1)
                val ^= 0xD0;    // exchange Landscape modes
            GS = (val & 0x80) ? (1 << 14) | (1 << 12) : 0;      //called TB (top-bottom)
            SS = (val & 0x40) ? (1 << 9) : 0;   //called RL (right-left)
            ORG = (val & 0x20) ? (1 << 3) : 0;  //called AM
            _lcd_drivOut = GS | SS | (REV << 13) | 0x013F;      //REV=0, BGR=0, MUX=319
            if (val & 0x08)
                _lcd_drivOut |= 0x0800; //BGR
            WriteCmdData(0x01, _lcd_drivOut);   // set Driver Output Control
            WriteCmdData(0x11, ORG | 0x6070);   // set GRAM write direction.
            break;
#endif
        }
    }
    if ((rotation & 1) && ((_lcd_capable & MV_AXIS) == 0)) {
        uint16_t x;
        x = _MC, _MC = _MP, _MP = x;
        x = _SC, _SC = _SP, _SP = x;    //.kbv check 0139
        x = _EC, _EC = _EP, _EP = x;    //.kbv check 0139
    }
    setAddrWindow(0, 0, width() - 1, height() - 1);
    vertScroll(0, HEIGHT, 0);   //reset scrolling after a rotation
}

void SPFD5408TFT::drawPixel(int16_t x, int16_t y, uint16_t color)
{
#ifdef ILI9327_SPECIAL
    color = ~color;
#endif
    // MCUFRIEND just plots at edge if you try to write outside of the box:
    if (x < 0 || y < 0 || x >= width() || y >= height())
        return;
#if defined(OFFSET_9327)
    if (_lcd_ID == 0x9327) {
        if (rotation == 2) y += OFFSET_9327;
        if (rotation == 3) x += OFFSET_9327;
    }
#endif
    if (_lcd_capable & MIPI_DCS_REV1) {
        WriteCmdParam4(_MC, x >> 8, x, x >> 8, x);
        WriteCmdParam4(_MP, y >> 8, y, y >> 8, y);
    }
    else if (_lcd_ID == 0x65)//HX8352B
    {
        if (!flag_write_bmp) {
            int16_t t;
            switch (rotation) {
            case 1:
                t = x;
                x = WIDTH - 1 - y;
                y = t;
                break;
            case 2:
                x = WIDTH - 1 - x;
                y = HEIGHT - 1 - y;
                break;
            case 3:
                t = x;
                x = y;
                y = HEIGHT - 1 - t;
                break;
            }
        }
        WriteCmdData(0x80, x >> 8);
        WriteCmdData(0x81, x);
        WriteCmdData(0x82, y >> 8);
        WriteCmdData(0x83, y);

    }
    else {
        WriteCmdData(_MC, x);
        WriteCmdData(_MP, y);
    }
    WriteCmdData(_MW, color);
}

void SPFD5408TFT::setAddrWindow(int16_t x, int16_t y, int16_t x1, int16_t y1)
{
#if defined(OFFSET_9327)
    if (_lcd_ID == 0x9327) {
        if (rotation == 2) y += OFFSET_9327, y1 += OFFSET_9327;
        if (rotation == 3) x += OFFSET_9327, x1 += OFFSET_9327;
    }
#endif
    if (_lcd_capable & MIPI_DCS_REV1) {
        WriteCmdParam4(_MC, x >> 8, x, x1 >> 8, x1);
        WriteCmdParam4(_MP, y >> 8, y, y1 >> 8, y1);
    }
    else if (_lcd_ID == 0x65)//HX8352B
    {
        if (!flag_write_bmp) {
            int x0, y0, t;
            switch (rotation) {
            default:
                x0 = x;
                y0 = y;
                break;
            case 1:
                t = y;
                y = x;
                x = WIDTH - 1 - y1;
                y1 = x1;
                x1 = WIDTH - 1 - t;
                x0 = x1;
                y0 = y;
                break;
            case 2:
                t = x;
                x = WIDTH - 1 - x1;
                x1 = WIDTH - 1 - t;
                t = y;
                y = HEIGHT - 1 - y1;
                y1 = HEIGHT - 1 - t;
                x0 = x1;
                y0 = y1;
                break;
            case 3:
                t = x;
                x = y;
                y = HEIGHT - 1 - x1;
                x1 = y1;
                y1 = HEIGHT - 1 - t;
                x0 = x;
                y0 = y1;
                break;
            }
        }
        WriteCmdData(0x02, x >> 8);
        WriteCmdData(0x03, x);
        WriteCmdData(0x04, x1 >> 8);
        WriteCmdData(0x05, x1);
        WriteCmdData(0x06, y >> 8);
        WriteCmdData(0x07, y);
        WriteCmdData(0x08, y1 >> 8);
        WriteCmdData(0x09, y1);
        WriteCmdData(0x80, x >> 8);
        WriteCmdData(0x81, x);
        WriteCmdData(0x82, y >> 8);
        WriteCmdData(0x83, y);
        /* WriteCmdData(0x80,x0>>8);
         WriteCmdData(0x81,x0);
         WriteCmdData(0x82,y0>>8);
         WriteCmdData(0x83,y0);*/

    }
    else {
        WriteCmdData(_MC, x);
        WriteCmdData(_MP, y);
        if (_lcd_capable & XSA_XEA_16BIT) {
            if (rotation & 1)
                y1 = y = (y1 << 8) | y;
            else
                x1 = x = (x1 << 8) | x;
        }
        WriteCmdData(_SC, x);
        WriteCmdData(_SP, y);
        WriteCmdData(_EC, x1);
        WriteCmdData(_EP, y1);
    }
}

void SPFD5408TFT::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
#ifdef ILI9327_SPECIAL
    color = ~color;
#endif
    int16_t end;
    if (w < 0) {
        w = -w;
        x -= w;
    }                           //+ve w
    end = x + w;
    if (x < 0)
        x = 0;
    if (end > width())
        end = width();
    w = end - x;
    if (h < 0) {
        h = -h;
        y -= h;
    }                           //+ve h
    end = y + h;
    if (y < 0)
        y = 0;
    if (end > height())
        end = height();
    h = end - y;

    setAddrWindow(x, y, x + w - 1, y + h - 1);
    CS_ACTIVE;
    WriteCmd(_MW);
    if (h > w) {
        end = h;
        h = w;
        w = end;
    }
    uint8_t hi = color >> 8, lo = color & 0xFF;
    CD_DATA;
    while (h-- > 0) {
        end = w;
#if USING_16BIT_BUS
#if defined(__SAM3X8E__)
#define STROBE_16BIT {WR_ACTIVE;WR_ACTIVE;WR_ACTIVE;WR_IDLE;WR_IDLE;}
#else
#define STROBE_16BIT {WR_ACTIVE; WR_IDLE;}
#endif
        write_16(color);        //we could just do the strobe
        lo = end & 7;
        hi = end >> 3;
        if (hi)
            do {
                STROBE_16BIT;
                STROBE_16BIT;
                STROBE_16BIT;
                STROBE_16BIT;
                STROBE_16BIT;
                STROBE_16BIT;
                STROBE_16BIT;
                STROBE_16BIT;
            } while (--hi > 0);
            while (lo-- > 0) {
                STROBE_16BIT;
            }
#else
        do {
            write8(hi);
            write8(lo);
        } while (--end != 0);
#endif
    }
    CS_IDLE;
    if (!(_lcd_capable & MIPI_DCS_REV1))
        setAddrWindow(0, 0, width() - 1, height() - 1);
}

void SPFD5408TFT::startWrite()
{
    WriteCmd(_MW);
}
void SPFD5408TFT::pushColors(uint16_t color)
{
    CS_ACTIVE;
    CD_DATA;
    write16(color);
    CS_IDLE;
}
void SPFD5408TFT::pushColors(uint16_t* block, int16_t n, bool first)
{
    uint16_t color;
    //delay();//
    CS_ACTIVE;
    if (first) {
        WriteCmd(_MW);
    }
    CD_DATA;
    while (n-- > 0) {
        color = *block++;
#ifdef ILI9327_SPECIAL
        color = ~color;
#endif
        write16(color);
    }
    CS_IDLE;
}

void SPFD5408TFT::pushColors(uint8_t* block, int16_t n, bool first)
{
    uint16_t color;
    uint8_t h, l;
    CS_ACTIVE;
    if (first) {
        WriteCmd(_MW);
    }
    CD_DATA;
    while (n-- > 0) {
        h = (*block++);
        l = (*block++);
        color = h << 8 | l;
#ifdef ILI9327_SPECIAL
        color = ~color;
#endif
        write16(color);
    }
    CS_IDLE;
}

void SPFD5408TFT::pushColors(const uint8_t* block, int16_t n, bool first)
{
    uint16_t color;
    uint8_t h, l;
    CS_ACTIVE;
    if (first) {
        WriteCmd(_MW);
    }
    CD_DATA;
    while (n-- > 0) {
        l = pgm_read_byte(block++);
        h = pgm_read_byte(block++);
        color = h << 8 | l;
#ifdef ILI9327_SPECIAL
        color = ~color;
#endif
        write16(color);
    }
    CS_IDLE;
}

void SPFD5408TFT::vertScroll(int16_t top, int16_t scrollines, int16_t offset)
{
#if defined(OFFSET_9327)
    if (_lcd_ID == 0x9327) {
        if (rotation == 2 || rotation == 3) top += OFFSET_9327;
    }
#endif
    int16_t bfa = HEIGHT - top - scrollines;  // bottom fixed area
    int16_t vsp;
    int16_t sea = top;
    if (_lcd_ID == 0x9327) bfa += 32;
    if (offset <= -scrollines || offset >= scrollines) offset = 0; //valid scroll
    vsp = top + offset; // vertical start position
    if (offset < 0)
        vsp += scrollines;          //keep in unsigned range
    sea = top + scrollines - 1;
    if (_lcd_capable & MIPI_DCS_REV1) {
        uint8_t d[6];           // for multi-byte parameters
/*
        if (_lcd_ID == 0x9327) {        //panel is wired for 240x432
            if (rotation == 2 || rotation == 3) { //180 or 270 degrees
                if (scrollines == HEIGHT) {
                    scrollines = 432;   // we get a glitch but hey-ho
                    vsp -= 432 - HEIGHT;
                }
                if (vsp < 0)
                    vsp += 432;
            }
            bfa = 432 - top - scrollines;
        }
*/
        d[0] = top >> 8;        //TFA
        d[1] = top;
        d[2] = scrollines >> 8; //VSA
        d[3] = scrollines;
        d[4] = bfa >> 8;        //BFA
        d[5] = bfa;
        WriteCmdParamN(is8347 ? 0x0E : 0x33, 6, d);
        //        if (offset == 0 && rotation > 1) vsp = top + scrollines;   //make non-valid
        d[0] = vsp >> 8;        //VSP
        d[1] = vsp;
        WriteCmdParamN(is8347 ? 0x14 : 0x37, 2, d);
        if (is8347) {
            d[0] = (offset != 0) ? (_lcd_ID == 0x8347 ? 0x02 : 0x08) : 0;
            WriteCmdParamN(_lcd_ID == 0x8347 ? 0x18 : 0x01, 1, d);  //HX8347-D
        }
        else if (offset == 0 && (_lcd_capable & MIPI_DCS_REV1)) {
            WriteCmdParamN(0x13, 0, NULL);    //NORMAL i.e. disable scroll
        }
        return;
    }
    // cope with 9320 style variants:
    switch (_lcd_ID) {
    case 0x7783:
        WriteCmdData(0x61, _lcd_rev);   //!NDL, !VLE, REV
        WriteCmdData(0x6A, vsp);        //VL#
        break;
#ifdef SUPPORT_0139
    case 0x0139:
        WriteCmdData(0x41, sea);        //SEA
        WriteCmdData(0x42, top);        //SSA
        WriteCmdData(0x43, vsp - top);  //SST
        break;
#endif
    case 0x0154:
        WriteCmdData(0x31, sea);        //SEA
        WriteCmdData(0x32, top);        //SSA
        WriteCmdData(0x33, vsp - top);  //SST
        break;
#ifdef SUPPORT_1289
    case 0x1289:
        WriteCmdData(0x41, vsp);        //VL#
        break;
#endif
    case 0x7793:
    case 0x9326:
    case 0xB509:
        WriteCmdData(0x401, (1 << 1) | _lcd_rev);       //VLE, REV 
        WriteCmdData(0x404, vsp);       //VL# 
        break;
    default:
        // 0x6809, 0x9320, 0x9325, 0x9335, 0xB505 can only scroll whole screen
        WriteCmdData(0x61, (1 << 1) | _lcd_rev);        //!NDL, VLE, REV
        WriteCmdData(0x6A, vsp);        //VL#
        break;
    }
}

void SPFD5408TFT::invertDisplay(boolean i)
{
    uint8_t val;
    _lcd_rev = ((_lcd_capable & REV_SCREEN) != 0) ^ i;
    if (_lcd_capable & MIPI_DCS_REV1) {
        if (is8347) {
            // HX8347D: 0x36 Panel Characteristic. REV_Panel
            // HX8347A: 0x36 is Display Control 10
            if (_lcd_ID == 0x8347 || _lcd_ID == 0x5252) // HX8347-A, HX5352-A
                val = _lcd_rev ? 6 : 2;       //INVON id bit#2,  NORON=bit#1
            else val = _lcd_rev ? 8 : 10;     //HX8347-D, G, I: SCROLLON=bit3, INVON=bit1
            // HX8347: 0x01 Display Mode has diff bit mapping for A, D 
            WriteCmdParamN(0x01, 1, &val);
        }
        else
            WriteCmdParamN(_lcd_rev ? 0x21 : 0x20, 0, NULL);
        return;
    }
    // cope with 9320 style variants:
    switch (_lcd_ID) {
#ifdef SUPPORT_0139
    case 0x0139:
#endif
    case 0x0154:
        WriteCmdData(0x07, 0x13 | (_lcd_rev << 2));     //.kbv kludge
        break;
#ifdef SUPPORT_1289
    case 0x1289:
        _lcd_drivOut &= ~(1 << 13);
        if (_lcd_rev)
            _lcd_drivOut |= (1 << 13);
        WriteCmdData(0x01, _lcd_drivOut);
        break;
#endif
    case 0x7793:
    case 0x9326:
    case 0xB509:
        WriteCmdData(0x401, (1 << 1) | _lcd_rev);       //.kbv kludge VLE 
        break;
    default:
        WriteCmdData(0x61, _lcd_rev);
        break;
    }
}

#define TFTLCD_DELAY 0xFFFF
#define TFTLCD_DELAY8 0xFF
static void init_table(const void* table, int16_t size)
{
    uint8_t* p = (uint8_t*)table, dat[16];
    while (size > 0) {
        uint8_t cmd = pgm_read_byte(p++);
        uint8_t len = pgm_read_byte(p++);
        if (cmd == TFTLCD_DELAY8) {
            delay(len);
            len = 0;
        }
        else {
            for (uint8_t i = 0; i < len; i++)
                dat[i] = pgm_read_byte(p++);
            WriteCmdParamN(cmd, len, dat);
        }
        size -= len + 2;
    }
}

static void init_table16(const void* table, int16_t size)
{
    uint16_t* p = (uint16_t*)table;
    while (size > 0) {
        uint16_t cmd = pgm_read_word(p++);
        uint16_t d = pgm_read_word(p++);
        if (cmd == TFTLCD_DELAY)
            delay(d);
        else {
            CS_ACTIVE;
            WriteCmd(cmd);
            WriteData(d);
            CS_IDLE;
        }
        size -= 2 * sizeof(int16_t);
    }
}

void SPFD5408TFT::begin(uint16_t ID)
{
    reset();
    _lcd_xor = 0;
    _lcd_ID = ID;
    switch (_lcd_ID) {

#ifdef SUPPORT_9320
    case 0x5408:
        _lcd_capable = 0 | REV_SCREEN | READ_BGR | INVERT_GS;
        goto common_9320;
    case 0x9320:
        _lcd_capable = 0 | REV_SCREEN | READ_BGR;
    common_9320:
        static const uint16_t ILI9320_regValues[] PROGMEM = {
            0x00e5, 0x8000,
            0x0000, 0x0001,
            0x0001, 0x100,
            0x0002, 0x0700,
            0x0003, 0x1030,
            0x0004, 0x0000,
            0x0008, 0x0202,
            0x0009, 0x0000,
            0x000A, 0x0000,
            0x000C, 0x0000,
            0x000D, 0x0000,
            0x000F, 0x0000,
            //-----Power On sequence-----------------------
            0x0010, 0x0000,
            0x0011, 0x0007,
            0x0012, 0x0000,
            0x0013, 0x0000,
            TFTLCD_DELAY, 50,
            0x0010, 0x17B0,
            0x0011, 0x0007,
            TFTLCD_DELAY, 10,
            0x0012, 0x013A,
            TFTLCD_DELAY, 10,
            0x0013, 0x1A00,
            0x0029, 0x000c,
            TFTLCD_DELAY, 10,
            //-----Gamma control-----------------------
            0x0030, 0x0000,
            0x0031, 0x0505,
            0x0032, 0x0004,
            0x0035, 0x0006,
            0x0036, 0x0707,
            0x0037, 0x0105,
            0x0038, 0x0002,
            0x0039, 0x0707,
            0x003C, 0x0704,
            0x003D, 0x0807,
            //-----Set RAM area-----------------------
            0x0050, 0x0000,
            0x0051, 0x00EF,
            0x0052, 0x0000,
            0x0053, 0x013F,
            0x0060, 0xA700,     //GS=1
            0x0061, 0x0001,
            0x006A, 0x0000,
            0x0021, 0x0000,
            0x0020, 0x0000,
            //-----Partial Display Control------------
            0x0080, 0x0000,
            0x0081, 0x0000,
            0x0082, 0x0000,
            0x0083, 0x0000,
            0x0084, 0x0000,
            0x0085, 0x0000,
            //-----Panel Control----------------------
            0x0090, 0x0010,
            0x0092, 0x0000,
            0x0093, 0x0003,
            0x0095, 0x0110,
            0x0097, 0x0000,
            0x0098, 0x0000,
            //-----Display on-----------------------
            0x0007, 0x0173,
            TFTLCD_DELAY, 50,
        };
        init_table16(ILI9320_regValues, sizeof(ILI9320_regValues));
        break;
#endif
        

    }
    _lcd_rev = ((_lcd_capable & REV_SCREEN) != 0);
    setRotation(0);             //PORTRAIT
    invertDisplay(false);
}
