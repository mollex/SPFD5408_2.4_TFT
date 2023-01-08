/*
    LM75 - An arduino library for the LM75 temperature sensor
    Copyright (C) 2011  Dan Fekete <thefekete AT gmail DOT com>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <Wire.h>
#include "SPFD5408Lm75.h"

SPFD5408Lm75::SPFD5408Lm75() {
    address = LM75_ADDRESS;
}

SPFD5408Lm75::SPFD5408Lm75(byte addr) {
    address = addr;
}

word SPFD5408Lm75::float2regdata(float temp)
{
    // First multiply by 8 and coerce to integer to get +/- whole numbers
    // Then coerce to word and bitshift 5 to fill out MSB
    return (word)((int)(temp * 8) << 5);
}

float SPFD5408Lm75::regdata2float(word regdata)
{
    return ((float)(int)regdata / 32) / 8;
}

word SPFD5408Lm75::_register16(byte reg) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.requestFrom(address, 2);
    word regdata = (Wire.read() << 8) | Wire.read();
    return regdata;
}

void SPFD5408Lm75::_register16(byte reg, word regdata) {
    byte msb = (byte)(regdata >> 8);
    byte lsb = (byte)(regdata);

    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(msb);
    Wire.write(lsb);
    Wire.endTransmission();
}

word SPFD5408Lm75::_register8(byte reg) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.requestFrom(address, 1);
    return Wire.read();
}

void SPFD5408Lm75::_register8(byte reg, byte regdata) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(regdata);
    Wire.endTransmission();
}

float SPFD5408Lm75::temp(void) {
    return regdata2float(_register16(LM75_TEMP_REGISTER));
}

byte SPFD5408Lm75::conf() {
    return _register8(LM75_CONF_REGISTER);
}

void SPFD5408Lm75::conf(byte data) {
    _register8(LM75_CONF_REGISTER, data);
}

float SPFD5408Lm75::tos() {
    return regdata2float(_register16(LM75_TOS_REGISTER));
}

void SPFD5408Lm75::tos(float temp) {
    _register16(LM75_TOS_REGISTER, float2regdata(temp));
}

float SPFD5408Lm75::thyst() {
    return regdata2float(_register16(LM75_THYST_REGISTER));
}

void SPFD5408Lm75::thyst(float temp) {
    _register16(LM75_THYST_REGISTER, float2regdata(temp));
}

boolean SPFD5408Lm75::shutdown() {
    return conf() & 0x01;
}

void SPFD5408Lm75::shutdown(boolean val) {
    conf(val << LM75_CONF_SHUTDOWN);
}
