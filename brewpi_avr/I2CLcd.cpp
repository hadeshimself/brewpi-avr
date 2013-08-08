/*
 * Copyright 2012 BrewPi/Elco Jacobs.
 * Copyright 2013 Luis Balbinot
 *
 * Portions of code extracted from LiquidCrystal_I2C.cpp
 * implemented by Francisco Malpartida.
 * https://bitbucket.org/fmalpartida/new-liquidcrystal/
 *
 * This file is part of BrewPi.
 *
 * BrewPi is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * BrewPi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with BrewPi.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "I2CLcd.h"

#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

void I2CLcd::init() {
	_Addr = I2C_ADDR;
	_backlightPinMask = 0;
	_backlightStsMask = LCD_BACKLIGHT;
	_polarity = NEGATIVE;

	_En = (1 << I2C_EN);
	_Rw = (1 << I2C_RW);
	_Rs = (1 << I2C_RS);

	// Initialise pin mapping
	_data_pins[0] = (1 << I2C_D4);
	_data_pins[1] = (1 << I2C_D5);
	_data_pins[2] = (1 << I2C_D6);
	_data_pins[3] = (1 << I2C_D7);

	// initialize the backpack IO expander
	// and display functions.
	// ------------------------------------------------------------------------
	if (_i2cio.begin (_Addr) == 1) {
		_i2cio.portMode ( OUTPUT );  // Set the entire IO extender to OUTPUT
		_displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
		_i2cio.write(0);  // Set the entire port to LOW
	}

	setBacklightPin(I2C_BLPIN, POSITIVE);
}

void I2CLcd::begin(uint8_t cols, uint8_t lines) {
	_numlines = lines;
	_currline = 0;
	_currpos = 0;
  
	LCD::begin(cols, lines);
	LCD::noDisplay();
	clear();
	LCD::leftToRight();
	LCD::noAutoscroll();
	home();
	LCD::noCursor();
	LCD::display();
}

void I2CLcd::clear() {
	LCD::clear();
	
	for(uint8_t i = 0; i<4; i++) {
		for(uint8_t j = 0; j<20; j++) {
			content[i][j] = ' '; // initialize on all spaces
		}
		content[i][20] = '\0'; // NULL terminate string
	}	
}

void I2CLcd::home() {
	LCD::setCursor(0, 0);
	_currline = 0;
	_currpos = 0;
}

void I2CLcd::setCursor(uint8_t col, uint8_t row) {
	if (row >= _numlines) {
		row = 0;  // write to first line if out off bounds
	}
	_currline = row;
	_currpos = col;
	LCD::setCursor(col, row);
}

inline size_t I2CLcd::write(uint8_t value) {
	send(value, HIGH);
	content[_currline][_currpos] = value;
	_currpos++;
	return 1;
}

void I2CLcd::send(uint8_t value, uint8_t mode) {
	if ( mode == FOUR_BITS ) {
		write4bits((value & 0x0F), COMMAND);
	}
	else {
		write4bits( (value >> 4), mode);
		write4bits( (value & 0x0F), mode);
	}
}

void I2CLcd::pulseEnable(uint8_t data) {
	_i2cio.write (data | _En);   // En HIGH
	_i2cio.write (data & ~_En);  // En LOW
}

void I2CLcd::write4bits(uint8_t value, uint8_t mode) {
	uint8_t pinMapValue = 0;

	// Map the value to LCD pin mapping
	// --------------------------------
	for (uint8_t i = 0; i<4; i++) {
		if ((value & 0x1) == 1) {
			pinMapValue |= _data_pins[i];
		}
		value = (value >> 1);
	}

	// Is it a command or data
	// -----------------------
	if ( mode == DATA ) {
		mode = _Rs;
	}

	pinMapValue |= mode | _backlightStsMask;
	pulseEnable (pinMapValue);
}

void I2CLcd::getLine(uint8_t lineNumber, char * buffer) {
	const char* src = content[lineNumber];
	for(uint8_t i = 0; i<20; i++){
		char c = src[i];
		buffer[i] = (c == 0b11011111) ? 0xB0 : c;
	}
	buffer[20] = '\0'; // NULL terminate string
}	

void I2CLcd::printSpacesToRestOfLine(void) {
	while(_currpos < 20) {
		print(' ');
	}
}

void I2CLcd::setBacklightPin (uint8_t value, t_backlighPol pol = POSITIVE) {
	_backlightPinMask = (1 << value);
	_polarity = pol;
	setBacklight(BACKLIGHT_OFF);
}

void I2CLcd::setBacklight(uint8_t value) {
	// Check if backlight is available
	// ----------------------------------------------------
	if (_backlightPinMask != 0x0)
	{
		// Check for polarity to configure mask accordingly
		// ----------------------------------------------------------
		if (((_polarity == POSITIVE) && (value > 0)) ||
		  ((_polarity == NEGATIVE) && (value == 0))) {
			_backlightStsMask = _backlightPinMask & LCD_BACKLIGHT;
		}
		else {
			_backlightStsMask = _backlightPinMask & LCD_NOBACKLIGHT;
		}
		_i2cio.write(_backlightStsMask);
	}
}

