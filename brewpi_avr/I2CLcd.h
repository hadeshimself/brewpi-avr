/*
 * Copyright 2012 BrewPi/Elco Jacobs.
 * Copyright 2013 Luis Balbinot
 *
 * Portions of code extracted from LiquidCrystal_I2C.h
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

#ifndef I2CLcd_h
#define I2CLcd_h

#include <inttypes.h>
#include "Print.h"
#include "Pins.h"
#include "I2CIO.h"
#include "LCD.h"

#define LCD_NOBACKLIGHT 0x00
#define LCD_BACKLIGHT   0xFF

#define I2C_ADDR	0x27	// Address
#define I2C_EN		2	// Enable bit
#define I2C_RW		1	// Read/Write bit
#define I2C_RS		0	// Register select bit
#define I2C_D4		4
#define I2C_D5		5
#define I2C_D6		6
#define I2C_D7		7
#define I2C_BLPIN	3

class I2CLcd : public LCD {
	public:
	I2CLcd(){};

	void init();
	void begin(uint8_t cols, uint8_t rows);

	void clear();
	void home();

	void createChar(uint8_t, uint8_t[]);
	void setCursor(uint8_t, uint8_t);

	virtual size_t write(uint8_t);

	size_t print_P(const char * str) { // print a string stored in PROGMEM
		char buf[21]; // create buffer in RAM
		strlcpy_P(buf, str, 20); // copy string to RAM
		return print(buf); // print from RAM
	}
	
	// copy a line from the shadow copy to a string buffer and correct the degree sign
	void getLine(uint8_t lineNumber, char * buffer); 
	void setBufferOnly(bool bufferOnly) {}
	void printSpacesToRestOfLine();
	void setBacklightPin ( uint8_t value, t_backlighPol pol );
	void setBacklight ( uint8_t value );
	void resetBacklightTimer(void){ /* not implemented for OLED, doesn't have a backlight. */ }
	void updateBacklight(void){ /* not implemented for OLED, doesn't have a backlight. */ }

	using Print::write;

	private:
	void send(uint8_t, uint8_t);
	void write4bits(uint8_t, uint8_t);
	void pulseEnable(uint8_t);

	uint8_t _Addr;             // I2C Address of the IO expander
	uint8_t _backlightPinMask; // Backlight IO pin mask
	uint8_t _backlightStsMask; // Backlight status mask
	I2CIO   _i2cio;            // I2CIO PCF8574* expansion module driver I2CLCDextraIO
	uint8_t _En;               // LCD expander word for enable pin
	uint8_t _Rw;               // LCD expander word for R/W pin
	uint8_t _Rs;               // LCD expander word for Register Select pin
	uint8_t _data_pins[4];     // LCD data lines

	uint8_t _displayfunction;
	uint8_t _displaycontrol;
	uint8_t _displaymode;
	uint8_t _initialized;
	uint8_t _currline;
	uint8_t _currpos;
	uint8_t _numlines;
	
	char content[4][21]; // always keep a copy of the display content in this variable
	
	bool	_bufferOnly;
};

#endif
