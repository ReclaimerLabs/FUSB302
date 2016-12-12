/*
  FUSB302.c - Library for interacting with the FUSB302B chip.
  Created by Jason Cerundolo, December 11, 2016.
  Released under an MIT license. See LICENSE.md. 
*/

#include "Arduino.h"
#include <Wire.h>
#include "usb-c.h"

USB_C::USB_C()
{
	Wire.begin();
}

int USB_C::tcpc_write(int port, int reg, int val) {
	if (port == 0) {
		Wire.beginTransmission(FUSB302_I2C_SLAVE_ADDR);
		Wire.write(reg & 0xFF);
		Wire.write(val & 0xFF);
		Wire.endTransmission();
	}
	
	return 0;
}

int USB_C::tcpc_read(int port, int reg, int *val) {
	int read_value;
	if (port == 0) {
		Wire.beginTransmission(FUSB302_I2C_SLAVE_ADDR);
		Wire.write(reg & 0xFF);
		Wire.endTransmission(false);
		read_value = Wire.requestFrom(FUSB302_I2C_SLAVE_ADDR, 1, true);
	}
	
	return read_value;
}