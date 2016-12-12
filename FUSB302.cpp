/*
  FUSB302.c - Library for interacting with the FUSB302B chip.
  Created by Jason Cerundolo, December 11, 2016.
  Released under an MIT license. See LICENSE.md. 
*/

#include "Arduino.h"
#include <Wire.h>
#include "FUSB302.h"

struct fusb302_chip_state {
	int cc_polarity;
	int vconn_enabled;
	/* 1 = pulling up (DFP) 0 = pulling down (UFP) */
	int pulling_up;
	int rx_enable;
	int dfp_toggling_on;
	int previous_pull;
	int togdone_pullup_cc1;
	int togdone_pullup_cc2;
	int tx_hard_reset_req;
	int set_cc_lock;
	uint8_t mdac_vnc;
	uint8_t mdac_rd;
} state;

FUSB302::FUSB302()
{
	Wire.begin();
}

/* bring the FUSB302 out of reset after Hard Reset signaling */
void FUSB302::pd_reset()
{
	this->tcpc_write(TCPC_REG_RESET, TCPC_REG_RESET_PD_RESET);
}

void FUSB302::flush_rx_fifo()
{
	/*
	 * other bits in the register _should_ be 0
	 * until the day we support other SOP* types...
	 * then we'll have to keep a shadow of what this register
	 * value should be so we don't clobber it here!
	 */
	this->tcpc_write(TCPC_REG_CONTROL1, TCPC_REG_CONTROL1_RX_FLUSH);
}

int FUSB302::tcpc_write(int reg, int val) {
	Wire.beginTransmission(FUSB302_I2C_SLAVE_ADDR);
	Wire.write(reg & 0xFF);
	Wire.write(val & 0xFF);
	Wire.endTransmission();
	
	return 0;
}

int FUSB302::tcpc_read(int reg) {
	Wire.beginTransmission(FUSB302_I2C_SLAVE_ADDR);
	Wire.write(reg & 0xFF);
	Wire.endTransmission(false);
	Wire.requestFrom(FUSB302_I2C_SLAVE_ADDR, 1, true);
  return Wire.read();
}