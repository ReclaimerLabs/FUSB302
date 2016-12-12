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
  state.vconn_enabled = 0;
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

void FUSB302::flush_tx_fifo()
{
	int reg;

	reg = this->tcpc_read(TCPC_REG_CONTROL0);
	reg |= TCPC_REG_CONTROL0_TX_FLUSH;
	this->tcpc_write(TCPC_REG_CONTROL0, reg);
}

/* Convert BC LVL values (in FUSB302) to Type-C CC Voltage Status */
int FUSB302::convert_bc_lvl(int bc_lvl)
{
	/* assume OPEN unless one of the following conditions is true... */
	int ret = TYPEC_CC_VOLT_OPEN;

	if (state.pulling_up) {
		if (bc_lvl == 0x00)
			ret = TYPEC_CC_VOLT_RA;
		else if (bc_lvl < 0x3)
			ret = TYPEC_CC_VOLT_RD;
	} else {
		if (bc_lvl == 0x1)
			ret = TYPEC_CC_VOLT_SNK_DEF;
		else if (bc_lvl == 0x2)
			ret = TYPEC_CC_VOLT_SNK_1_5;
		else if (bc_lvl == 0x3)
			ret = TYPEC_CC_VOLT_SNK_3_0;
	}

	return ret;
}

int FUSB302::measure_cc_pin_source(int cc_measure)
{
	int switches0_reg;
	int reg;
	int cc_lvl;

	/* Read status register */
	reg = this->tcpc_read(TCPC_REG_SWITCHES0);
	/* Save current value */
	switches0_reg = reg;
	/* Clear pull-up register settings and measure bits */
	reg &= ~(TCPC_REG_SWITCHES0_MEAS_CC1 | TCPC_REG_SWITCHES0_MEAS_CC2);
	/* Set desired pullup register bit */
	if (cc_measure == TCPC_REG_SWITCHES0_MEAS_CC1)
		reg |= TCPC_REG_SWITCHES0_CC1_PU_EN;
	else
		reg |= TCPC_REG_SWITCHES0_CC2_PU_EN;
	/* Set CC measure bit */
	reg |= cc_measure;

	/* Set measurement switch */
	this->tcpc_write(TCPC_REG_SWITCHES0, reg);

	/* Set MDAC for Open vs Rd/Ra comparison */
  this->tcpc_write(TCPC_REG_MEASURE, state.mdac_vnc);

	/* Wait on measurement */
	delayMicroseconds(250);

	/* Read status register */
	reg = this->tcpc_read(TCPC_REG_STATUS0);

	/* Assume open */
	cc_lvl = TYPEC_CC_VOLT_OPEN;

	/* CC level is below the 'no connect' threshold (vOpen) */
	if ((reg & TCPC_REG_STATUS0_COMP) == 0) {
		/* Set MDAC for Rd vs Ra comparison */
		this->tcpc_write(TCPC_REG_MEASURE, state.mdac_rd);

		/* Wait on measurement */
		delayMicroseconds(250);

		/* Read status register */
		reg = this->tcpc_read(TCPC_REG_STATUS0);

		cc_lvl = (reg & TCPC_REG_STATUS0_COMP) ? TYPEC_CC_VOLT_RD
						       : TYPEC_CC_VOLT_RA;
	}

	/* Restore SWITCHES0 register to its value prior */
	this->tcpc_write(TCPC_REG_SWITCHES0, switches0_reg);

	return cc_lvl;
}

/* Determine cc pin state for source when in manual detect mode */
void FUSB302::detect_cc_pin_source_manual(int *cc1_lvl, int *cc2_lvl)
{
	int cc1_measure = TCPC_REG_SWITCHES0_MEAS_CC1;
	int cc2_measure = TCPC_REG_SWITCHES0_MEAS_CC2;

	if (state.vconn_enabled) {
		/* If VCONN enabled, measure cc_pin that matches polarity */
		if (state.cc_polarity)
			*cc2_lvl = this->measure_cc_pin_source(cc2_measure);
		else
			*cc1_lvl = this->measure_cc_pin_source(cc1_measure);
	} else {
		/* If VCONN not enabled, measure both cc1 and cc2 */
		*cc1_lvl = this->measure_cc_pin_source(cc1_measure);
		*cc2_lvl = this->measure_cc_pin_source(cc2_measure);
	}

}

/* Determine cc pin state for sink */
void FUSB302::detect_cc_pin_sink(int *cc1, int *cc2)
{
	int reg;
	int orig_meas_cc1;
	int orig_meas_cc2;
	int bc_lvl_cc1;
	int bc_lvl_cc2;

	/*
	 * Measure CC1 first.
	 */
	reg = this->tcpc_read(TCPC_REG_SWITCHES0);

	/* save original state to be returned to later... */
	if (reg & TCPC_REG_SWITCHES0_MEAS_CC1)
		orig_meas_cc1 = 1;
	else
		orig_meas_cc1 = 0;

	if (reg & TCPC_REG_SWITCHES0_MEAS_CC2)
		orig_meas_cc2 = 1;
	else
		orig_meas_cc2 = 0;

	/* Disable CC2 measurement switch, enable CC1 measurement switch */
	reg &= ~TCPC_REG_SWITCHES0_MEAS_CC2;
	reg |= TCPC_REG_SWITCHES0_MEAS_CC1;

	this->tcpc_write(TCPC_REG_SWITCHES0, reg);

	/* CC1 is now being measured by FUSB302. */

	/* Wait on measurement */
	delayMicroseconds(250);

	bc_lvl_cc1 = this->tcpc_read(TCPC_REG_STATUS0);

	/* mask away unwanted bits */
	bc_lvl_cc1 &= (TCPC_REG_STATUS0_BC_LVL0 | TCPC_REG_STATUS0_BC_LVL1);

	/*
	 * Measure CC2 next.
	 */

	reg = this->tcpc_read(TCPC_REG_SWITCHES0);

	/* Disable CC1 measurement switch, enable CC2 measurement switch */
	reg &= ~TCPC_REG_SWITCHES0_MEAS_CC1;
	reg |= TCPC_REG_SWITCHES0_MEAS_CC2;

	this->tcpc_write(TCPC_REG_SWITCHES0, reg);

	/* CC2 is now being measured by FUSB302. */

	/* Wait on measurement */
	delayMicroseconds(250);

	bc_lvl_cc2 = this->tcpc_read(TCPC_REG_STATUS0);

	/* mask away unwanted bits */
	bc_lvl_cc2 &= (TCPC_REG_STATUS0_BC_LVL0 | TCPC_REG_STATUS0_BC_LVL1);

	*cc1 = convert_bc_lvl(bc_lvl_cc1);
	*cc2 = convert_bc_lvl(bc_lvl_cc2);

	/* return MEAS_CC1/2 switches to original state */
	reg = this->tcpc_read(TCPC_REG_SWITCHES0);
	if (orig_meas_cc1)
		reg |= TCPC_REG_SWITCHES0_MEAS_CC1;
	else
		reg &= ~TCPC_REG_SWITCHES0_MEAS_CC1;
	if (orig_meas_cc2)
		reg |= TCPC_REG_SWITCHES0_MEAS_CC2;
	else
		reg &= ~TCPC_REG_SWITCHES0_MEAS_CC2;

	this->tcpc_write(TCPC_REG_SWITCHES0, reg);
}

int FUSB302::tcpm_init()
{
	int reg;

	/* set default */
	state.cc_polarity = -1;

	state.previous_pull = TYPEC_CC_RD;
	/* set the voltage threshold for no connect detection (vOpen) */
	state.mdac_vnc = TCPC_REG_MEASURE_MDAC_MV(PD_SRC_DEF_VNC_MV);
	/* set the voltage threshold for Rd vs Ra detection */
	state.mdac_rd = TCPC_REG_MEASURE_MDAC_MV(PD_SRC_DEF_RD_THRESH_MV);

	/* all other variables assumed to default to 0 */

	/* Restore default settings */
	this->tcpc_write(TCPC_REG_RESET, TCPC_REG_RESET_SW_RESET);

	/* Turn on retries and set number of retries */
	reg = this->tcpc_read(TCPC_REG_CONTROL3);
	reg |= TCPC_REG_CONTROL3_AUTO_RETRY;
	reg |= (PD_RETRY_COUNT & 0x3) <<
		TCPC_REG_CONTROL3_N_RETRIES_POS;
	this->tcpc_write(TCPC_REG_CONTROL3, reg);

	/* Create interrupt masks */
	reg = 0xFF;
	/* CC level changes */
	reg &= ~TCPC_REG_MASK_BC_LVL;
	/* collisions */
	reg &= ~TCPC_REG_MASK_COLLISION;
	/* misc alert */
	reg &= ~TCPC_REG_MASK_ALERT;
	this->tcpc_write(TCPC_REG_MASK, reg);

	reg = 0xFF;
	/* when all pd message retries fail... */
	reg &= ~TCPC_REG_MASKA_RETRYFAIL;
	/* when fusb302 send a hard reset. */
	reg &= ~TCPC_REG_MASKA_HARDSENT;
	/* when fusb302 receives GoodCRC ack for a pd message */
	reg &= ~TCPC_REG_MASKA_TX_SUCCESS;
	/* when fusb302 receives a hard reset */
	reg &= ~TCPC_REG_MASKA_HARDRESET;
	this->tcpc_write(TCPC_REG_MASKA, reg);

	reg = 0xFF;
	/* when fusb302 sends GoodCRC to ack a pd message */
	reg &= ~TCPC_REG_MASKB_GCRCSENT;
	this->tcpc_write(TCPC_REG_MASKB, reg);

	/* Interrupt Enable */
	reg = this->tcpc_read(TCPC_REG_CONTROL0);
	reg &= ~TCPC_REG_CONTROL0_INT_MASK;
	this->tcpc_write(TCPC_REG_CONTROL0, reg);

	/* Set VCONN switch defaults */
	this->tcpm_set_polarity(0);
	this->tcpm_set_vconn(0);

	/* Turn on the power! */
	/* TODO: Reduce power consumption */
	this->tcpc_write(TCPC_REG_POWER, TCPC_REG_POWER_PWR_ALL);

	return 0;
}

int FUSB302::tcpm_set_polarity(int polarity)
{
	/* Port polarity : 0 => CC1 is CC line, 1 => CC2 is CC line */
	int reg;

	reg = this->tcpc_read(TCPC_REG_SWITCHES0);

	/* clear VCONN switch bits */
	reg &= ~TCPC_REG_SWITCHES0_VCONN_CC1;
	reg &= ~TCPC_REG_SWITCHES0_VCONN_CC2;

	if (state.vconn_enabled) {
		/* set VCONN switch to be non-CC line */
		if (polarity)
			reg |= TCPC_REG_SWITCHES0_VCONN_CC1;
		else
			reg |= TCPC_REG_SWITCHES0_VCONN_CC2;
	}

	/* clear meas_cc bits (RX line select) */
	reg &= ~TCPC_REG_SWITCHES0_MEAS_CC1;
	reg &= ~TCPC_REG_SWITCHES0_MEAS_CC2;

	/* set rx polarity */
	if (polarity)
		reg |= TCPC_REG_SWITCHES0_MEAS_CC2;
	else
		reg |= TCPC_REG_SWITCHES0_MEAS_CC1;

	this->tcpc_write(TCPC_REG_SWITCHES0, reg);

	reg = this->tcpc_read(TCPC_REG_SWITCHES1);

	/* clear tx_cc bits */
	reg &= ~TCPC_REG_SWITCHES1_TXCC1_EN;
	reg &= ~TCPC_REG_SWITCHES1_TXCC2_EN;

	/* set tx polarity */
	if (polarity)
		reg |= TCPC_REG_SWITCHES1_TXCC2_EN;
	else
		reg |= TCPC_REG_SWITCHES1_TXCC1_EN;

	this->tcpc_write(TCPC_REG_SWITCHES1, reg);

	/* Save the polarity for later */
	state.cc_polarity = polarity;

	return 0;
}

int FUSB302::tcpm_set_vconn(int enable)
{
	/*
	 * FUSB302 does not have dedicated VCONN Enable switch.
	 * We'll get through this by disabling both of the
	 * VCONN - CC* switches to disable, and enabling the
	 * saved polarity when enabling.
	 * Therefore at startup, tcpm_set_polarity should be called first,
	 * or else live with the default put into tcpm_init.
	 */
	int reg;

	/* save enable state for later use */
	state.vconn_enabled = enable;

	if (enable) {
		/* set to saved polarity */
		tcpm_set_polarity(state.cc_polarity);
	} else {

		reg = this->tcpc_read(TCPC_REG_SWITCHES0);

		/* clear VCONN switch bits */
		reg &= ~TCPC_REG_SWITCHES0_VCONN_CC1;
		reg &= ~TCPC_REG_SWITCHES0_VCONN_CC2;

		this->tcpc_write(TCPC_REG_SWITCHES0, reg);
	}

	return 0;
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