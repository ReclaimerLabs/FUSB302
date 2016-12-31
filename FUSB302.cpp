/*
  FUSB302.c - Library for interacting with the FUSB302B chip.
  Created by Jason Cerundolo, December 11, 2016.
  Released under an MIT license. See LICENSE.md. 
*/

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

void FUSB302::auto_goodcrc_enable(int enable) {
	int reg;

	tcpc_read(TCPC_REG_SWITCHES1, &reg);

	if (enable)
		reg |= TCPC_REG_SWITCHES1_AUTO_GCRC;
	else
		reg &= ~TCPC_REG_SWITCHES1_AUTO_GCRC;

	tcpc_write(TCPC_REG_SWITCHES1, reg);
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

	this->tcpc_read(TCPC_REG_CONTROL0, &reg);
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
	this->tcpc_read(TCPC_REG_SWITCHES0, &reg);
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
	this->tcpc_read(TCPC_REG_STATUS0, &reg);

	/* Assume open */
	cc_lvl = TYPEC_CC_VOLT_OPEN;

	/* CC level is below the 'no connect' threshold (vOpen) */
	if ((reg & TCPC_REG_STATUS0_COMP) == 0) {
		/* Set MDAC for Rd vs Ra comparison */
		this->tcpc_write(TCPC_REG_MEASURE, state.mdac_rd);

		/* Wait on measurement */
		delayMicroseconds(250);

		/* Read status register */
		this->tcpc_read(TCPC_REG_STATUS0, &reg);

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
	this->tcpc_read(TCPC_REG_SWITCHES0, &reg);

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

	this->tcpc_read(TCPC_REG_STATUS0, &bc_lvl_cc1);

	/* mask away unwanted bits */
	bc_lvl_cc1 &= (TCPC_REG_STATUS0_BC_LVL0 | TCPC_REG_STATUS0_BC_LVL1);

	/*
	 * Measure CC2 next.
	 */

	this->tcpc_read(TCPC_REG_SWITCHES0, &reg);

	/* Disable CC1 measurement switch, enable CC2 measurement switch */
	reg &= ~TCPC_REG_SWITCHES0_MEAS_CC1;
	reg |= TCPC_REG_SWITCHES0_MEAS_CC2;

	this->tcpc_write(TCPC_REG_SWITCHES0, reg);

	/* CC2 is now being measured by FUSB302. */

	/* Wait on measurement */
	delayMicroseconds(250);

	this->tcpc_read(TCPC_REG_STATUS0, &bc_lvl_cc2);

	/* mask away unwanted bits */
	bc_lvl_cc2 &= (TCPC_REG_STATUS0_BC_LVL0 | TCPC_REG_STATUS0_BC_LVL1);

	*cc1 = convert_bc_lvl(bc_lvl_cc1);
	*cc2 = convert_bc_lvl(bc_lvl_cc2);

	/* return MEAS_CC1/2 switches to original state */
	this->tcpc_read(TCPC_REG_SWITCHES0, &reg);
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
	this->tcpc_read(TCPC_REG_CONTROL3, &reg);
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
	this->tcpc_read(TCPC_REG_CONTROL0, &reg);
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

/**
 * Set polarity
 *
 * @param polarity 0=> transmit on CC1, 1=> transmit on CC2
 *
 * @return 0 or error
 */
int FUSB302::tcpm_set_polarity(int polarity)
{
	/* Port polarity : 0 => CC1 is CC line, 1 => CC2 is CC line */
	int reg;

	this->tcpc_read(TCPC_REG_SWITCHES0, &reg);

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

	this->tcpc_read(TCPC_REG_SWITCHES1, &reg);

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

		this->tcpc_read(TCPC_REG_SWITCHES0, &reg);

		/* clear VCONN switch bits */
		reg &= ~TCPC_REG_SWITCHES0_VCONN_CC1;
		reg &= ~TCPC_REG_SWITCHES0_VCONN_CC2;

		this->tcpc_write(TCPC_REG_SWITCHES0, reg);
	}

	return 0;
}

int FUSB302::send_message(int port, uint16_t header, const uint32_t *data,
				 uint8_t *buf, int buf_pos)
{
	int rv;
	int reg;
	int len;

	len = get_num_bytes(header);

	/*
	 * packsym tells the TXFIFO that the next X bytes are payload,
	 * and should not be interpreted as special tokens.
	 * The 5 LSBs represent X, the number of bytes.
	 */
	reg = FUSB302_TKN_PACKSYM;
	reg |= (len & 0x1F);

	buf[buf_pos++] = reg;

	/* write in the header */
	reg = header;
	buf[buf_pos++] = reg & 0xFF;

	reg >>= 8;
	buf[buf_pos++] = reg & 0xFF;

	/* header is done, subtract from length to make this for-loop simpler */
	len -= 2;

	/* write data objects, if present */
	memcpy(&buf[buf_pos], data, len);
	buf_pos += len;

	/* put in the CRC */
	buf[buf_pos++] = FUSB302_TKN_JAMCRC;

	/* put in EOP */
	buf[buf_pos++] = FUSB302_TKN_EOP;

	/* Turn transmitter off after sending message */
	buf[buf_pos++] = FUSB302_TKN_TXOFF;

	/* Start transmission */
	reg = FUSB302_TKN_TXON;
	buf[buf_pos++] = FUSB302_TKN_TXON;

	/* burst write for speed! */
	rv = tcpc_xfer(port, buf, buf_pos, 0, 0);

	return rv;
}

/**
	 * Set the value of the CC pull-up used when we are a source.
	 *
	 * @param rp One of enum tcpc_rp_value
	 *
	 * @return 0 or error
	 */
int FUSB302::select_rp_value(int rp) {
	int reg;
	int rv;
	uint8_t vnc, rd;

	rv = tcpc_read(TCPC_REG_CONTROL0, &reg);
	if (rv) {
		return rv;
	}

	/* Set the current source for Rp value */
	reg &= ~TCPC_REG_CONTROL0_HOST_CUR_MASK;
	switch (rp) {
	case TYPEC_RP_1A5:
		reg |= TCPC_REG_CONTROL0_HOST_CUR_1A5;
		vnc = TCPC_REG_MEASURE_MDAC_MV(PD_SRC_1_5_VNC_MV);
		rd = TCPC_REG_MEASURE_MDAC_MV(PD_SRC_1_5_RD_THRESH_MV);
		break;
	case TYPEC_RP_3A0:
		reg |= TCPC_REG_CONTROL0_HOST_CUR_3A0;
		vnc = TCPC_REG_MEASURE_MDAC_MV(PD_SRC_3_0_VNC_MV);
		rd = TCPC_REG_MEASURE_MDAC_MV(PD_SRC_3_0_RD_THRESH_MV);
		break;
	case TYPEC_RP_USB:
	default:
		reg |= TCPC_REG_CONTROL0_HOST_CUR_USB;
		vnc = TCPC_REG_MEASURE_MDAC_MV(PD_SRC_DEF_VNC_MV);
		rd = TCPC_REG_MEASURE_MDAC_MV(PD_SRC_DEF_RD_THRESH_MV);
	}
	state.mdac_vnc = vnc;
	state.mdac_rd = rd;
	return tcpc_write(TCPC_REG_CONTROL0, reg);
}

int FUSB302::tcpc_write(int reg, int val) {
	Wire.beginTransmission(FUSB302_I2C_SLAVE_ADDR);
	Wire.write(reg & 0xFF);
	Wire.write(val & 0xFF);
	Wire.endTransmission();
	
	return 0;
}

int FUSB302::tcpc_read(int reg, int *val) {
	Wire.beginTransmission(FUSB302_I2C_SLAVE_ADDR);
	Wire.write(reg & 0xFF);
	Wire.endTransmission(false);
	Wire.requestFrom(FUSB302_I2C_SLAVE_ADDR, 1, true);
	*val = Wire.read();

    return 0;
}

int FUSB302::tcpc_xfer(int port,
			    const uint8_t *out, int out_size,
			    uint8_t *in, int in_size) {
	Wire.beginTransmission(FUSB302_I2C_SLAVE_ADDR);
	for (; out_size>0; out_size--) {
		Wire.write(*out);
		out++;
	}
	if (in_size) {
		Wire.endTransmission(false);
		Wire.requestFrom(FUSB302_I2C_SLAVE_ADDR, in_size, true);
		for (; in_size>0; in_size--) {
			*in = Wire.read();
			in++;
		}
	} else {
		Wire.endTransmission(true);
	}
}
