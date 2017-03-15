/*
  FUSB302.c - Library for interacting with the FUSB302B chip.
  Copyright 2010 The Chromium OS Authors
  Copyright 2017 Jason Cerundolo
  Released under an MIT license. See LICENSE file. 
*/

#include "FUSB302.h"

FUSB302::FUSB302() : USB_TCPM()
{
    Wire.begin();
    this->vconn_enabled = 0;
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

    if (this->pulling_up) {
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
  this->tcpc_write(TCPC_REG_MEASURE, this->mdac_vnc);

    /* Wait on measurement */
    delayMicroseconds(250);

    /* Read status register */
    this->tcpc_read(TCPC_REG_STATUS0, &reg);

    /* Assume open */
    cc_lvl = TYPEC_CC_VOLT_OPEN;

    /* CC level is below the 'no connect' threshold (vOpen) */
    if ((reg & TCPC_REG_STATUS0_COMP) == 0) {
        /* Set MDAC for Rd vs Ra comparison */
        this->tcpc_write(TCPC_REG_MEASURE, this->mdac_rd);

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

    if (this->vconn_enabled) {
        /* If VCONN enabled, measure cc_pin that matches polarity */
        if (this->cc_polarity)
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

int FUSB302::init(void)
{
    int reg;

    /* set default */
    this->cc_polarity = -1;

    this->previous_pull = TYPEC_CC_RD;
    /* set the voltage threshold for no connect detection (vOpen) */
    this->mdac_vnc = TCPC_REG_MEASURE_MDAC_MV(PD_SRC_DEF_VNC_MV);
    /* set the voltage threshold for Rd vs Ra detection */
    this->mdac_rd = TCPC_REG_MEASURE_MDAC_MV(PD_SRC_DEF_RD_THRESH_MV);

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
    this->set_polarity(0);
    this->set_vconn(0);

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
int FUSB302::set_polarity(int polarity)
{
    /* Port polarity : 0 => CC1 is CC line, 1 => CC2 is CC line */
    int reg;

    this->tcpc_read(TCPC_REG_SWITCHES0, &reg);

    /* clear VCONN switch bits */
    reg &= ~TCPC_REG_SWITCHES0_VCONN_CC1;
    reg &= ~TCPC_REG_SWITCHES0_VCONN_CC2;

    if (this->vconn_enabled) {
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

    // Enable auto GoodCRC sending
    reg |= TCPC_REG_SWITCHES1_AUTO_GCRC;

    this->tcpc_write(TCPC_REG_SWITCHES1, reg);

    /* Save the polarity for later */
    this->cc_polarity = polarity;

    return 0;
}

int FUSB302::set_vconn(int enable)
{
    /*
     * FUSB302 does not have dedicated VCONN Enable switch.
     * We'll get through this by disabling both of the
     * VCONN - CC* switches to disable, and enabling the
     * saved polarity when enabling.
     * Therefore at startup, set_polarity should be called first,
     * or else live with the default put into init.
     */
    int reg;

    /* save enable state for later use */
    this->vconn_enabled = enable;

    if (enable) {
        /* set to saved polarity */
        set_polarity(this->cc_polarity);
    } else {

        this->tcpc_read(TCPC_REG_SWITCHES0, &reg);

        /* clear VCONN switch bits */
        reg &= ~TCPC_REG_SWITCHES0_VCONN_CC1;
        reg &= ~TCPC_REG_SWITCHES0_VCONN_CC2;

        this->tcpc_write(TCPC_REG_SWITCHES0, reg);
    }

    return 0;
}

int FUSB302::set_msg_header(int power_role, int data_role)
{
    int reg;

    this->tcpc_read(TCPC_REG_SWITCHES1, &reg);

    reg &= ~TCPC_REG_SWITCHES1_POWERROLE;
    reg &= ~TCPC_REG_SWITCHES1_DATAROLE;

    if (power_role)
        reg |= TCPC_REG_SWITCHES1_POWERROLE;
    if (data_role)
        reg |= TCPC_REG_SWITCHES1_DATAROLE;

    this->tcpc_write(TCPC_REG_SWITCHES1, reg);

    return 0;
}

int FUSB302::set_rx_enable(int enable)
{
    int reg;

    this->rx_enable = enable;

    /* Get current switch state */
    this->tcpc_read(TCPC_REG_SWITCHES0, &reg);

    /* Clear CC1/CC2 measure bits */
    reg &= ~TCPC_REG_SWITCHES0_MEAS_CC1;
    reg &= ~TCPC_REG_SWITCHES0_MEAS_CC2;

    if (enable) {
        switch (this->cc_polarity) {
        /* if CC polarity hasnt been determined, can't enable */
        case -1:
            return EC_ERROR_UNKNOWN;
        case 0:
            reg |= TCPC_REG_SWITCHES0_MEAS_CC1;
            break;
        case 1:
            reg |= TCPC_REG_SWITCHES0_MEAS_CC2;
            break;
        default:
            /* "shouldn't get here" */
            return EC_ERROR_UNKNOWN;
        }
        this->tcpc_write(TCPC_REG_SWITCHES0, reg);

        tcpc_read(TCPC_REG_SWITCHES1, &reg);
        reg |= TCPC_REG_SWITCHES1_AUTO_GCRC;
        tcpc_write(TCPC_REG_SWITCHES1, reg);

        /* flush rx fifo in case messages have been coming our way */
        this->flush_rx_fifo();


    } else {
        /*
         * bit of a hack here.
         * when this function is called to disable rx (enable=0)
         * using it as an indication of detach (gulp!)
         * to reset our knowledge of where
         * the toggle state machine landed.
         */
        this->togdone_pullup_cc1 = 0;
        this->togdone_pullup_cc2 = 0;

        this->set_cc(this->previous_pull);

        this->tcpc_write(TCPC_REG_SWITCHES0, reg);

        tcpc_read(TCPC_REG_SWITCHES1, &reg);
        reg &= ~(TCPC_REG_SWITCHES1_AUTO_GCRC);
        tcpc_write(TCPC_REG_SWITCHES1, reg);
    }

    this->auto_goodcrc_enable(enable);

    return 0;
}
/*
 * Make sure to allocate enough memory for *payload. 
 *
 * Maximum size is (max number of data objects + 1) * 4
 * The extra "+1" is for the CRC32 calculation
 */
int FUSB302::get_message(uint32_t *payload, int *head)
{
    /*
     * this is the buffer that will get the burst-read data
     * from the fusb302.
     *
     * it's re-used in a couple different spots, the worst of which
     * is the PD packet (not header) and CRC.
     * maximum size necessary = 28 + 4 = 32
     */
    uint8_t buf[32];
    int rv = 0;
    int len;

    /* NOTE: Assuming enough memory has been allocated for payload. */

    /*
     * PART 1 OF BURST READ: Write in register address.
     * Issue a START, no STOP.
     */
    //tcpc_lock(port, 1);
    buf[0] = TCPC_REG_FIFOS;
    rv |= this->tcpc_xfer(buf, 1, 0, 0, I2C_XFER_START);

    /*
     * PART 2 OF BURST READ: Read up to the header.
     * Issue a repeated START, no STOP.
     * only grab three bytes so we can get the header
     * and determine how many more bytes we need to read.
     */
    rv |= this->tcpc_xfer(0, 0, buf, 3, I2C_XFER_START);

    /* Grab the header */
    *head = (buf[1] & 0xFF);
    *head |= ((buf[2] << 8) & 0xFF00);

    /* figure out packet length, subtract header bytes */
    len = get_num_bytes(*head) - 2;

    /*
     * PART 3 OF BURST READ: Read everything else.
     * No START, but do issue a STOP at the end.
     * add 4 to len to read CRC out
     */
    rv |= this->tcpc_xfer(0, 0, buf, len+4, I2C_XFER_STOP);

    //tcpc_lock(port, 0);

    /* return the data */
    memcpy(payload, buf, len+4);

    return rv;
}

int FUSB302::send_message(uint16_t header, const uint32_t *data,
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
    rv = tcpc_xfer(buf, buf_pos, 0, 0, I2C_XFER_SINGLE);

    return rv;
}

int FUSB302::transmit(enum tcpm_transmit_type type,
                 uint16_t header, const uint32_t *data)
{
    /*
     * this is the buffer that will be burst-written into the fusb302
     * maximum size necessary =
     * 1: FIFO register address
     * 4: SOP* tokens
     * 1: Token that signifies "next X bytes are not tokens"
     * 30: 2 for header and up to 7*4 = 28 for rest of message
     * 1: "Insert CRC" Token
     * 1: EOP Token
     * 1: "Turn transmitter off" token
     * 1: "Star Transmission" Command
     * -
     * 40: 40 bytes worst-case
     */
    uint8_t buf[40];
    int buf_pos = 0;

    int reg;

    /* Flush the TXFIFO */
    this->flush_tx_fifo();

    switch (type) {
    case TCPC_TX_SOP:

        /* put register address first for of burst tcpc write */
        buf[buf_pos++] = TCPC_REG_FIFOS;

        /* Write the SOP Ordered Set into TX FIFO */
        buf[buf_pos++] = FUSB302_TKN_SYNC1;
        buf[buf_pos++] = FUSB302_TKN_SYNC1;
        buf[buf_pos++] = FUSB302_TKN_SYNC1;
        buf[buf_pos++] = FUSB302_TKN_SYNC2;

        return this->send_message(header, data, buf, buf_pos);
    case TCPC_TX_HARD_RESET:
        this->tx_hard_reset_req = 1;

        /* Simply hit the SEND_HARD_RESET bit */
        this->tcpc_read(TCPC_REG_CONTROL3, &reg);
        reg |= TCPC_REG_CONTROL3_SEND_HARDRESET;
        this->tcpc_write(TCPC_REG_CONTROL3, reg);

        break;
    case TCPC_TX_BIST_MODE_2:
        /* Simply hit the BIST_MODE2 bit */
        this->tcpc_read(TCPC_REG_CONTROL1, &reg);
        reg |= TCPC_REG_CONTROL1_BIST_MODE2;
        this->tcpc_write(TCPC_REG_CONTROL1, reg);
        break;
    default:
        return EC_ERROR_UNIMPLEMENTED;
    }

    return 0;
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
    this->mdac_vnc = vnc;
    this->mdac_rd = rd;
    return tcpc_write(TCPC_REG_CONTROL0, reg);
}

int FUSB302::get_vbus_level()
{
    int reg;

    /* Read status register */
    this->tcpc_read(TCPC_REG_STATUS0, &reg);

    return (reg & TCPC_REG_STATUS0_VBUSOK) ? 1 : 0;
}

int FUSB302::get_cc(int *cc1, int *cc2)
{
    /*
     * can't measure while doing DFP toggling -
     * FUSB302 takes control of the switches.
     * During this time, tell software that CCs are open -
     * at least until we get the TOGDONE interrupt...
     * which signals that the hardware found something.
     */
    if (this->dfp_toggling_on) {
        *cc1 = TYPEC_CC_VOLT_OPEN;
        *cc2 = TYPEC_CC_VOLT_OPEN;
        return 0;
    }

    if (this->pulling_up) {
        /* Source mode? */
        this->detect_cc_pin_source_manual(cc1, cc2);
    } else {
        /* Sink mode? */
        this->detect_cc_pin_sink(cc1, cc2);
    }

    return 0;
}

int FUSB302::set_cc(int pull)
{
    int reg;

    /*
     * Ensure we aren't in the process of changing CC from the alert
     * handler, then cancel any pending toggle-triggered CC change.
     */
    //mutex_lock(&this->set_cc_lock);
    this->dfp_toggling_on = 0;
    //mutex_unlock(&this->set_cc_lock);

    this->previous_pull = pull;

    /* NOTE: FUSB302 toggles a single pull-up between CC1 and CC2 */
    /* NOTE: FUSB302 Does not support Ra. */
    switch (pull) {
        case TYPEC_CC_RP:
            /* enable the pull-up we know to be necessary */
            tcpc_read(TCPC_REG_SWITCHES0, &reg);

            reg &= ~(TCPC_REG_SWITCHES0_CC2_PU_EN |
                 TCPC_REG_SWITCHES0_CC1_PU_EN |
                 TCPC_REG_SWITCHES0_CC1_PD_EN |
                 TCPC_REG_SWITCHES0_CC2_PD_EN |
                 TCPC_REG_SWITCHES0_VCONN_CC1 |
                 TCPC_REG_SWITCHES0_VCONN_CC2);

            reg |= TCPC_REG_SWITCHES0_CC1_PU_EN |
                TCPC_REG_SWITCHES0_CC2_PU_EN;

            if (this->vconn_enabled)
                reg |= this->togdone_pullup_cc1 ?
                       TCPC_REG_SWITCHES0_VCONN_CC2 :
                       TCPC_REG_SWITCHES0_VCONN_CC1;

            tcpc_write(TCPC_REG_SWITCHES0, reg);

            this->pulling_up = 1;
            this->dfp_toggling_on = 0;
            break;
        case TYPEC_CC_RD:
            /* Enable UFP Mode */

            /* turn off toggle */
            tcpc_read(TCPC_REG_CONTROL2, &reg);
            reg &= ~TCPC_REG_CONTROL2_TOGGLE;
            tcpc_write(TCPC_REG_CONTROL2, reg);

            /* enable pull-downs, disable pullups */
            tcpc_read(TCPC_REG_SWITCHES0, &reg);

            reg &= ~(TCPC_REG_SWITCHES0_CC2_PU_EN);
            reg &= ~(TCPC_REG_SWITCHES0_CC1_PU_EN);
            reg |= (TCPC_REG_SWITCHES0_CC1_PD_EN);
            reg |= (TCPC_REG_SWITCHES0_CC2_PD_EN);
            tcpc_write(TCPC_REG_SWITCHES0, reg);

            this->pulling_up = 0;
            this->dfp_toggling_on = 0;
            break;
        case TYPEC_CC_OPEN:
            /* Disable toggling */
            tcpc_read(TCPC_REG_CONTROL2, &reg);
            reg &= ~TCPC_REG_CONTROL2_TOGGLE;
            tcpc_write(TCPC_REG_CONTROL2, reg);

            /* Ensure manual switches are opened */
            tcpc_read(TCPC_REG_SWITCHES0, &reg);
            reg &= ~TCPC_REG_SWITCHES0_CC1_PU_EN;
            reg &= ~TCPC_REG_SWITCHES0_CC2_PU_EN;
            reg &= ~TCPC_REG_SWITCHES0_CC1_PD_EN;
            reg &= ~TCPC_REG_SWITCHES0_CC2_PD_EN;
            tcpc_write(TCPC_REG_SWITCHES0, reg);

            this->pulling_up = 0;
            this->dfp_toggling_on = 0;
            break;
        default:
            /* Unsupported... */
            return EC_ERROR_UNIMPLEMENTED;
    }
    return 0;
}

void FUSB302::set_bist_test_data()
{
    int reg;

    /* Read control3 register */
    this->tcpc_read(TCPC_REG_CONTROL3, &reg);

    /* Set the BIST_TMODE bit (Clears on Hard Reset) */
    reg |= TCPC_REG_CONTROL3_BIST_TMODE;

    /* Write the updated value */
    this->tcpc_write(TCPC_REG_CONTROL3, reg);
}

int FUSB302::get_chip_id(int *id) {
    int return_val;
    return_val = this->tcpc_read(TCPC_REG_DEVICE_ID, id);
    return return_val;
}

uint32_t FUSB302::get_interrupt_reason(void) {
    int res;
    uint32_t return_val;
    this->tcpc_read(TCPC_REG_INTERRUPTA, &res);
    return_val = ((res&0xFFl) << 16);
    this->tcpc_read(TCPC_REG_INTERRUPTB, &res);
    return_val |= ((res&0xFF) << 8);
    this->tcpc_read(TCPC_REG_INTERRUPT, &res);
    return_val |= (res&0xFF);

    return return_val;
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

int FUSB302::tcpc_xfer(const uint8_t *out, int out_size,
                uint8_t *in, int in_size, int flags) {
    Wire.beginTransmission(FUSB302_I2C_SLAVE_ADDR);
    for (; out_size>0; out_size--) {
        Wire.write(*out);
        out++;
    }

    if (in_size) {
        Wire.endTransmission(false);
        Wire.requestFrom(FUSB302_I2C_SLAVE_ADDR, in_size, (flags & I2C_XFER_STOP));
        for (; in_size>0; in_size--) {
            *in = Wire.read();
            in++;
        }
    } else {
        Wire.endTransmission(flags & I2C_XFER_STOP);
    }
}


void FUSB302::clear_int_pin(void) {
    int res;
    this->tcpc_read(TCPC_REG_INTERRUPTA, &res);
    this->tcpc_read(TCPC_REG_INTERRUPTB, &res);
    this->tcpc_read(TCPC_REG_INTERRUPT, &res);
}