/*
  USB_TCPM.h - Library for interacting with the FUSB302B chip.
  Copyright 2010 The Chromium OS Authors
  Copyright 2017 Jason Cerundolo
  Released under an MIT license. See LICENSE file. 
*/

#ifndef USB_TCPM_H
#define USB_TCPM_H

#include "Arduino.h"

#define PD_HEADER_CNT(header)  (((header) >> 12) & 7)
#define PD_HEADER_TYPE(header) ((header) & 0xF)
#define PD_HEADER_ID(header)   (((header) >> 9) & 7)

/* Minimum PD supply current  (mA) */
#define PD_MIN_MA   500

/* Minimum PD voltage (mV) */
#define PD_MIN_MV   5000

/* No connect voltage threshold for sources based on Rp */
#define PD_SRC_DEF_VNC_MV        1600
#define PD_SRC_1_5_VNC_MV        1600
#define PD_SRC_3_0_VNC_MV        2600

/* Rd voltage threshold for sources based on Rp */
#define PD_SRC_DEF_RD_THRESH_MV  200
#define PD_SRC_1_5_RD_THRESH_MV  400
#define PD_SRC_3_0_RD_THRESH_MV  800

/* Voltage threshold to detect connection when presenting Rd */
#define PD_SNK_VA_MV             250

/* Flags for i2c_xfer() */
#define I2C_XFER_START (1 << 0)  /* Start smbus session from idle state */
#define I2C_XFER_STOP (1 << 1)  /* Terminate smbus session with stop bit */
#define I2C_XFER_SINGLE (I2C_XFER_START | I2C_XFER_STOP)  /* One transaction */

/* Default retry count for transmitting */
#define PD_RETRY_COUNT 3

enum tcpc_cc_voltage_status {
    TYPEC_CC_VOLT_OPEN = 0,
    TYPEC_CC_VOLT_RA = 1,
    TYPEC_CC_VOLT_RD = 2,
    TYPEC_CC_VOLT_SNK_DEF = 5,
    TYPEC_CC_VOLT_SNK_1_5 = 6,
    TYPEC_CC_VOLT_SNK_3_0 = 7,
};

enum tcpc_cc_pull {
    TYPEC_CC_RA = 0,
    TYPEC_CC_RP = 1,
    TYPEC_CC_RD = 2,
    TYPEC_CC_OPEN = 3,
};

enum tcpc_rp_value {
    TYPEC_RP_USB = 0,
    TYPEC_RP_1A5 = 1,
    TYPEC_RP_3A0 = 2,
    TYPEC_RP_RESERVED = 3,
};

enum tcpm_transmit_type {
    TCPC_TX_SOP = 0,
    TCPC_TX_SOP_PRIME = 1,
    TCPC_TX_SOP_PRIME_PRIME = 2,
    TCPC_TX_SOP_DEBUG_PRIME = 3,
    TCPC_TX_SOP_DEBUG_PRIME_PRIME = 4,
    TCPC_TX_HARD_RESET = 5,
    TCPC_TX_CABLE_RESET = 6,
    TCPC_TX_BIST_MODE_2 = 7
};

enum tcpc_transmit_complete {
    TCPC_TX_COMPLETE_SUCCESS =   0,
    TCPC_TX_COMPLETE_DISCARDED = 1,
    TCPC_TX_COMPLETE_FAILED =    2,
};

/* List of common error codes that can be returned */
enum ec_error_list {
    /* Success - no error */
    EC_SUCCESS = 0,
    /* Unknown error */
    EC_ERROR_UNKNOWN = 1,
    /* Function not implemented yet */
    EC_ERROR_UNIMPLEMENTED = 2,
    /* Overflow error; too much input provided. */
    EC_ERROR_OVERFLOW = 3,
    /* Timeout */
    EC_ERROR_TIMEOUT = 4,
    /* Invalid argument */
    EC_ERROR_INVAL = 5,
    /* Already in use, or not ready yet */
    EC_ERROR_BUSY = 6,
    /* Access denied */
    EC_ERROR_ACCESS_DENIED = 7,
    /* Failed because component does not have power */
    EC_ERROR_NOT_POWERED = 8,
    /* Failed because component is not calibrated */
    EC_ERROR_NOT_CALIBRATED = 9,
    /* Failed because CRC error */
    EC_ERROR_CRC = 10,
    /* Invalid console command param (PARAMn means parameter n is bad) */
    EC_ERROR_PARAM1 = 11,
    EC_ERROR_PARAM2 = 12,
    EC_ERROR_PARAM3 = 13,
    EC_ERROR_PARAM4 = 14,
    EC_ERROR_PARAM5 = 15,
    EC_ERROR_PARAM6 = 16,
    EC_ERROR_PARAM7 = 17,
    EC_ERROR_PARAM8 = 18,
    EC_ERROR_PARAM9 = 19,
    EC_ERROR_PARAM_COUNT = 20,  /* Wrong number of params */

    EC_ERROR_NOT_HANDLED = 21,  /* Interrupt event not handled */

    /* Module-internal error codes may use this range.   */
    EC_ERROR_INTERNAL_FIRST = 0x10000,
    EC_ERROR_INTERNAL_LAST =  0x1FFFF
};

class USB_TCPM
{
  public:
    USB_TCPM() {}

    // Common methods for TCPM implementations
    virtual int     init(void) =0;
    virtual int     get_cc(int *cc1, int *cc2) =0;
    virtual int     get_vbus_level(void) =0;
    virtual int     select_rp_value(int rp) =0;
    virtual int     set_cc(int pull) =0;
    virtual int     set_polarity(int polarity) =0;
    virtual int     set_vconn(int enable) =0;
    virtual int     set_msg_header(int power_role, int data_role) =0;
    virtual int     set_rx_enable(int enable) =0;
    virtual int     get_message(uint32_t *payload, int *head) =0;
    virtual int     transmit(enum tcpm_transmit_type type,
                        uint16_t header, const uint32_t *data) =0;
    //int   alert(void);
    
    static int      get_num_bytes(uint16_t header);
};

#endif /* USB_TCPM_H */
