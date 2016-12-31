#ifndef USBC_H
#define USBC_H

#include "Arduino.h"

#define PD_HEADER_CNT(header)  (((header) >> 12) & 7)
#define PD_HEADER_TYPE(header) ((header) & 0xF)
#define PD_HEADER_ID(header)   (((header) >> 9) & 7)

/* Flags for i2c_xfer() */
#define I2C_XFER_START (1 << 0)  /* Start smbus session from idle state */
#define I2C_XFER_STOP (1 << 1)  /* Terminate smbus session with stop bit */
#define I2C_XFER_SINGLE (I2C_XFER_START | I2C_XFER_STOP)  /* One transaction */

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

int get_num_bytes(uint16_t header);

#endif /* USBC_H */