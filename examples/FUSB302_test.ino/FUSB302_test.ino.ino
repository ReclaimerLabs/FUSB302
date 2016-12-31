#include <FUSB302.h>

FUSB302 usbc;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Welcome to USB-C!");
  digitalWrite(13, HIGH);

  usbc.tcpm_init();

  int return_val;
  usbc.tcpc_read(0x01, &return_val);
  Serial.print("FUSB302 ID = 0x");
  Serial.println(return_val, HEX);

  usbc.pd_reset();
  usbc.flush_rx_fifo();
  usbc.flush_tx_fifo();

  int cc1_meas, cc2_meas;
  usbc.detect_cc_pin_sink(&cc1_meas, &cc2_meas);
  Serial.print("CC1 level = ");
  Serial.println(cc1_meas);
  Serial.print("CC2 level = ");
  Serial.println(cc2_meas);
}

void loop() {
  // put your main code here, to run repeatedly:

}
