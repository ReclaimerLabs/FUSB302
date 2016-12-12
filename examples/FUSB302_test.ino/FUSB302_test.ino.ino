#include <FUSB302.h>

FUSB302 usbc;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Hello World!");
  digitalWrite(13, HIGH);

  int return_val;
  return_val = usbc.tcpc_read(0x01);
  Serial.print("Return Value = 0x");
  Serial.println(return_val, HEX);

  usbc.pd_reset();
  usbc.flush_rx_fifo();

  return_val = usbc.tcpc_read(0x01);
  Serial.print("Return Value = 0x");
  Serial.println(return_val, HEX);
}

void loop() {
  // put your main code here, to run repeatedly:

}

