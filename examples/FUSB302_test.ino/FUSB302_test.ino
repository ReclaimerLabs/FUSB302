#include <FUSB302.h>

/*
 * Use this example with the 
 * Reclaimer Labs USB-C Breakout Board
 * https://www.tindie.com/products/ReclaimerLabs/usb-type-c-power-delivery-phy-breakout-board/
 * 
 * How to connect your Arduino. 
 *
 * Name on Arduino  ->  Name on Breakout Board
 *
 * GND              ->  GND
 * 3.3V             ->  VDD
 * IOREF            ->  Vpu
 *
 * The I2C lines must be connected as per the Wire library
 * https://www.arduino.cc/en/Reference/Wire
 *
 * For Arduino Uno
 * A4               ->  SDA
 * A5               ->  SCL
 */

FUSB302 usbc;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Welcome to USB-C!");
  digitalWrite(13, HIGH);

  int       usb_pd_message_header;
  uint32_t  usb_pd_message_buffer[10];

  usbc.init();

  int chip_id;
  usbc.get_chip_id(&chip_id);
  Serial.print("FUSB302 ID = 0x");
  Serial.println(chip_id, HEX);

  usbc.pd_reset();

  int cc1_meas, cc2_meas;
  usbc.detect_cc_pin_sink(&cc1_meas, &cc2_meas);
  Serial.print("CC1 level = ");
  switch (cc1_meas) {
    case TYPEC_CC_VOLT_OPEN :
      Serial.println("Open");
      break;
    case TYPEC_CC_VOLT_RA :
      Serial.println("Ra pull-down");
      break;
    case TYPEC_CC_VOLT_RD :
      Serial.println("Rd pull-down");
      break;
    case TYPEC_CC_VOLT_SNK_DEF :
      Serial.println("Connected with default power");
      break;
    case TYPEC_CC_VOLT_SNK_1_5 :
      Serial.println("Connected with 1.5A at 5V");
      break;
    case TYPEC_CC_VOLT_SNK_3_0 :
      Serial.println("Connected with 3.0A at 5V");
      break;
    default :
      Serial.println("Unknown");
      break;
  }

  Serial.print("CC2 level = ");
  switch (cc2_meas) {
    case TYPEC_CC_VOLT_OPEN :
      Serial.println("Open");
      break;
    case TYPEC_CC_VOLT_RA :
      Serial.println("Ra pull-down");
      break;
    case TYPEC_CC_VOLT_RD :
      Serial.println("Rd pull-down");
      break;
    case TYPEC_CC_VOLT_SNK_DEF :
      Serial.println("Connected with default power");
      break;
    case TYPEC_CC_VOLT_SNK_1_5 :
      Serial.println("Connected with 1.5A at 5V");
      break;
    case TYPEC_CC_VOLT_SNK_3_0 :
      Serial.println("Connected with 3.0A at 5V");
      break;
    default :
      Serial.println("Unknown");
      break;
  }

  if (cc1_meas > cc2_meas) {
    usbc.set_polarity(0);
  } else {
    usbc.set_polarity(1);
  }

  delay(1000);

  usbc.get_message(usb_pd_message_buffer, &usb_pd_message_header);
  Serial.print("Header = 0x");
  Serial.println(usb_pd_message_header, HEX);
  for (int i=0; i<10; i++) {
    Serial.print(i, DEC);
    Serial.print(" = 0x");
    Serial.println(usb_pd_message_buffer[i], HEX);
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
