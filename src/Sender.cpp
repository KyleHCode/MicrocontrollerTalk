#include <Arduino.h>

void setup() {
  Serial.begin(115200);           // USB debug serial
  Serial1.begin(115200, SERIAL_8N1, RX2, TX2);  // RX, TX pins for ESP32
  Serial.println("Sender started");
}

void loop() {
  uint16_t relayState = 0b1010101000000000; // relays 2,4,6 on
  Serial.print("Sending relay state: ");
  Serial.println(relayState, BIN);
  Serial1.write((uint8_t*)&relayState, sizeof(relayState));
  delay(2000);

  relayState = 0b1101010000000000; // relays 1,3,5 on
  Serial.print("Sending relay state: ");
  Serial.println(relayState, BIN);  
  Serial1.write((uint8_t*)&relayState, sizeof(relayState));
  delay(2000);
}