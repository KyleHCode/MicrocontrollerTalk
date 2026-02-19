#include "lora_config.h"
#include "LoRaModule.h"
#include<HardwareSerial.h>
#include <Wire.h>
#define RX_PIN 8  
#define TX_PIN 7 
#define SLAVE_ADDR 0x08

#define RELAY1 1
#define RELAY2 2
#define RELAY3 3
#define RELAY4 4
#define RELAY5 9
#define RELAY6 10

// Array of relay pins - using GPIO numbers that correspond to D0-D5 on XIAO
const uint8_t relayPins[6] = {RELAY1, RELAY2, RELAY3, RELAY4, RELAY5, RELAY6};
int bitArray[16];

// Create LoRa module instance

HardwareSerial* loraSerial = &Serial1;
// Function Prototypes
void setRelays(uint16_t state);
String sendATcommand(const char *toSend, unsigned long milliseconds);
void i2cScan();

void setup() {
  Serial.begin(115200);           // USB debug serial
  delay(2000);  // Wait for serial to initialize
  
  Serial.println("STARTING RECEIVER...");
  Serial.flush();
  
  // Initialize relay pins as outputs
  for (uint8_t i = 0; i < 6; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], LOW);  // Start with all relays off
  }
  
  Serial.println("=== LoRa Relay Receiver ===");
  
  // Initialize and configure LoRa module
#if defined(ARDUINO_ARCH_ESP32)
  loraSerial->begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
#else
  loraSerial->begin(115200);
#endif
  delay(1000);
  
  Serial.println("Initializing LoRa Receiver...");
  
  // Verify module is responding
  if (sendATcommand("AT", 1000).indexOf("OK") != -1) {
    Serial.println("LoRa module responding");
  } else {
    Serial.println("WARNING: LoRa module not responding!");
  }
  
  // Configure this module as receiver (address 7)
  sendATcommand("AT+ADDRESS=7", 1000);
  delay(100);
  sendATcommand("AT+BAND=915000000", 1000);
  delay(100);
  sendATcommand("AT+NETWORKID=18", 1000);
  delay(1000);
  sendATcommand("AT+PARAMETER=11,9,4,24",1000);
  delay(200);
  Serial.println("Setup complete. Listening for messages...");
  Serial.println("This module is Address 7");
  sendATcommand("AT+PARAMETER=11,9,4,24",1000);

  Serial.println("Listening for relay commands...");

#if defined(ARDUINO_ARCH_ESP32)
  Wire.begin(4, 5); // SDA, SCL pins for I2C
#else
  Wire.begin();
#endif
  delay(50);
  i2cScan();

}
void sendInt16(uint16_t val) {
  Wire.beginTransmission(SLAVE_ADDR);

  uint8_t high = highByte(val);
  uint8_t low = lowByte(val);

  Wire.write(high);
  Wire.write(low);

  byte error = Wire.endTransmission();
  if (error == 0) {
    Serial.print("Sent successfully: ");
    Serial.print(val);
    Serial.print(" (0x");
    Serial.print(val, HEX);
    Serial.println(")");
    return;
  }

  // Retry once or twice for address NACKs
  if (error == 2) {
    for (int attempt = 1; attempt <= 2 && error != 0; ++attempt) {
      delay(50);
      Wire.beginTransmission(SLAVE_ADDR);
      Wire.write(high);
      Wire.write(low);
      error = Wire.endTransmission();
      Serial.print("Retry "); Serial.print(attempt); Serial.print(" result: "); Serial.println(error);
    }
  }

  Serial.print("Error: ");
  Serial.print(error);
  Serial.print(" - ");
  switch (error) {
    case 1: Serial.println("Data too long to fit in transmit buffer"); break;
    case 2: Serial.println("NACK on address (no device responding)"); break;
    case 3: Serial.println("NACK on data"); break;
    case 4: Serial.println("Other error (bus error)"); break;
    default: Serial.println("Unknown error"); break;
  }
}

void i2cScan() {
  Serial.println("I2C scan starting...");
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    byte err = Wire.endTransmission();
    if (err == 0) {
      Serial.print(" - Found device at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      ++found;
    }
  }
  if (found == 0) Serial.println("No I2C devices found.");
  else {
    Serial.print("I2C scan complete. Devices found: ");
    Serial.println(found);
  }

  // Probe expected slave specifically
  Wire.beginTransmission(SLAVE_ADDR);
  byte probe = Wire.endTransmission();
  Serial.print("Probe slave 0x"); Serial.print(SLAVE_ADDR, HEX); Serial.print(" -> "); Serial.println(probe);
}

void loop() {
  String hexData;
  String data;
  // Use LoRa module to receive data
  // Instead of reading LoRa, send a fake relay command over I2C periodically.
  // Format: MSB set + bits 9-14 represent the six relays (use 0x7E00 to toggle all).
  uint16_t fakeRelayCode = RELAY_MSB_BIT | 0x7E00; // MSB + relays bits set
  Serial.print("Sending fake relay code over I2C: 0x");
  Serial.println(fakeRelayCode, HEX);
  sendInt16(fakeRelayCode);

  // wait before sending the next fake command
  delay(2000);
}

// write all relays at once from a 6-bit value
void setRelays(uint16_t state) {
    for (uint8_t i = 0; i <6; i++) {
        bool off = state & (1u << (14-i));  // check bits 9-14 for relays
        if(off)digitalWrite(relayPins[i],LOW);
        else digitalWrite(relayPins[i],HIGH);
        //digitalWrite(relayPins[i], on ? HIGH : LOW);

        Serial.println("Setting relay " + String(i+1) + " to " + String(off ? "ON" : "OFF"));
    }
    // String convert = String(state, HEX);
    // Wire.beginTransmission(SLAVE_ADDR);
    // // Send the string directly
    // Wire.write((const uint8_t*)convert.c_str(), convert.length());
    // Wire.endTransmission();
    // if (error == 0) {
    // Serial.println("Sent successfully");
    // } else {
    // Serial.print("Error: ");
    // Serial.println(error);
    // } 
    delay(20);
  
}
String sendATcommand(const char *toSend, unsigned long milliseconds) {
  String result = "";
  
  Serial.print("Sending: ");
  Serial.println(toSend);
  
  // Clear any pending data in buffer
  while (loraSerial->available()) {
    loraSerial->read();
  }
  
  loraSerial->println(toSend);
  
  unsigned long startTime = millis();
  Serial.print("Received: ");
  
  while (millis() - startTime < milliseconds) {
    if (loraSerial->available()) {
      char c = loraSerial->read();
      Serial.write(c);
      result += c;
    }
  }
  
  Serial.println();
  return result;
}