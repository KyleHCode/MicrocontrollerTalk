// Add in decode for LoRA recieving side
// LoRA will send in a hex code to convert to relay states
// Add in LoRA config setup later

#include <Arduino.h>
#define RX_PIN 44  // GPIO44 D7 (RX on XIAO)
#define TX_PIN 43  // GPIO43 D6 (TX on XIAO)

#define RELAY1 1
#define RELAY2 2
#define RELAY3 3
#define RELAY4 4
#define RELAY5 5
#define RELAY6 6

const uint8_t BIT_START = 9;  // Start bit for relays in the 16-bit value

// Array of relay pins - using GPIO numbers that correspond to D0-D5 on XIAO
const uint8_t relayPins[6] = {RELAY1, RELAY2, RELAY3, RELAY4, RELAY5, RELAY6};  // D0, D1, D2, D3, D4, D5
// Function Prototypes
void setRelays(uint16_t state);

void setup() {
  Serial.begin(115200);           // USB debug serial
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);  // UART1 for ESP-to-ESP
  
  // Initialize relay pins as outputs
  for (uint8_t i = 0; i < 6; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], LOW);  // Start with all relays off
  }
  
  Serial.println("Receiver started");
}

void loop() {
  // Wait for new data to arrive
  if (Serial1.available()) {
    String hexString = Serial1.readStringUntil('\n');
    Serial.print("Received raw data: ");
    Serial.println(hexString);

    hexString.trim();  // Remove whitespace/newline
    uint16_t recievedBytes = (uint16_t)strtol(hexString.c_str(), NULL, 16);
    Serial.println("Received hex: " + hexString);
    Serial.println("Converted to binary: " + String(recievedBytes, BIN));
    
    // if MSB == 1, process the received relay state
    if (recievedBytes & 0x8000) {
      Serial.println("Setting relays to: " + String(recievedBytes, BIN));
      setRelays(recievedBytes);
    } else {
      Serial.println("Invalid data - MSB not set");
    }
  } else {
    // Show status when waiting for data
    Serial.print("Waiting for data... Available bytes: ");
    Serial.println(Serial1.available());
  }
  // Small delay to prevent excessive looping
  delay(100);
}

// write all relays at once from a 6-bit value
void setRelays(uint16_t state) {
    for (uint8_t i = 0; i < 6; ++i) {
        bool on = state & (1u << (i + BIT_START));  // check bits 9-14 for relays
        digitalWrite(relayPins[i], on ? HIGH : LOW);
        Serial.println("Setting relay " + String(i+1) + " to " + String(on ? "ON" : "OFF"));
    }
}

String stringToHex(String text) {
    String hex = "";
    for(int i = 0; i < text.length(); i++) {
        char buf[3];
        sprintf(buf, "%02X", text[i]);
        hex += buf;
    }
    return hex;
}

uint16_t hexToUint16(String hexString) {
    hexString.trim();
    return (uint16_t)strtol(hexString.c_str(), NULL, 16);
}