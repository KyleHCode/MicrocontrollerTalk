// Receiver: LoRa2 -> MCU2 -> Relays
// Receives hex relay commands from LoRa and controls relays

#include <Arduino.h>
#include "lora_config.h"

#define RX_PIN 44  // GPIO44 D7 (RX on XIAO) - connects to LoRa TX
#define TX_PIN 43  // GPIO43 D6 (TX on XIAO) - connects to LoRa RX

#define RELAY1 1
#define RELAY2 2
#define RELAY3 3
#define RELAY4 4
#define RELAY5 5
#define RELAY6 6

// Array of relay pins - using GPIO numbers that correspond to D0-D5 on XIAO
const uint8_t relayPins[6] = {RELAY1, RELAY2, RELAY3, RELAY4, RELAY5, RELAY6};
// Function Prototypes
void setRelays(uint16_t state);

HardwareSerial loraSerial(1);

String sendATCommand(const char* cmd, unsigned long timeout);

void setup() {
  Serial.begin(115200);           // USB debug serial
  delay(2000);  // Wait for serial to initialize
  
  Serial.println("STARTING RECEIVER...");
  Serial.flush();
  
  loraSerial.begin(LORA_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);  // UART1 for LoRa module
  delay(LORA_INIT_DELAY);
  
  // Initialize relay pins as outputs
  for (uint8_t i = 0; i < 6; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], LOW);  // Start with all relays off
  }
  
  Serial.println("=== LoRa Relay Receiver ===");
  
  // Clear any boot noise from LoRa serial buffer
  delay(500);
  while (loraSerial.available()) {
    loraSerial.read();
  }
  
  // Configure LoRa module
  Serial.println("Testing LoRa connection...");
  String response = sendATCommand(AT_TEST, AT_COMMAND_TIMEOUT);
  if (response.indexOf("OK") != -1) {
    Serial.println("✓ LoRa module responding");
  } else {
    Serial.println("✗ LoRa not responding!");
    Serial.print("Got: ");
    Serial.println(response);
  }
  
  char cmd[50];
  sprintf(cmd, AT_SET_ADDRESS_FMT, LORA_RECEIVER_ADDRESS);
  sendATCommand(cmd, AT_COMMAND_TIMEOUT);
  delay(LORA_CONFIG_DELAY);
  
  sprintf(cmd, AT_SET_BAND_FMT, LORA_BAND);
  sendATCommand(cmd, AT_COMMAND_TIMEOUT);
  delay(LORA_CONFIG_DELAY);
  
  sprintf(cmd, AT_SET_NETWORK_FMT, LORA_NETWORK_ID);
  sendATCommand(cmd, AT_COMMAND_TIMEOUT);
  delay(LORA_INIT_DELAY);
  
  Serial.println("LoRa configured: Address " + String(LORA_RECEIVER_ADDRESS) + ", Network " + String(LORA_NETWORK_ID));
  Serial.println("Listening for relay commands...");
}

void loop() {
  // Check for incoming LoRa data
  if (loraSerial.available()) {
    String incomingMsg = "";
    
    // Read with timeout
    unsigned long startTime = millis();
    while (millis() - startTime < 1000) {
      if (loraSerial.available()) {
        char c = loraSerial.read();
        if (c == '\n') break;
        incomingMsg += c;
      }
    }
    
    Serial.print("Raw LoRa: ");
    Serial.println(incomingMsg);
    
    // Parse: +RCV=<address>,<length>,<data>,<RSSI>,<SNR>
    int firstComma = incomingMsg.indexOf(',');
    int secondComma = incomingMsg.indexOf(',', firstComma + 1);
    int thirdComma = incomingMsg.indexOf(',', secondComma + 1);
    
    if (secondComma > 0 && thirdComma > 0) {
      // Extract hex data between 2nd and 3rd comma
      String hexData = incomingMsg.substring(secondComma + 1, thirdComma);
      hexData.trim();
      
      Serial.print("Hex data: ");
      Serial.println(hexData);
      
      // Convert hex to uint16_t
      uint16_t receivedBytes = (uint16_t)strtol(hexData.c_str(), NULL, 16);
      Serial.print("Binary: ");
      Serial.println(String(receivedBytes, BIN));
      
      // Check MSB and process
      if (receivedBytes & RELAY_MSB_BIT) {
        Serial.println("Valid relay command - updating relays");
        setRelays(receivedBytes);
      } else {
        Serial.println("Invalid: MSB not set");
      }
    }
  }
  
  delay(50);
}

// write all relays at once from a 6-bit value
void setRelays(uint16_t state) {
    for (uint8_t i = 0; i < 6; ++i) {
        bool on = state & (1u << (i + RELAY_BIT_START));  // check bits 9-14 for relays
        digitalWrite(relayPins[i], on ? HIGH : LOW);
        Serial.println("Setting relay " + String(i+1) + " to " + String(on ? "ON" : "OFF"));
    }
}

uint16_t hexToUint16(String hexString) {
    hexString.trim();
    return (uint16_t)strtol(hexString.c_str(), NULL, 16);
}

String sendATCommand(const char* cmd, unsigned long timeout) {
    String result = "";
    
    Serial.print("AT: ");
    Serial.println(cmd);
    
    // Clear buffer
    while (loraSerial.available()) {
        loraSerial.read();
    }
    
    loraSerial.println(cmd);
    
    unsigned long start = millis();
    while (millis() - start < timeout) {
        if (loraSerial.available()) {
            char c = loraSerial.read();
            Serial.write(c);
            result += c;
        }
    }
    
    return result;
}