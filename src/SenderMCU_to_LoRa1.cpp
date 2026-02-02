// Sender: MCU1 -> LoRa1 -> LoRa2 -> MCU2 -> Relays
// Sends relay control commands as hex over LoRa

#include <Arduino.h>
#include "lora_config.h"

// Pin configuration for XIAO ESP32S3 sender
#define RX_PIN 44  // GPIO44 D7 (RX on XIAO) - connects to LoRa TX
#define TX_PIN 43  // GPIO43 D6 (TX on XIAO) - connects to LoRa RX

HardwareSerial loraSerial(1);

// Function prototypes
String sendATCommand(const char* cmd, unsigned long timeout);
void sendRelayCommand(uint16_t relayState);
void setupLoRa();

void setup() {
  Serial.begin(115200);
  delay(100);
  
  loraSerial.begin(LORA_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(LORA_INIT_DELAY);
  
  setupLoRa();
  
  Serial.println("\n=== Relay Command Sender ===");
  Serial.println("Commands:");
  Serial.println("  1-6: Toggle relay 1-6");
  Serial.println("  0: All relays OFF");
  Serial.println("  a: All relays ON");
  Serial.println("  h: Send hex directly (e.g., h8400)");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.length() == 0) return;
    
    char cmd = input[0];
    static uint16_t currentState = RELAY_MSB_BIT;  // Start with MSB set, all relays off
    
    if (cmd >= '1' && cmd <= '6') {
      // Toggle specific relay
      uint8_t relay = cmd - '1';
      currentState ^= (1 << (relay + RELAY_BIT_START));  // Toggle relay bits
      Serial.println("Toggling relay " + String(relay + 1));
      sendRelayCommand(currentState);
      
    } else if (cmd == '0') {
      // All OFF
      currentState = RELAY_MSB_BIT;  // Only MSB set
      Serial.println("All relays OFF");
      sendRelayCommand(currentState);
      
    } else if (cmd == 'a') {
      // All ON
      currentState = RELAY_MSB_BIT | 0x7E00;  // MSB + bits 9-14
      Serial.println("All relays ON");
      sendRelayCommand(currentState);
      
    } else if (cmd == 'h' && input.length() > 1) {
      // Send hex directly
      String hexStr = input.substring(1);
      uint16_t value = (uint16_t)strtol(hexStr.c_str(), NULL, 16);
      Serial.println("Sending hex: " + hexStr);
      sendRelayCommand(value);
      currentState = value;
      
    } else {
      Serial.println("Unknown command");
    }
    
    Serial.println("Current state: " + String(currentState, HEX) + " (" + String(currentState, BIN) + ")");
  }
}

void setupLoRa() {
  Serial.println("Initializing LoRa Sender...");
  
  // Verify module responding
  if (sendATCommand(AT_TEST, AT_COMMAND_TIMEOUT).indexOf("OK") != -1) {
    Serial.println("LoRa module responding");
  } else {
    Serial.println("WARNING: LoRa not responding!");
  }
  
  // Configure as sender
  char cmd[50];
  sprintf(cmd, AT_SET_ADDRESS_FMT, LORA_SENDER_ADDRESS);
  sendATCommand(cmd, AT_COMMAND_TIMEOUT);
  delay(LORA_CONFIG_DELAY);
  
  sprintf(cmd, AT_SET_BAND_FMT, LORA_BAND);
  sendATCommand(cmd, AT_COMMAND_TIMEOUT);
  delay(LORA_CONFIG_DELAY);
  
  sprintf(cmd, AT_SET_NETWORK_FMT, LORA_NETWORK_ID);
  sendATCommand(cmd, AT_COMMAND_TIMEOUT);
  delay(LORA_INIT_DELAY);
  
  Serial.println("LoRa configured: Address " + String(LORA_SENDER_ADDRESS) + ", Network " + String(LORA_NETWORK_ID));
}

void sendRelayCommand(uint16_t relayState) {
  // Convert to 4-character hex string (e.g., 8400, A200)
  char hexStr[5];
  sprintf(hexStr, "%04X", relayState);
  
  // Send: AT+SEND=<dest>,<length>,<data>
  char atCmd[50];
  sprintf(atCmd, AT_SEND_FMT, LORA_RECEIVER_ADDRESS, 4, hexStr);
  
  Serial.print("Sending to LoRa: ");
  Serial.println(atCmd);
  
  loraSerial.println(atCmd);
  
  // Wait for response
  delay(500);
  while (loraSerial.available()) {
    Serial.write(loraSerial.read());
  }
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
