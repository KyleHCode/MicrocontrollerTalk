#include "LoRaModule.h"

LoRaModule::LoRaModule(uint8_t rxPin, uint8_t txPin, uint8_t address) 
    : _rxPin(rxPin), _txPin(txPin), _address(address) {
#if defined(__IMXRT1062__)
    // Teensy 4.x: use Serial1, ignore pins
    // loraSerial is reference to Serial1
#else
    loraSerial = new HardwareSerial(1);  // Use UART1
#endif
}

bool LoRaModule::begin() {
#if defined(__IMXRT1062__)
    loraSerial.begin(115200);
    delay(1000);
    Serial.println("Initializing LoRa Module...");
#else
    loraSerial->begin(115200, SERIAL_8N1, _rxPin, _txPin);
    delay(1000);
    Serial.println("Initializing LoRa Module...");
#endif
    if (sendATCommand("AT").indexOf("OK") != -1) {
        Serial.println("LoRa module responding");
        return true;
    } else {
        Serial.println("WARNING: LoRa module not responding!");
        return false;
    }
}

bool LoRaModule::configure(uint8_t address, unsigned long band, uint8_t networkId) {
    char cmd[50];
    
    // Set address
    sprintf(cmd, "AT+ADDRESS=%d", address);
    sendATCommand(cmd);
    delay(100);
    
    // Set band
    sprintf(cmd, "AT+BAND=%lu", band);
    sendATCommand(cmd);
    delay(100);
    
    // Set network ID
    sprintf(cmd, "AT+NETWORKID=%d", networkId);
    sendATCommand(cmd);
    delay(100);
    
    Serial.println("LoRa configured: Address=" + String(address) + 
                   ", Band=" + String(band) + 
                   ", NetworkID=" + String(networkId));
    return true;
}

String LoRaModule::sendATCommand(const char* command, unsigned long timeout) {
    String result = "";
#if defined(__IMXRT1062__)
    while (loraSerial.available()) {
        loraSerial.read();
    }
    loraSerial.println(command);
#else
    while (loraSerial->available()) {
        loraSerial->read();
    }
    loraSerial->println(command);
#endif
    unsigned long startTime = millis();
    unsigned long lastCharTime = millis();
    while (millis() - startTime < timeout) {
#if defined(__IMXRT1062__)
        if (loraSerial.available()) {
            char c = loraSerial.read();
#else
        if (loraSerial->available()) {
            char c = loraSerial->read();
#endif
            result += c;
            lastCharTime = millis();
            if ((result.indexOf("OK") != -1 || result.indexOf("ERROR") != -1) && 
                millis() - lastCharTime > 50) {
                break;
            }
        }
    }
    Serial.print("Response: ");
    Serial.println(result);
    return result;
}

bool LoRaModule::sendData(uint8_t destAddress, String hexData) {
    char cmd[100];
    sprintf(cmd, "AT+SEND=%d,%d,%s", destAddress, hexData.length() / 2, hexData.c_str());
    String response = sendATCommand(cmd, 2000);
    return response.indexOf("OK") != -1;
}

bool LoRaModule::receiveData(String& hexData) {
#if defined(__IMXRT1062__)
    if (!loraSerial.available()) {
        return false;
    }
#else
    if (!loraSerial->available()) {
        return false;
    }
#endif
    String incomingString = "";
    // Read with timeout
    unsigned long startTime = millis();
    unsigned long lastCharTime = millis();
    while (millis() - startTime < 1000) {
#if defined(__IMXRT1062__)
        if (loraSerial.available()) {
            char c = loraSerial.read();
#else
        if (loraSerial->available()) {
            char c = loraSerial->read();
#endif
            if (c == '\n') break;
            incomingString += c;
            lastCharTime = millis();
        } else if (millis() - lastCharTime > 50) {
            // No data for 50ms, likely complete
            break;
        }
    }
    // Parse format: +RCV=<address>,<length>,<data>,<RSSI>,<SNR>
    int firstComma = incomingString.indexOf(',');
    int secondComma = incomingString.indexOf(',', firstComma + 1);
    
    if (firstComma > 0 && secondComma > 0) {
        hexData = incomingString.substring(secondComma + 1);
        // Remove RSSI and SNR if present
        int thirdComma = hexData.indexOf(',');
        if (thirdComma > 0) {
            hexData = hexData.substring(0, thirdComma);
        }
        hexData.trim();
        return true;
    }
    
    return false;
}
