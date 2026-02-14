// Teensy 4.1: SPI slave receives 2 bytes from master and forwards over LoRa

#include <Arduino.h>
#include <SPI.h>
#include <SPISlave_T4.h>

#include "lora_config.h"
#include "LoRaModule.h"

// LoRa UART pins (Teensy 4.1 Serial1: RX=0, TX=1)
#define LORA_RX_PIN 0
#define LORA_TX_PIN 1

// LoRa addressing
#define LORA_RETURN_ADDRESS 3
#define LORA_RETURN_DEST_ADDRESS 6

// SPI pins (Teensy 4.1 default SPI pins)
#define SPI_CS_PIN 10
#define SPI_MOSI_PIN 11
#define SPI_MISO_PIN 12
#define SPI_SCK_PIN 13

// SPI slave instance (8-bit)
SPISlave_T4<&SPI, SPI_8_BITS> spiSlave;

// SPI receive buffer (2 bytes per frame)
volatile uint8_t spiRxBuf[2] = {0, 0};
volatile uint8_t spiRxCount = 0;
volatile bool spiFrameReady = false;

// LoRa module instance (UART1)
LoRaModule lora(LORA_RX_PIN, LORA_TX_PIN, LORA_RETURN_ADDRESS);

bool sendLoRaHex(uint8_t destAddress, uint16_t value);

void spiReceiveISR() {
	while (spiSlave.available()) {
		uint32_t data = spiSlave.popr();
		if (spiRxCount < 2) {
			spiRxBuf[spiRxCount++] = static_cast<uint8_t>(data & 0xFF);
		}
	}

	if (spiRxCount >= 2) {
		spiFrameReady = true;
	}
}

void setup() {
	Serial.begin(115200);
	delay(200);

	Serial.println("=== Teensy SPI Slave -> LoRa Return ===");

	// Initialize LoRa module
	if (lora.begin()) {
		lora.configure(LORA_RETURN_ADDRESS, LORA_BAND, LORA_NETWORK_ID);
		Serial.println("✓ LoRa configured");
	} else {
		Serial.println("✗ LoRa configuration failed");
	}

	// Initialize SPI slave
	pinMode(SPI_CS_PIN, INPUT_PULLUP);
	pinMode(SPI_MISO_PIN, OUTPUT);
	spiSlave.onReceive(spiReceiveISR);
	spiSlave.begin();
	Serial.println("✓ SPI slave initialized");
}

void loop() {
	if (spiFrameReady) {
		uint8_t hi = 0;
		uint8_t lo = 0;

		noInterrupts();
		hi = spiRxBuf[0];
		lo = spiRxBuf[1];
		spiRxCount = 0;
		spiFrameReady = false;
		interrupts();

		uint16_t receivedBytes = (static_cast<uint16_t>(hi) << 8) | lo;
		Serial.print("SPI received: 0x");
		Serial.println(receivedBytes, HEX);

		if (sendLoRaHex(LORA_RETURN_DEST_ADDRESS, receivedBytes)) {
			Serial.println("✓ LoRa return sent");
		} else {
			Serial.println("✗ LoRa return failed");
		}
	}
}

bool sendLoRaHex(uint8_t destAddress, uint16_t value) {
	char hexStr[5];
	sprintf(hexStr, "%04X", value);
	return lora.sendData(destAddress, String(hexStr));
}
