/*
 LORA GATEWAY
 - Receives packets
 - Forwards to Serial / GSM / MQTT later
*/

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

#define LORA_SS   5
#define LORA_RST  14
#define LORA_DIO0 2

void setup() {
  Serial.begin(115200);
  SPI.begin();
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  LoRa.begin(433E6);
  Serial.println("LoRa Gateway Started");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }
    Serial.println();
  }
}
