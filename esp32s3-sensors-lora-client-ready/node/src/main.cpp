/*
 ESP32-S3 SENSOR NODE
 - Deep Sleep
 - Rain Interrupt
 - MPU6050 Interrupt
 - LoRa Transmitter
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_MPU6050.h>

#define RAIN_PIN        16   // RTC GPIO
#define MPU_INT_PIN     9
#define LORA_SS         5
#define LORA_RST        14
#define LORA_DIO0       2

Adafruit_MPU6050 mpu;

RTC_DATA_ATTR uint32_t bootCount = 0;

void IRAM_ATTR rainISR() {}
void IRAM_ATTR mpuISR() {}

void setup() {
  Serial.begin(115200);
  delay(500);

  bootCount++;
  Serial.printf("Boot #%lu\n", bootCount);

  pinMode(RAIN_PIN, INPUT_PULLUP);
  attachInterrupt(RAIN_PIN, rainISR, FALLING);

  pinMode(MPU_INT_PIN, INPUT);
  attachInterrupt(MPU_INT_PIN, mpuISR, RISING);

  Wire.begin();
  mpu.begin();

  SPI.begin();
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  LoRa.begin(433E6);

  LoRa.beginPacket();
  LoRa.print("WAKE EVENT");
  LoRa.endPacket();

  esp_sleep_enable_ext0_wakeup((gpio_num_t)RAIN_PIN, 0);
  esp_sleep_enable_ext1_wakeup(1ULL << MPU_INT_PIN, ESP_EXT1_WAKEUP_ANY_HIGH);

  Serial.println("Going to deep sleep");
  delay(100);
  esp_deep_sleep_start();
}

void loop() {}
