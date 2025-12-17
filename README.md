# 🌍 ESP32 GSM + LoRa Environmental Monitoring Node

A **production-ready, low-power environmental monitoring system** built on ESP32, designed for **remote field deployment**, **solar operation**, and **long-range communication** using **LoRa + GSM**.

This repository contains the **complete working firmware**, hardware references, and documentation required to deploy the node in real-world conditions such as **landslide monitoring, weather stations, agriculture, and remote sensing**.

---

## 🚀 Key Highlights

✅ Multi-sensor environmental data acquisition  
✅ LoRa long-range wireless transmission  
✅ GSM (SIM7670) for GNSS + cloud fallback  
✅ Deep-sleep optimized for solar & battery use  
✅ Interrupt-based wakeup (rain + motion)  
✅ Field-tested architecture (not a demo project)

---

## 🧠 System Capabilities

### 📡 Communication
- **LoRa SX1278** – primary long-range data link  
- **SIM7670 GSM / LTE**  
  - GNSS (location + time)
  - Cloud / server fallback
  - Remote diagnostics

### 🌱 Sensors Supported
- **MPU6050** – motion / vibration (interrupt-based)
- **BMP180** – barometric pressure & temperature
- **BH1750** – ambient light intensity
- **DHT22** – temperature & humidity
- **Soil Moisture Sensor** – analog input
- **Rain Gauge (Tipping Bucket)** – pulse counter (RTC GPIO)

---

## 🔋 Power & Reliability
- ESP32 **deep sleep** between measurements
- Wake-up sources:
  - 🌧️ Rain tipping interrupt
  - 🧭 Motion (MPU6050 interrupt)
  - ⏱️ Timer-based periodic wake
- Designed for **solar-powered unattended operation**

---

---

## 🧪 Firmware Overview

The firmware implements:
- Sensor polling with sanity checks
- Interrupt-driven rainfall counting
- GNSS data acquisition via GSM module
- LoRa packet formation & transmission
- GSM fallback when LoRa is unavailable
- Robust deep sleep state management

> ⚠️ This is **not a sample or tutorial sketch** — it is a **field-deployable firmware**.

---

## 🛠 Build & Flash

### Using PlatformIO
```bash
pio run
pio run -t upload
pio device monitor

## 📁 Repository Structure

