# SmartShunt-BLE (ESP32-C6 Edition) 🔋

A high-precision, wireless Battery Management and Monitoring system based on the **ESP32-C6** and **INA226** sensor. Designed for real-time tracking of current, voltage, and capacity (Coulomb Counter) with BLE data transmission.

Initially developed for 6S Li-ion setups, it is easily configurable for 12V Lead-Acid (Automotive) or other battery chemistries.

## 🚀 Features

- **High-Current Monitoring:** Integrated support for a **300A Shunt** (0.25mΩ).
- **Precision Metrology:** Uses INA226 for high-side or low-side sensing with digital filtering (EMA).
- **Coulomb Counting:** Tracks Ah (Ampere-hours) with automatic session detection.
- **Smart Learning Mode:** Calibrates battery capacity by tracking a full charge cycle.
- **BLE Interface:** Real-time data streaming and remote configuration via Bluetooth Low Energy.
- **Persistence:** Automatic state saving (SoC, capacity, offsets) to Non-Volatile Storage (NVS).
- **ESP32-C6 Optimized:** Efficient power management and non-blocking I2C implementation.

## 🛠 Hardware Requirements

* **Microcontroller:** ESP32-C6 (supports BLE 5.0 and Matter/Zigbee).
* **Sensor:** INA226 High-Side/Low-Side Bi-Directional Current/Power Monitor.
* **Shunt:** 300A / 75mV (Resistance: 0.00025Ω).
* **I2C Connections (Default):** - SDA: `GPIO 20`
  - SCL: `GPIO 19`

## 📊 Data Protocol (BLE)

The device broadcasts a formatted string every second:
`V|mA|Ah|%|Status|Session|SinceFull|LastCharge|LastDischarge|Time`

**Example:**
`24.12V|1500mA|7.850Ah|98.1%|OK|S:0.1500|SF:0.150|LC:1.200|LD:0.850|T:00:15:20`

## ⚙️ Configuration & Commands

You can send commands via the **Configuration Characteristic** (`82250625...`):

| Command | Action |
|:---|:---|
| `sync:1` | Force Sync to 100% SoC |
| `sync:0` | Force Sync to 0% (Activates Learning Mode) |
| `cap:60.0` | Set Battery Nominal Capacity (e.g., 60Ah for Corolla) |
| `volt:14.2` | Set Full Charge Voltage threshold |

## 🔧 Installation

1. Install **ESP32 Arduino Core** (v3.0+ recommended for C6 support).
2. Install libraries: `INA226_WE`, `BLEDevice`.
3. Open `SmartShuntBLE.ino` and verify your I2C pins.
4. Flash the firmware.

## ⚠️ Safety Note

This project is designed to handle high currents (up to **300A**). Ensure all high-power connections use appropriate gauge wire (0AWG or similar) and that the shunt is securely mounted. Improper wiring can lead to fire or equipment damage.

---
**Developed by fobaty** 
