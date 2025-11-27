# 🚀 SkyLord - Flight Computer

**SkyLord** is a flight computer system developed for the model rocket we used in the Teknofest Mid-Altitude Rocket Competition as the Aeronis Aerospace team. [...]

![Zlink88](https://github.com/user-attachments/assets/6f41cb7d-4e2b-48e0-b7b9-aa0fc39d0f60)

---

## 📋 Table of Contents

- [Features](#-features)
- [Hardware Specifications](#️-hardware-specifications)
- [Software Architecture](#-software-architecture)
- [Sensors and Modules Used](#-sensors-and-modules-used)
- [Setup](#-setup)
- [Build and Flash](#️-build-and-flash)
- [System Modes](#-system-modes)
- [Flight Algorithms](#-flight-algorithms)
- [Pin Configuration](#-pin-configuration)
- [Telemetry Packet](#-telemetry-packet)
- [Contributing](#-contributing)
- [License](#-license)
- [Contact](#-contact)

---

![ZLink55](https://github.com/user-attachments/assets/2065dcf8-8668-46a6-8ab4-cea2bcc864cb)

## ✨ Features

### 🎯 Core Features
- ✅ **Real-Time Flight Tracking**: High-frequency data acquisition with a 10 ms period
- ✅ **Advanced Sensor Fusion**: Mahony and Kalman filter algorithms
- ✅ **Flight Phase Detection**: Automatic detection of launch, ascent, apogee, and landing phases
- ✅ **LoRa Telemetry**: Long-range wireless data transmission (1+ km)
- ✅ **GPS Integration**: Position tracking with the L86 GNSS module
- ✅ **SD Card Data Logging**: High-speed logging using FAT filesystem
- ✅ **Reset Protection**: Critical data preserved using Backup SRAM
- ✅ **Multiple Test Modes**: SIT (System Integration Test) and SUT (Algorithm Validation)
- ✅ **Power Management**: Low-voltage detection and power-saving modes
- ✅ **Recovery System Control**: Support for automatic parachute deployment

### 🔬 Sensor Capabilities
- **6-Axis IMU**: ±24g acceleration, ±2000°/s gyroscope range
- **Barometric Altitude**: ±0.5 m accuracy
- **Quaternion-Based Orientation**: Avoids gimbal lock
- **Automatic Calibration**: Initial offsets and base altitude setup

---

## 🛠️ Hardware Specifications

### Main Processor
- **MCU**: STM32F446RET6 (ARM Cortex-M4F @ 180 MHz)
- **Flash**: 512 KB
- **RAM**: 128 KB
- **Backup SRAM**: 4 KB (for preserving critical data)

### Peripherals
| Peripheral | Count | Purpose |
|------------|-------|---------|
| I2C        | 2     | Sensor communication (BME280, BMI088) |
| UART       | 4     | GPS, LoRa, Telemetry, Debug |
| SPI        | 2     | SD Card, W25Q Flash |
| ADC        | 2     | Battery voltage and current measurement |
| Timer      | 1     | 10 ms periodic interrupt |
| DMA        | 5     | High-speed data transfer |
| RTC        | 1     | Timestamping |

---

## 📦 Sensors and Modules Used

### Sensors
| Sensor | Model | Interface | Purpose |
|--------|-------|-----------|---------|
| **IMU** | BMI088 | I2C (DMA) | 6-axis accelerometer and gyroscope |
| **Barometric Sensor** | BME280 | I2C | Pressure, temperature, humidity, altitude |
| **GPS** | L86 GNSS | UART5 (DMA) | Position, velocity, time |

### Communication Modules
| Module | Model | Interface | Range |
|--------|-------|-----------|-------|
| **LoRa** | E22-900T22S | UART2 (115200 baud) | 1+ km |
| **Debug UART** | - | UART4 (115200 baud) | Wired debug |

### Storage
- **SD Card**: FAT filesystem, high-speed logging
- **W25Q Flash**: Additional data storage (SPI)

---

## 🏗️ Software Architecture

### Module Structure

```
SkyLord/
├── Core/
│   ├── Src/
│   │   ├── main.c                 # Main program loop
│   │   ├── sensor_fusion.c        # Mahony/Kalman filter algorithms
│   │   ├── flight_algorithm.c     # Flight phase detection
│   │   ├── uart_handler.c         # Command handling
│   │   ├── packet.c               # Telemetry packet management
│   │   ├── data_logger.c          # SD card data logging
│   │   ├── reset_detect.c         # Reset detection and recovery
│   │   └── test_modes.c           # Test modes
│   │   ├── bme280.c               # BME280 driver implementation
│   │   ├── bmi088.c               # BMI088 driver implementation
│   │   ├── e22_lib.c              # LoRa module implementation
│   │   ├── l86_gnss.c             # GPS module implementation
│   │   ├── quaternion.c           # Quaternion math implementation
│   │   ├── filter.c               # Filter implementations
│   │   └── kalman.c               # Kalman filter implementation
│   └── Inc/
│       ├── bme280.h               # BME280 driver header
│       ├── bmi088.h               # BMI088 driver header
│       ├── e22_lib.h              # LoRa module header
│       ├── l86_gnss.h             # GPS module header
│       └── quaternion.h           # Quaternion math
│       ├── sensor_fusion.h        # Sensor fusion header
│       ├── flight_algorithm.h     # Flight algorithm header
│       ├── uart_handler.h         # UART command handler header
│       ├── packet.h               # Telemetry packet header
│       ├── data_logger.h          # Data logger header
│       ├── reset_detect.h         # Reset detection header
│       ├── test_modes.h           # Test modes header
│       ├── filter.h               # Filter header
│       └── kalman.h               # Kalman filter header
├── Drivers/                        # STM32 HAL drivers
├── FATFS/                          # FAT filesystem middleware
└── Middlewares/                    # USB and additional libraries
```

### Main Loop

```
Main Loop (continuous):
├── IMU Update (BMI088)
├── Barometric Sensor Update (BME280)
├── UART Command Handling
└── 100 ms Timer Interrupt:
    ├── GPS Update
    ├── Mode-Based Tasks:
    │   ├── NORMAL: Sensor fusion + Flight algorithm + Logging
    │   ├── SIT: Sensor integration tests
    │   └── SUT: Algorithm validation (synthetic data)
    ├── ADC Readings (voltage/current)
    └── Status LEDs update

1 Second Timer:
└── LoRa Telemetry Transmission + Backup SRAM Save
```

---

## 🚀 Flight Algorithms

### 1. Sensor Fusion
- **Mahony Filter**: Combines IMU and barometric data
- **Kalman Filter**: Optimal estimation for altitude and velocity
- **Quaternion Math**: 3D orientation without gimbal lock

### 2. Flight Phase Detection
| Phase | Detection Criteria | Action |
|-------|--------------------|--------|
| **Idle** | Acceleration < 40 m/s² | Base altitude setup |
| **Launch** | Acceleration > 40 m/s² | Start data logging |
| **Ascent** | Velocity > 0, positive acceleration | Track max altitude |
| **Apogee** | Velocity ≈ 0 | Trigger recovery system (optional) |
| **Descent** | Velocity < 0, altitude decreasing | Monitor descent rate |
| **Ground** | Altitude ≈ base altitude | Stop logging |

### 3. Reset Protection
- Reset detection using RTC timestamps
- Save critical data to Backup SRAM:
  - Sensor calibration data
  - Base altitude
  - Flight phase state
  - IMU offset values

---

## 🔌 Pin Configuration

### I2C Pins
| Pin | Function | Sensor |
|-----|----------|--------|
| PB8 | I2C1_SCL | BME280 |
| PB9 | I2C1_SDA | BME280 |
| PA8 | I2C3_SCL | BMI088 |
| PC9 | I2C3_SDA | BMI088 |

### UART Pins
| UART | TX | RX | Use |
|------|----|----|-----|
| UART4 | PA0 | PA1 | Debug/Command |
| UART5 | PC12 | PD2 | GPS (L86) |
| USART1 | PA9 | PA10 | Telemetry output |
| USART2 | PA2 | PA3 | LoRa (E22) |

### SPI Pins
| SPI | SCK | MISO | MOSI | CS | Use |
|-----|-----|------|------|----|-----|
| SPI1 | PA5 | PA6 | PA7 | PA4 | SD Card |
| SPI2 | PB13 | PB14 | PB15 | PB12 | W25Q Flash |

### GPIO Pins
| Pin | Function | Description |
|-----|----------|-------------|
| PC13 | CAMERA | Camera trigger |
| PA11 | RECOVERY1 | Parachute separation 1 |
| PA15 | RECOVERY2 | Parachute separation 2 |
| PB0 | BUZZER | Audible alert |
| PB10 | SGU_LED1 | Status LED 1 |
| PB11 | SGU_LED2 | Status LED 2 |
| PC8 | MCU_LED | MCU activity indicator |

### ADC Pins
| ADC | Channel | Pin | Measurement |
|-----|---------|-----|-------------|
| ADC1 | CH9 | PB1 | Battery voltage (0-13.2V) |
| ADC2 | CH8 | PB0 | Current sensor |

---

## 💻 Setup

### Requirements

#### Software
- **STM32CubeIDE** v1.8.0 or later
- **STM32CubeMX** (optional — project is preconfigured)
- **ST-Link Utility** or **OpenOCD** (for flashing firmware)

#### Hardware
- ST-Link V2 programmer
- USB-UART adapter (for debug)
- STM32F446RET6 development board or custom PCB

### Steps

1. **Clone the repository**
```bash
git clone https://github.com/halilsrky/SkyLord.git
cd SkyLord
```

2. **Open in STM32CubeIDE**
   - File → Open Projects from File System
   - Select the project folder
   - Click Finish

3. **Check Dependencies**
   - HAL libraries are included automatically
   - Ensure FATFS middleware is enabled

---

## ⚙️ Build and Flash

### Build

1. Open the project in STM32CubeIDE
2. Project → Build All (Ctrl+B)
3. It should compile without errors (aim for 0 errors and 0 warnings)

### Flash

#### Using ST-Link
1. Connect the ST-Link to your PC and board
2. Run → Debug (F11) or Run (Ctrl+F11)
3. Firmware will be flashed automatically

#### Using OpenOCD (alternative)
```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program SkyLord.elf verify reset exit"
```

---

## 📡 System Modes

SkyLord supports three operation modes:

### 1️⃣ MODE_NORMAL (Operational Mode)
- **Purpose**: Used during an actual flight
- **Features**:
  - All sensors active
  - Sensor fusion and flight algorithm running
  - Data logged to SD card
  - Telemetry sent via LoRa
  - GPS position updated
- **Start**: Via UART command (`MODE:NORMAL`)

### 2️⃣ MODE_SIT (System Integration Test)
- **Purpose**: Hardware and sensor testing
- **Features**:
  - Real sensor data is read
  - Data flow is verified
  - Calibration tests run
  - Communication modules tested
- **Start**: Via UART command (`MODE:SIT`)

### 3️⃣ MODE_SUT (System Unit Test)
- **Purpose**: Validate algorithms using synthetic data
- **Features**:
  - Uses simulated flight data
  - Flight algorithm validation
  - Kalman filter performance testing
- **Start**: Via UART command (`MODE:SUT`)

---

## 📊 Telemetry Packet

### Normal Packet Format (62 bytes)

| Byte | Field | Data Type | Unit | Description |
|------|-------|-----------|------|-------------|
| 0-3  | Timestamp | uint32_t | ms | System uptime |
| 4-7  | Latitude | float | degrees | GPS latitude (-90 to 90) |
| 8-11 | Longitude | float | degrees | GPS longitude (-180 to 180) |
| 12-15 | GPS Altitude | float | m | GPS altitude |
| 16-19 | Barometric Altitude | float | m | BME280 altitude |
| 20-23 | Vertical Speed | float | m/s | Kalman estimate |
| 24-27 | Accel X | float | m/s² | BMI088 |
| 28-31 | Accel Y | float | m/s² | BMI088 |
| 32-35 | Accel Z | float | m/s² | BMI088 |
| 36-39 | Gyro X | float | °/s | BMI088 |
| 40-43 | Gyro Y | float | °/s | BMI088 |
| 44-47 | Gyro Z | float | °/s | BMI088 |
| 48-51 | Angle | float | ° | Orientation |
| 52-55 | Voltage | float | V | ADC1 |
| 56-59 | Temperature | float | °C | BME280 |
| 60   | Flight Phase | uint8_t | - | 0-5 |
| 61   | CRC | uint8_t | - | Packet validation |

### SD Card Packet Format (64 bytes)
In addition to the normal packet:
- **62-63**: Additional status info (system health, error codes)

---

## 🧪 Testing and Validation

### Preflight Checklist

```
☐ BME280 sensor ID check (0x60)
☐ BMI088 accelerometer ID check (0x1E)
☐ BMI088 gyroscope ID check (0x0F)
☐ SD card connection test
☐ GPS fix obtained (outdoors, 1-2 min)
☐ LoRa range test (100m+)
☐ Battery voltage measurement verification
☐ Recovery system safety tests
```

### Calibration Procedure

1. **Horizontal Surface Calibration**
   - Place the board on a flat surface
   - Power on and wait 5 seconds
   - When the buzzer beeps once, calibration is complete

2. **Base Altitude Setup**
   - Power on the board at the launch site
   - Keep the board stationary for the first 10 seconds
   - Base altitude is recorded automatically

3. **GPS Fix**
   - Wait outdoors
   - Verify GPS data on the telemetry output

---

## 🔧 Troubleshooting

### Sensor Readings Are Zero
Possible causes:
- I2C connection issue
- Incorrect pull-up resistor values
- Insufficient power supply

Solutions:
1. Run an I2C scan (via debug UART)
2. Check SCL/SDA signals with an oscilloscope
3. Measure the 3.3V supply voltage

### LoRa Telemetry Not Sending
Possible causes:
- Low battery voltage (below 7V)
- Incorrect LoRa module configuration
- Antenna disconnected

Solutions:
1. Check battery voltage (minimum 7.4V recommended)
2. Restart the LoRa module (M0/M1 pins)
3. Measure antenna VSWR

### SD Card Not Logging
Possible causes:
- Not FAT32 formatted
- Corrupted SD card
- SPI communication issue

Solutions:
1. Format the SD card as FAT32 (max 32GB)
2. Try a different SD card
3. Inspect SPI signals with an oscilloscope

---

## 📞 Contact

**Project Owner**: Halil Sarıkaya

- GitHub: [@halilsrky](https://github.com/halilsrky)
- Email: halilsarikaya070@gmail.com
- LinkedIn: www.linkedin.com/in/halil-sarıkaya-3a777321b

---

## 📈 Project Status

**Version**: 1.0.0  
**Status**: 🟢 Active Development  
**Last Updated**: 03 October 2025

---

<p align="center">
  <strong>🚀 Fly Safe! 🚀</strong>
</p>
