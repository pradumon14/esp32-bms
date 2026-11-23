# Battery Monitoring System (BMS) Using ESP32 and C Firmware

[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.x-blue.svg)](https://docs.espressif.com/projects/esp-idf/en/latest/)
[![Target](https://img.shields.io/badge/Target-ESP32-green.svg)](https://www.espressif.com/en/products/socs/esp32)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

An embedded single-cell Lithium-ion (Li-ion) Battery Monitoring System (BMS) engineered for the **ESP32 DevKit V1** microcontroller using **ESP-IDF v5.x** and native C firmware.

---

## 👤 Author Information

* **Author:** Pradumon Sahani
* **Program:** BS Electronics Engineering
* **Institution:** Indian Institute of Technology Madras (IIT Madras)
* **Roll No.:** `26T3Q1S2ESQ1S200903`
* **Date:** November 2026

---

## 📋 Project Overview

Rechargeable lithium-ion cells (3.7 V nominal, 4.2 V maximum) require strict voltage and thermal supervision to avoid premature capacity degradation, copper dissolution (under-discharge), and thermal runaway. 

This project delivers a complete, modular, and self-contained battery monitoring solution that:
1. **Conditions Battery Voltage:** Steps down the 3.00 V – 4.20 V cell terminal voltage using a precision $100\text{ k}\Omega / 100\text{ k}\Omega$ resistive voltage divider ($2.0\times$ division) buffered with a $100\text{ nF}$ filter capacitor ($f_c \approx 31.8\text{ Hz}$, $21\,\mu\text{A}$ max quiescent drain).
2. **Quantizes & Calibrates ADC:** Samples ADC1 Channel 6 (GPIO34, input-only) at 12-bit resolution and 12 dB attenuation with factory eFuse line-fitting calibration and 64-sample digital averaging.
3. **Estimates State of Charge (SOC):** Evaluates battery charge percentage via an 11-point resting Open-Circuit Voltage (OCV) lookup table with piecewise linear interpolation and boundary clamping.
4. **Acquires Digital Temperature:** Reads cell surface temperature from a Maxim DS18B20 1-Wire digital thermometer ($0.0625\,^\circ\text{C}$ resolution) with 8-bit CRC scratchpad verification.
5. **Evaluates Safety Alerts with Hysteresis:** Manages low-battery ($\le 3.30\text{ V}$ or $\le 15.0\%$, clears $\ge 3.40\text{ V}$ and $\ge 18.0\%$) and high-temperature ($\ge 45.0\,^\circ\text{C}$, clears $\le 43.0\,^\circ\text{C}$) states to prevent alert chatter.
6. **Renders Telemetry Locally:** Displays live metrics and status flags on an SSD1306 $128 \times 64$ monochrome OLED display over I2C at 400 kHz with custom 5x7 font rendering, while logging telemetry over UART.
7. **Maintains Deterministic Timing:** Operates on an exact FreeRTOS `vTaskDelayUntil()` 2000 ms periodic supervision loop.

---

## ⚙️ System Architecture

```
   +------------------------------------+
   |  3.7 V Li-ion Battery (3.0 - 4.2V) |
   +-----------------+------------------+
                     |
                     v
   +------------------------------------+
   | Voltage Divider (100k / 100k) +    |
   | Bypass Filter Capacitor (100 nF)   |
   +-----------------+------------------+
                     | V_adc (1.5 - 2.1V)
                     v
+--------------------+---------------------------------------+
|  ESP32 DevKit V1 (ESP-IDF v5.x / FreeRTOS C Firmware)      |
|                                                            |
|  * ADC1_CH6 (GPIO34): 64x Avg + Line-Fit eFuse Calibration |
|  * Voltage Reconstruction: V_bat = V_adc * (R1 + R2) / R2  |
|  * SOC Estimation: 11-Point OCV-SOC Linear Interpolation   |
|  * 1-Wire Driver (GPIO4): DS18B20 Read + CRC-8 Validation  |
|  * Safety Engine: Dual-Threshold Hysteresis Logic          |
|  * FreeRTOS Task: Deterministic 2000 ms DelayUntil cadence |
+--------------------+---------------------+-----------------+
                     |                     |
     I2C (GPIO21/22) |                     | UART (USB)
                     v                     v
   +-----------------------------+   +-----------------------+
   | SSD1306 128x64 OLED Display |   | Serial Telemetry Log  |
   | (Voltage, SOC, Temp, Alert) |   | (ESP_LOGI / ESP_LOGW) |
   +-----------------------------+   +-----------------------+
```

---

## 🔌 Hardware Pin Mapping & Circuit Connections

| Peripheral / Signal | Peripheral Pin | ESP32 DevKit V1 Pin | Description / Electrical Specification |
| :--- | :--- | :--- | :--- |
| **Battery Sense** | $V_{\text{adc}}$ | `GPIO34` (ADC1_CH6) | Voltage divider midpoint ($1.50\text{ V} - 2.10\text{ V}$) |
| **Battery Negative**| `GND` | `GND` | Common system ground reference |
| **DS18B20 VDD** | `VDD` | `3V3` | Dedicated 3.3 V regulated power rail |
| **DS18B20 Data** | `DQ` | `GPIO4` | Open-drain 1-Wire bus with external $4.7\text{ k}\Omega$ pull-up |
| **DS18B20 GND** | `GND` | `GND` | Common ground reference |
| **SSD1306 VCC** | `VCC` | `3V3` | 3.3 V logic and panel supply |
| **SSD1306 GND** | `GND` | `GND` | Common ground reference |
| **SSD1306 SDA** | `SDA` | `GPIO21` | I2C0 Serial Data (400 kHz Fast-Mode) |
| **SSD1306 SCL** | `SCL` | `GPIO22` | I2C0 Serial Clock (400 kHz Fast-Mode) |

### Voltage Divider & Filter Schematic

```
(+ V_battery: 3.00 V - 4.20 V)
         |
         +----+
              |
             [R1] 100 kOhm, 1%
              |
              +--------+----------> V_adc to GPIO34 (ADC1_CH6)
              |        |            (1.50 V - 2.10 V)
             [R2]     [Cf] 100 nF
         100 kOhm, 1%  |  Ceramic
              |        |
         +----+--------+
         |
      ( GND ) Common Ground
```

---

## 📐 Mathematical Formulation

### 1. Voltage Reconstruction
Given divider resistances $R_1 = 100\text{ k}\Omega$ and $R_2 = 100\text{ k}\Omega$:
$$V_{\text{battery}} = V_{\text{adc}} \times \frac{R_1 + R_2}{R_2} = 2.0 \times V_{\text{adc}}$$

### 2. ADC Oversampling and Noise Attenuation
Sampling $N = 64$ points reduces uncorrelated Gaussian noise standard deviation by:
$$\sigma_{\text{avg}} = \frac{\sigma_{\text{raw}}}{\sqrt{N}} = \frac{\sigma_{\text{raw}}}{8}$$

### 3. Piecewise Linear SOC Interpolation
Given adjacent lookup table entries $(V_1, \text{SOC}_1)$ and $(V_2, \text{SOC}_2)$ bracketing measured $V_{\text{battery}}$:
$$\text{SOC} = \text{SOC}_1 + \left(\frac{V_{\text{battery}} - V_1}{V_2 - V_1}\right) \times (\text{SOC}_2 - \text{SOC}_1)$$

#### 11-Point Resting OCV-SOC Lookup Table

| $V_{\text{battery}}$ (V) | SOC (%) | $V_{\text{battery}}$ (V) | SOC (%) |
| :---: | :---: | :---: | :---: |
| $\le 3.00$ | 0.0 | 3.84 | 60.0 |
| 3.50 | 10.0 | 3.93 | 70.0 |
| 3.61 | 20.0 | 4.02 | 80.0 |
| 3.66 | 30.0 | 4.11 | 90.0 |
| 3.70 | 40.0 | $\ge 4.20$ | 100.0 |
| 3.74 | 50.0 | — | — |

---

## 📁 Repository Structure

```
esp32-bms/
├── CMakeLists.txt          # Root ESP-IDF CMake project definition
├── sdkconfig.defaults      # Default configuration (IDF target, tick rate, ADC cali)
├── main/
│   ├── CMakeLists.txt      # Component build script
│   └── main.c              # Complete modular ESP-IDF v5.x C firmware
├── esp32.c                 # Standalone C firmware copy for portable reference
├── BMS project.pdf         # Compiled IIT Madras technical design report (PDF)
├── main.tex                # Comprehensive LaTeX technical report source
├── reference.bib           # BibTeX bibliography with IEEE citations
├── .gitignore              # Ignores build artifacts and intermediate LaTeX files
└── README.md               # Complete project documentation
```

---

## 🛠️ Build and Flashing Instructions

### Prerequisites
* [ESP-IDF v5.x](https://docs.espressif.com/projects/esp-idf/en/v5.3/esp32/get-started/) installed and sourced in your environment.
* USB cable connected to ESP32 DevKit V1.

### 1. Build and Flash Firmware
```bash
# Clone the repository
git clone https://github.com/pradumon14/esp32-bms.git
cd esp32-bms

# Set target to classic ESP32
idf.py set-target esp32

# Build the project
idf.py build

# Flash to device and open serial monitor (replace /dev/ttyUSB0 with your port)
idf.py -p /dev/ttyUSB0 flash monitor
```

### 2. Compile the Technical Report (LaTeX)
To build the 32-page technical proposal with citations:
```bash
pdflatex -interaction=nonstopmode main.tex
bibtex main
pdflatex -interaction=nonstopmode main.tex
pdflatex -interaction=nonstopmode main.tex
```
The output file is `main.pdf` (and synced to `BMS project.pdf`).

---

## 🧪 Planned Validation Protocol

| Test Case | Stimulus | Acceptance Criterion | Status |
| :--- | :--- | :--- | :---: |
| **Voltage Accuracy Sweep** | Bench PSU (3.00 V to 4.20 V in 0.1 V steps) | $\|V_{\text{ESP32}} - V_{\text{DMM}}\| \le 30\text{ mV}$ | Planned |
| **SOC Interpolation** | Mid-interval test points (e.g. 3.78 V $\to$ 54.0%) | Matches mathematical model within $\pm 0.5\%$ | Planned |
| **Low-Battery Hysteresis**| Sweep 3.60 V $\to$ 3.25 V $\to$ 3.45 V | Triggers $\le 3.30\text{ V}$, resets $\ge 3.40\text{ V}$ | Planned |
| **High-Temp Hysteresis**  | Heat probe $>45\,^\circ\text{C}$, cool $<43\,^\circ\text{C}$ | Triggers $\ge 45\,^\circ\text{C}$, clears $\le 43\,^\circ\text{C}$ | Planned |
| **1-Wire Disconnect Fault**| Unplug DS18B20 DQ pin | Displays `T:ERR`, voltage loop continues safely | Planned |

---

## 📄 License

This project is licensed under the [MIT License](LICENSE).
Academic Technical Proposal submitted for BS Electronics Engineering at the Indian Institute of Technology Madras (IIT Madras).
