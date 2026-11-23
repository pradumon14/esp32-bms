# Battery Monitoring System (BMS) Using ESP32 & Native C Firmware

[![Framework: ESP-IDF v5.x](https://img.shields.io/badge/Framework-ESP--IDF%20v5.x-blue.svg)](https://docs.espressif.com/projects/esp-idf/en/latest/)
[![Target: ESP32 DevKit V1](https://img.shields.io/badge/Target-ESP32%20DevKit%20V1-red.svg)](https://www.espressif.com/en/products/socs/esp32)
[![Language: C99](https://img.shields.io/badge/Language-C99-brightgreen.svg)](https://en.wikipedia.org/wiki/C99)
[![Academic: IIT Madras](https://img.shields.io/badge/Institution-IIT%20Madras-orange.svg)](https://www.iitm.ac.in/)

> **Electronics Project Proposal and Technical Design Specification**  
> **Author:** Pradumon Sahani  
> **Program:** BS Electronics Engineering  
> **Institution:** Indian Institute of Technology Madras (IIT Madras)  
> **Roll Number:** `26T3Q1S2ESQ1S200903`  
> **Date:** November 2026  

---

## Executive Summary

Rechargeable single-cell Lithium-ion (Li-ion) batteries (3.7 V nominal, 4.2 V peak) are the standard power source for portable consumer gadgets, IoT sensor nodes, and embedded prototypes. Operating a cell outside its safe electrical and thermal boundary causes rapid irreversible capacity decay, internal copper dissolution below 3.0 V, and thermal runaway hazards above 45 °C.

In this project, I have developed the complete hardware interfacing and modular C firmware architecture for an embedded **Battery Monitoring System (BMS)** built around the **ESP32 DevKit V1** microcontroller. The firmware is developed natively using **Espressif IoT Development Framework (ESP-IDF v5.x)** on top of FreeRTOS.

The system performs:
1. **Safe Analog Signal Conditioning:** Attenuates terminal voltage via a low-drain $100\text{ k}\Omega / 100\text{ k}\Omega$ voltage divider ($2.0\times$) with a $100\text{ nF}$ ceramic filter.
2. **Calibrated 12-Bit SAR ADC Acquisition:** Samples ADC1 Channel 6 (GPIO34) using eFuse line-fitting calibration and 64-sample digital averaging to suppress noise.
3. **State of Charge (SOC) Estimation:** Computes charge percentage using an 11-point resting Open-Circuit Voltage (OCV) lookup table with piecewise linear interpolation.
4. **Digital Cell Surface Temperature Sensing:** Queries a Maxim DS18B20 1-Wire temperature sensor with 8-bit CRC validation at $0.0625\,^\circ\text{C}$ resolution.
5. **Hysteresis Alert Evaluation:** Latches low-battery ($\le 3.30\text{ V}$ or $\le 15\%$) and high-temperature ($\ge 45.0\,^\circ\text{C}$) alarms with hysteresis to eliminate switching chatter.
6. **Local Telemetry & UART Logging:** Drives an SSD1306 $128 \times 64$ monochrome OLED over I2C at 400 kHz with a custom 5x7 font while streaming diagnostic logs over UART.
7. **Jitter-Free Periodic Supervision:** Operates on an exact 2000 ms periodic schedule using FreeRTOS `vTaskDelayUntil()`.

---

## Engineering Problem Statement

Small-scale embedded devices powered by single-cell Li-ion batteries typically suffer from four critical challenges:

1. **Voltage Level Incompatibility:** A fully charged Li-ion cell reaches 4.20 V, directly exceeding the 3.3 V absolute maximum rating of ESP32 GPIOs and ADC inputs.
2. **ADC Non-Linearity & Reference Variations:** The ESP32 12-bit SAR ADC exhibits non-linear response at input rails and chip-to-chip reference voltage spread ($V_{\text{ref}} \approx 1100\text{ mV} \pm 100\text{ mV}$), requiring hardware division into the linear zone (150 mV – 2450 mV) and factory eFuse calibration.
3. **Flat Discharge Plateau (Non-Linear SOC Mapping):** A Li-ion cell does not discharge linearly; its voltage plateaus between 3.60 V and 3.90 V. Naive two-point linear scaling yields substantial state-of-charge error.
4. **Thermal Safety Blindness:** Discharging or charging Li-ion cells without surface temperature tracking risks thermal degradation. Robust digital CRC verification is essential to prevent false alarms from corrupted bus reads.

---

## System Architecture

```
                      +-----------------------------------+
                      |   3.7 V Li-ion Battery (3.0-4.2V) |
                      +-----------------+-----------------+
                                        |
                                        v
                      +-----------------------------------+
                      |  Resistive Divider (100k / 100k)  |
                      |  + Bypass Capacitor (100 nF)      |
                      +-----------------+-----------------+
                                        | V_adc (1.50 V - 2.10 V)
                                        v
+--------------------------------------------------------------------------------+
| ESP32 DevKit V1 (ESP-IDF v5.x / FreeRTOS C Firmware)                           |
|                                                                                |
|  [ADC1_CH6 (GPIO34)] --> 64x Sample Averaging --> eFuse Line-Fit Calibration   |
|                                        |                                       |
|                                        v                                       |
|  [Voltage Reconstruction] --------------> V_battery = V_adc * (R1 + R2) / R2   |
|                                        |                                       |
|                                        v                                       |
|  [SOC Estimator] -----------------------> 11-Point Piecewise Linear LUT        |
|                                        |                                       |
|  [1-Wire Driver (GPIO4)] ---------------> DS18B20 Read + 8-Bit CRC Check       |
|                                        |                                       |
|                                        v                                       |
|  [Safety Engine] -----------------------> Dual-Threshold Hysteresis Logic      |
|                                        |                                       |
|  [Periodic Task Engine] ----------------> vTaskDelayUntil (Exact 2000 ms Loop) |
+----------------------------------------+---------------------------------------+
                     |                                       |
                     | I2C (GPIO21/22)                       | UART Console (USB)
                     v                                       v
    +---------------------------------+     +----------------------------------+
    | SSD1306 128x64 I2C OLED Display |     | Serial Diagnostics               |
    | (V, SOC%, Temp, Alert Status)   |     | (ESP_LOGI / ESP_LOGW Telemetry)  |
    +---------------------------------+     +----------------------------------+
```

---

## Hardware Pin Mapping & Circuit Schematic

### Pin Allocation Table

| Signal / Net | Peripheral Terminal | ESP32 Pin | Mode / Configuration | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **Battery Voltage Sense** | Divider Tap ($V_{\text{adc}}$) | `GPIO34` | ADC1 Channel 6 | Input-only pin; 12-bit SAR, 12 dB attenuation |
| **System Ground** | Battery $(-)$ | `GND` | Ground Reference | Common ground plane across all modules |
| **DS18B20 Power** | VDD | `3V3` | Power Output | 3.3 V rail from onboard LDO |
| **DS18B20 Data** | DQ | `GPIO4` | Open-Drain Bidirectional | External $4.7\text{ k}\Omega$ pull-up to 3.3 V rail |
| **DS18B20 Ground** | GND | `GND` | Ground Reference | Common system ground |
| **SSD1306 Power** | VCC / GND | `3V3` / `GND` | Power Output / Ground | 3.3 V logic and display controller supply |
| **SSD1306 SDA** | SDA | `GPIO21` | I2C0 Master Data | 400 kHz Fast-Mode with internal pull-up |
| **SSD1306 SCL** | SCL | `GPIO22` | I2C0 Master Clock | 400 kHz Fast-Mode with internal pull-up |

### Signal Conditioning Circuit Schematic

```
(+ V_battery: 3.00 V - 4.20 V)
         |
         +----+
              |
             [R1] 100 kOhm, 1% Metal Film
              |
              +--------+----------> V_adc to GPIO34 (ADC1_CH6)
              |        |            (1.50 V - 2.10 V linear range)
             [R2]     [Cf] 100 nF
         100 kOhm, 1%  |  Ceramic X7R
              |        |
         +----+--------+
         |
       (GND) Common System Ground
```

---

## Mathematical Foundations

### 1. Voltage Divider & Battery Reconstruction
The continuous divider attenuation ratio is:
$$V_{\text{adc}} = V_{\text{battery}} \times \left(\frac{R_2}{R_1 + R_2}\right)$$

In firmware, the terminal cell voltage is reconstructed via:
$$V_{\text{battery}} = V_{\text{adc}} \times \left(\frac{R_1 + R_2}{R_2}\right) = 2.0 \times V_{\text{adc}}$$

* **DC Thevenin Source Impedance:** $R_{\text{th}} = \frac{R_1 \cdot R_2}{R_1 + R_2} = 50\text{ k}\Omega$
* **Max Quiescent Current:** $I_{\text{div}} = \frac{4.20\text{ V}}{200\text{ k}\Omega} = 21.0\,\mu\text{A}$ (negligible parasitic discharge)
* **Bypass Cut-off Frequency:** $f_c = \frac{1}{2\pi R_{\text{th}} C_f} \approx 31.8\text{ Hz}$ (attenuates high-frequency EMI and supplies instantaneous charge to the SAR sampling capacitor)

### 2. ADC Oversampling Noise Reduction
The firmware accumulates $N = 64$ successive readings. The standard deviation of uncorrelated zero-mean Gaussian noise is reduced by:
$$\sigma_{\text{filtered}} = \frac{\sigma_{\text{raw}}}{\sqrt{N}} = \frac{\sigma_{\text{raw}}}{\sqrt{64}} = \frac{\sigma_{\text{raw}}}{8}$$

### 3. Piecewise Linear State of Charge (SOC) Interpolation
Given adjacent lookup table entries $(V_i, \text{SOC}_i)$ and $(V_{i+1}, \text{SOC}_{i+1})$ where $V_i \le V_{\text{battery}} \le V_{i+1}$:
$$\text{SOC} = \text{SOC}_i + \left(\frac{V_{\text{battery}} - V_i}{V_{i+1} - V_i}\right) \times (\text{SOC}_{i+1} - \text{SOC}_i)$$

#### 11-Point Resting OCV-SOC Lookup Table (3.7 V Nominal Li-ion Cell at 25 °C)

| $V_{\text{battery}}$ (V) | SOC (%) | $V_{\text{battery}}$ (V) | SOC (%) |
| :---: | :---: | :---: | :---: |
| $\le 3.00$ | 0.0 (Cutoff) | 3.84 | 60.0 |
| 3.50 | 10.0 | 3.93 | 70.0 |
| 3.61 | 20.0 | 4.02 | 80.0 |
| 3.66 | 30.0 | 4.11 | 90.0 |
| 3.70 | 40.0 | $\ge 4.20$ | 100.0 (Full) |
| 3.74 | 50.0 | — | — |

---

## OLED User Interface & Alert States

The SSD1306 OLED displays four status pages updated at each 2-second cycle:

```
+------------------------+      +------------------------+
| BMS MONITOR            |      | BMS MONITOR            |
| V:3.85V   60%          |      | V:3.28V    5%          |
| T:27.4C                |      | T:29.1C                |
| STATUS: OK             |      | WARN:LOW BAT           |
+------------------------+      +------------------------+
      Normal State                   Low-Battery Alert

+------------------------+      +------------------------+
| BMS MONITOR            |      | BMS MONITOR            |
| V:3.92V   70%          |      | V:3.75V   52%          |
| T:46.8C                |      | T:ERR                  |
| WARN:HI TEMP           |      | STATUS: OK             |
+------------------------+      +------------------------+
    High-Temp Alert                1-Wire Sensor Disconnect
```

### Alert Thresholds with Hysteresis
* **Low-Battery Warning:** Asserts when $V_{\text{battery}} \le 3.30\text{ V}$ or $\text{SOC} \le 15.0\%$. Clears only when $V_{\text{battery}} \ge 3.40\text{ V}$ and $\text{SOC} \ge 18.0\%$.
* **High-Temperature Warning:** Asserts when $T \ge 45.0\,^\circ\text{C}$. Clears only when $T \le 43.0\,^\circ\text{C}$.
* **Fault Handling:** If the DS18B20 presence pulse or CRC fails, temperature is flagged as `T:ERR`, preventing false over-temperature alarms while continuing voltage monitoring.

---

## Repository Structure

```
esp32-bms/
├── CMakeLists.txt          # Root ESP-IDF CMake project definition
├── sdkconfig.defaults      # Target ESP32, 1000Hz tick rate, line-fit calibration
├── main/
│   ├── CMakeLists.txt      # Component registration
│   └── main.c              # Modular ESP-IDF v5.x C firmware implementation
├── esp32.c                 # Standalone copy of the C firmware
├── BMS project.pdf         # Complete 32-page IIT Madras Technical Design Report
├── main.tex                # Comprehensive LaTeX source document
├── reference.bib           # IEEE BibTeX bibliographic references
├── .gitignore              # Ignores build artifacts and temporary files
└── README.md               # Technical documentation
```

---

## Build and Flashing Instructions

### Prerequisites
* [ESP-IDF v5.x](https://docs.espressif.com/projects/esp-idf/en/v5.3/esp32/get-started/) installed and environment variables exported (`. $HOME/esp/esp-idf/export.sh`).
* Micro-USB cable connected to the ESP32 DevKit V1.
* A standard TeX Live distribution (`pdflatex`, `bibtex`) if rebuilding the report.

### 1. Compile and Flash the ESP-IDF Firmware
```bash
# Clone repository
git clone https://github.com/pradumon14/esp32-bms.git
cd esp32-bms

# Set compilation target
idf.py set-target esp32

# Build firmware
idf.py build

# Flash to ESP32 and open serial monitor
idf.py -p /dev/ttyUSB0 flash monitor
```

### 2. Compile the Technical Report (LaTeX)
```bash
pdflatex -interaction=nonstopmode main.tex
bibtex main
pdflatex -interaction=nonstopmode main.tex
pdflatex -interaction=nonstopmode main.tex
```
The output PDF is generated as `main.pdf` (and mirrors `BMS project.pdf`).

---

## Planned Validation Protocol

| Validation Test | Input Stimulus | Acceptance Criterion | Verification Status |
| :--- | :--- | :--- | :---: |
| **Voltage Accuracy Sweep** | Bench PSU stepped from 3.00 V to 4.20 V in 0.1 V increments | $\|V_{\text{ESP32}} - V_{\text{DMM}}\| \le 30\text{ mV}$ | Planned |
| **SOC Piecewise Linearity** | Intermediate voltages (e.g., 3.78 V) | Computes $54.0\% \pm 0.5\%$ matching mathematical model | Planned |
| **Low-Battery Hysteresis** | Voltage swept 3.60 V $\to$ 3.25 V $\to$ 3.45 V | Asserts $\le 3.30\text{ V}$, releases $\ge 3.40\text{ V}$ | Planned |
| **High-Temp Hysteresis** | Heat probe to 46 °C, cool to 42 °C | Asserts $\ge 45\,^\circ\text{C}$, releases $\le 43\,^\circ\text{C}$ | Planned |
| **1-Wire Disconnect Fault** | Disconnect DS18B20 DQ data line | Displays `T:ERR`, system continues voltage logging | Planned |

---

## Future Roadmap

1. **Precision Current Sensing:** Integrate an INA226/INA219 I2C current sensor across a low-side shunt to implement Coulomb counting and Coulomb-OCV hybrid estimation.
2. **IoT Wireless Telemetry:** Transmit real-time telemetry over Wi-Fi/MQTT or BLE to a dashboard (e.g., Home Assistant or mobile app).
3. **Active Solid-State Cutoff:** Drive external N-channel MOSFET switches (or a DW01A protection IC) for hardware over-current and under-voltage load disconnection.
4. **Deep-Sleep Power Optimization:** Gate the divider ground using a small P-channel MOSFET to achieve near-zero quiescent standby drain during deep sleep.

---

## Academic Attribution & License

* **Project:** Battery Monitoring System Using ESP32 and C Firmware
* **Author:** Pradumon Sahani
* **Institution:** Indian Institute of Technology Madras (IIT Madras)
* **License:** [MIT License](LICENSE) — Open for educational and embedded engineering reference.
