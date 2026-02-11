# CPP FSAE Electric 2026 Firmware

Firmware repository for the **Cal Poly Pomona FSAE Electric 2026** vehicle electronics. This repo contains embedded code for the main control modules and CAN-based data acquisition/logging nodes.

## Modules
- **VCU (Teensy 4.1)**  
  Sends command/control data to the **PM100DX inverter**, reads accelerator/pedal position sensors, and drives the **ST7796S** driver display. Publishes vehicle status on CAN.

- **BMS (Teensy 4.1 + TI BQ79616)**  
  Interfaces with the **BQ79616** for cell monitoring and controls the HV **contactors**. Publishes BMS status/faults on CAN.

- **CAN Node Board / Data Acquisition (ESP32-S3)**  
  CAN-connected DAQ node that reads **8 strain gauges** arranged as **perpendicular pairs** (multi-axis measurement) and supports additional **0–5V analog sensors**. Streams sensor data over CAN for logging/analysis.
