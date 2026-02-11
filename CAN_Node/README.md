# CAN Node Board Firmware (ESP32-S3)

Firmware for the Cal Poly Pomona FSAE Electric 2026 **CAN Node Board** data acquisition system, running on an **ESP32-S3**.

## What it does
- Reads **8 strain gauges** arranged as **perpendicular pairs** (for multi-axis strain/force measurement)
- Reads additional **0–5V analog sensors** (any supported channels)
- Converts raw ADC readings to engineering units 
- Sends sensor data over **CAN** for logging and vehicle integration

