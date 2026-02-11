# BMS Firmware (Teensy 4.1 + BQ79616)

Firmware for the Cal Poly Pomona FSAE Electric 2026 **BMS**, running on a **Teensy 4.1** and interfacing with the **TI BQ79616** cell monitor.

## What it does
- Reads cell/pack data from the **BQ79616**
- Evaluates basic safety limits and fault conditions
- Controls the HV **contactors** (enable/disable, shutdown on fault)
- Reports BMS status/faults over **CAN** to the VCU / loggers

