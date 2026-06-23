# BMS CAN Message Reference

## Bus Assignments

| Bus  | Hardware | Pins (TX/RX) | Baud    | Frame type      | Connected to        |
|------|----------|--------------|---------|-----------------|---------------------|
| CAN1 | CAN1     | 22 / 23      | 250 kbps | 29-bit extended | Charger (HK-MF OBC) |
| CAN3 | CAN3     | 30 / 31      | 500 kbps | 11-bit standard | Vehicle / IMD       |

---

## Messages the BMS Sends Automatically

### `0x310` — BMS Status
**Bus:** CAN3 (vehicle) + CAN1 (charger, charge mode only)  
**Rate:** every 500 ms, all states including FAULT

| Byte | Field | Type | Scale / Notes |
|------|-------|------|---------------|
| 0 | flags | uint8 | bit 0 = fault active, bit 1 = contactors closed, bit 2 = charge mode |
| 1–2 | cell_v_min | uint16 | mV (e.g. 3500 = 3.500 V) |
| 3–4 | cell_v_max | uint16 | mV |
| 5–6 | cell_v_avg | uint16 | mV |
| 7 | SOC | uint8 | % (0–100) |

---

### `0x311` — BMS Temperature / Current
**Bus:** CAN3 (vehicle) + CAN1 (charger, charge mode only)  
**Rate:** every 500 ms, all states including FAULT

| Byte | Field | Type | Scale / Notes |
|------|-------|------|---------------|
| 0 | temp_min | uint8 | °C + 40 offset (e.g. 65 = 25 °C); 0xFF = no data |
| 1 | temp_max | uint8 | °C + 40 offset; 0xFF = no data |
| 2 | temp_avg | uint8 | °C + 40 offset; 0xFF = no data |
| 3 | fault_code | uint8 | 0=none, 1=cell_UV, 2=cell_OV, 3=pack_UV, 4=pack_OV, 5=over_temp, 6=spread, 7=precharge_timeout, 8=precharge_fast, 9=IMD, 10=charger_comms, 11=BQ_init, 12=BQ_comms |
| 4–5 | pack_current | int16 | (value × 0.1 A) + 3000 bias (e.g. 3000 = 0 A, 3100 = 10 A, 2900 = −10 A) |
| 6–7 | reserved | — | 0xFF |

---

### `0x1806E5F4` — BMS to Charger Command  *(Elcon OBC protocol)*
**Bus:** CAN1 only  
**Rate:** every 1 s during active charging; also sent once on fault to stop the charger

| Byte | Field | Type | Scale / Notes |
|------|-------|------|---------------|
| 0–1 | max_voltage | uint16 | 0.1 V/bit (e.g. 4000 = 400.0 V) |
| 2–3 | max_current | uint16 | 0.1 A/bit (e.g. 100 = 10.0 A); **0 = stop** |
| 4 | control | uint8 | 0x00 = charge, 0x01 = stop |
| 5–7 | reserved | — | 0x00 |

---

## Messages the BMS Sends on Request

### `0x313` — Cell Data Dump
**Bus:** CAN3 or CAN1, depending on which bus the request (`0x312`) arrived on  
**Rate:** one frame every 5 ms until dump complete  
**Trigger:** receive `0x312` with byte 0 = 1 (voltages) or 2 (temperatures)

Cell ordering: IC1 cells 1–16 → IC2 cells 1–16 → … → IC6 cells 1–16 (96 cells total)

#### Voltage dump (type 1) — 32 frames

| Byte | Field | Type | Notes |
|------|-------|------|-------|
| 0 | frame_index | uint8 | 0–31 |
| 1–2 | cell[base+0] | uint16 | mV; 0xFFFF = invalid |
| 3–4 | cell[base+1] | uint16 | mV; 0xFFFF = invalid |
| 5–6 | cell[base+2] | uint16 | mV; 0xFFFF = invalid |
| 7 | padding | — | 0xFF |

`base = frame_index × 3`

#### Temperature dump (type 2) — 14 frames

| Byte | Field | Type | Notes |
|------|-------|------|-------|
| 0 | frame_index | uint8 | 0–13 |
| 1–7 | cells[base+0..+6] | uint8 | °C + 40 offset; 0xFF = invalid |

`base = frame_index × 7`

---

## Messages the BMS Receives

### `0x312` — Cell Data Request
**Bus:** CAN3 or CAN1 (handled on both)

| Byte | Field | Notes |
|------|-------|-------|
| 0 | type | 1 = request voltage dump, 2 = request temperature dump |

BMS replies with `0x313` frames on the same bus the request came in on.

---

### `0x314` — Set SOC
**Bus:** CAN3 or CAN1 (handled on both)

| Byte | Field | Type | Scale |
|------|-------|------|-------|
| 0–1 | SOC | uint16 | 0.1 %/bit (e.g. 725 = 72.5 %) |

---

### `0x315` — Current Sensor Calibration Point
**Bus:** CAN3 or CAN1 (handled on both)

| Byte | Field | Type | Notes |
|------|-------|------|-------|
| 0 | point | uint8 | 1 or 2 (two-point linear cal) |
| 1–2 | actual_current | int16 | 0.1 A/bit (e.g. 100 = 10.0 A, −50 = −5.0 A) |

Send both point 1 and point 2 with different known currents applied; BMS computes and saves gain + offset to EEPROM automatically.

---

### `0x18FF50E5` — Charger Broadcast  *(Elcon OBC protocol)*
**Bus:** CAN1 only (received by BMS; charger sends this)

| Byte | Field | Type | Scale / Notes |
|------|-------|------|---------------|
| 0–1 | output_voltage | uint16 | 0.1 V/bit |
| 2–3 | output_current | uint16 | 0.1 A/bit |
| 4 | status | uint8 | bit 0 = HW fault, bit 1 = temp fault, bit 2 = AC fault, bit 3 = no AC input, bit 4 = comms timeout |
| 5–7 | reserved | — | — |

BMS uses `output_current < 0.5 A` as the charge-done / current-taper termination condition.

---

### `0x23` — IMD Response
**Bus:** CAN3 only (IMD device sends this)

| Byte | Field | Type | Notes |
|------|-------|------|-------|
| 0 | register_index | uint8 | echoed from request |
| 1–2 | value | uint16 | little-endian; for reg 0x5E: voltage = (raw − 32128) × 0.05 V |

---

### `0x37` — IMD Cyclic Heartbeat
**Bus:** CAN3 only  
Contents ignored by BMS; used only to distinguish IMD traffic from vehicle traffic during mode detection at boot.
