# Aero Piston Engine Digital Twin

**Team:** Majdhaar
**Problem Statement:** SIH26054, AI-Enabled Real-Time Digital Twin System for Health Monitoring, Fault Prediction and Mission Management
**Sponsor:** DRDO | **Category:** Software | **Theme:** Robotics and Drones

---

## Table of Contents

1. [Overview](#1-overview)
2. [Quick Start](#2-quick-start)
3. [Key Features](#3-key-features)
4. [Problem Statement Coverage](#4-problem-statement-coverage)
5. [System Architecture](#5-system-architecture)
6. [Data Lineage and Provenance](#6-data-lineage-and-provenance)
7. [Baseline Calibration](#7-baseline-calibration)
8. [Hardware Bring-Up (Live CAN)](#8-hardware-bring-up-live-can)
9. [Verification and Validation](#9-verification-and-validation)
10. [Demo Flow](#10-demo-flow)
11. [Anticipated Questions](#11-anticipated-questions)

---

## 1. Overview

Conventional UAV engine monitors are threshold-based. They warn only after a redline is crossed, when damage is often irreversible. This project is a grey-box digital twin of the aero piston engine that mirrors the real engine from live CAN telemetry, detects fault precursors early, and estimates Remaining Useful Life (RUL) with confidence bounds.

Design principle: **Reality-First Architecture**. Telemetry is decoded from standard 29-bit CAN (J1939 / DroneCAN) and ARINC 825 buses. The twin is an independently formulated reduced-order physics model, so there is no circular same-code validation.

---

## 2. Quick Start

Start the Ground Control Station (GCS) dashboard:

```powershell
python scripts/run_pipeline.py --mode server --port 8000
```

Open `http://localhost:8000`.

Run the full verification suite:

```powershell
python scripts/verify_all.py
```

Run unit and integration tests:

```powershell
python -m pytest tests -v
```

Full pipeline order:

```powershell
python -m pytest tests -v
python scripts/verify_all.py
python scripts/calibrate_baseline.py
python scripts/train_and_export_pinn.py
python scripts/run_pipeline.py --mode server --port 8000
```

---

## 3. Key Features

| # | Feature | Detail |
|---|---------|--------|
| 1 | Real telemetry foundation | 29-bit CAN decoding (J1939 / DroneCAN), immutable raw candump archives, full signal lineage |
| 2 | Independent physics twin | 0D/1D Mean Value Engine Model (MVEM) intake dynamics, 4-node lumped CHT network, Woschni heat transfer |
| 3 | Hybrid PINN residual observer | PyTorch / TorchScript JIT model with mass conservation, energy balance and wear monotonicity loss terms |
| 4 | Analytical virtual sensors | IMEP, crankcase blowby flow, hydrodynamic oil film thickness (h_min), each with uncertainty intervals |
| 5 | Multi-cylinder precursor isolation | Injector coking, exhaust valve recession, ring scuffing, turbo drag, oil shear, mapped to ATA-100 chapters |
| 6 | Stochastic Wiener RUL | Inverse-Gaussian First Hitting Time density with P10 / P50 / P90 bounds |
| 7 | Avionics downlink | Bidirectional MAVLink v2 `EFI_STATUS` (Message ID 225) |
| 8 | Defense GCS console | Dark theme (`#000000`), animated 4-cylinder boxer cross-section, thermal heatmaps, fault injection rig |

---

## 4. Problem Statement Coverage

| PS Requirement | Our Implementation |
|----------------|--------------------|
| A. Digital Twin Core | Physics twin synced to live CAN data, modular pipeline, real-time ingestion |
| B. Health Monitoring | RPM, CHT, EGT, oil pressure and temperature, vibration and blowby, multi-sensor health index |
| C. Fault Detection and Prediction | 6 failure modes with precursor isolation and ATA-100 work orders |
| D. AI/ML Layer | PINN residual observer, anomaly detection, Wiener RUL, trend analysis, maintenance advisory |
| E. Simulation and Replay | candump log replay, ISA atmosphere up to 9,000 m (30,000 ft), controlled fault injection |
| F. Dashboard | Real-time health status, fault alerts, thermal and efficiency views, work orders |

Innovation areas addressed: physics-informed AI, edge AI (TorchScript JIT), hybrid thermodynamic plus data-driven modelling, explainable AI (feature attribution), autonomous maintenance advisory.

---

## 5. System Architecture

```text
              PHYSICAL ENGINE / CAN BUS
                         |
                         v
                 Raw CAN Capture
               (candump -L format)
                         |
                         v
                 PGN / SPN Decoder
                (29-bit J1939 frame)
                         |
                         v
                Telemetry Normalizer
                         |
          +--------------+--------------+
          v                             v
   Raw Data Archive            Independent Digital Twin
 (Immutable Evidence)          (0D/1D MVEM + CHT Net)
                                        |
                                        v
                              Residual Calculation
                            r(t) = Y_sensor - Y_phys
                                        |
                                        v
                              Hybrid PINN Observer
                              (TorchScript JIT Edge)
                                        |
        +-------------------------------+------------------------------+
        v                               v                              v
 Virtual Sensors               Fault Diagnostics                Health Index
 - IMEP Cyl 1..4               - Precursor isolation            - Multi-sensor distress
 - Crankcase blowby            - ATA-100 work orders            - Monotonic wear filter
 - Hydrodynamic film           - XAI feature attribution                |
                                                                        v
                                                                Wiener Process RUL
                                                                - State-dependent drift
                                                                - Inv-Gaussian FHT PDF
                                                                - P10 / P50 / P90
                                                                        |
                                                                        v
                                                              GCS Console and MAVLink
                                                              - WebSocket (50 Hz)
                                                              - EFI_STATUS (Msg 225)
```

**Independence guarantee:** the telemetry decoder and the physics kernel share no plant model implementation. The twin is a reduced-order model calibrated against frozen healthy baseline runs.

---

## 6. Data Lineage and Provenance

Every normalized signal traces back to raw CAN frames.

```text
Signal: engine_speed_rpm
 +- CAN Arbitration ID: 0x18FEE000
    +- PGN: 0xFEE0 (Engine Dynamics)
       +- SPN: 190 (Engine Speed)
          +- Byte slice [0:2], little-endian unsigned 16-bit
             +- Raw hex 0x28A0 (integer 10400)
                +- Scale 0.125, offset 0.0
                   +- Decoded value: 1300.0 RPM
                      +- Source: ECU-ROTAX-01
                         +- Timestamp: 1725350400.000000 UTC
```

**Archive layout**

| Path | Contents |
|------|----------|
| `data/raw/` | Original unaltered `candump -L` logs |
| `data/decoded/` | Unit-normalized, time-aligned JSON state vectors |
| `data/metadata/` | YAML: test date, operator, test cell, atmospheric conditions, compliance disclosures |

---

## 7. Baseline Calibration

Calibrate the physics model on healthy runs before enabling fault detection or prognostics.

1. Load `run02_healthy_cruise_climb` (takeoff plus continuous cruise).
2. Fit volumetric efficiency (eta_v) and thermal resistance (R_coolant) by least-squares.
3. Freeze parameters in `data/reports/calibration_report_run02_healthy_cruise_climb.json`.
4. Validate on held-out `run03_held_out_validation_loiter` (no data leakage).

---

## 8. Hardware Bring-Up (Live CAN)

Moves the system from candump replay to a live Rotax 914 F / 915 iS on a test bench or in the TAPAS-BH-201 avionics bay.

**Supported adapters (any Linux SocketCAN device)**
- USB-CAN: CANable (Candlelight), PEAK PCAN-USB, Kvaser Leaf Light v2, Waveshare 2-CH CAN FD
- Embedded: Microchip MCP2515 (SPI-CAN), native CAN on Raspberry Pi, NVIDIA Jetson Orin, Xilinx Zynq

**Configure the interface** (Rotax 914/915 bitrate: 500 kbps)

```bash
ip link show
sudo ip link set can0 type can bitrate 500000 restart-ms 100
sudo ip link set can0 up
ip -details link show can0
```

Expected: `state ERROR-ACTIVE`, `bitrate 500000`.

**Verify bus integrity before launch**

```bash
candump can0
candump -L can0 > data/raw/live_engine_run_can.log
```

| Arbitration ID | Content | Rate |
|----------------|---------|------|
| `18FEE000` | Engine dynamics | 50 Hz |
| `18FEE100` | Thermal matrix 1 (CHT) | 10 Hz |
| `18FEE200` | Thermal matrix 2 (EGT) | 10 Hz |
| `18FEE300` | Fluids and lube | 10 Hz |
| `18FEE400` | Vibration and blowby | 50 Hz |

**Launch in live mode.** In `configs/telemetry.yaml`:

```yaml
telemetry:
  interface_type: "socketcan"
  channel: "can0"
  bitrate: 500000
```

```bash
python scripts/run_pipeline.py --mode server --port 8000
```

The dashboard badge switches from `REPLAY STREAM` to `LIVE PHYSICAL CAN0`.

---

## 9. Verification and Validation

| Level | Scope | Check |
|-------|-------|-------|
| 1 | Signal integrity and CAN decoding | IDs, PGNs, SPNs, scale and offset parse to physical ranges, no frame drop or bit corruption |
| 2 | Twin baseline | Held-out healthy run `run03_held_out_validation_loiter`; MAE, RMSE, bias |
| 3 | Anomaly detection and isolation | 6 failure modes, detection latency under 35 ms, automatic ATA-100 work orders |
| 4 | High-altitude robustness | ISA pressure lapse up to 9,000 m (30,000 ft) |
| 5 | Virtual sensor observability | IMEP, blowby, oil film thickness with confidence intervals and model-derived disclosure |

**Failure modes covered:** `INJECTOR_COKING`, `EXHAUST_VALVE_RECESSION`, `RING_PACK_SCUFFING`, `TURBO_BEARING_COKING`, `OIL_THERMAL_SHEAR`, `SENSOR_DRIFT`.

**Compliance evidence:** 12 system requirements trace to automated Pytest suites (100% pass). All reported metrics are reproducible from raw logs in `data/raw/`.

---

## 10. Demo Flow

| Time | Segment | What to show |
|------|---------|--------------|
| 0:00 to 1:00 | Problem and architecture | Header bar: `DRDO TAPAS-BH-201`, `Rotax 914 F / 915 iS`, `50 Hz SYNC` |
| 1:00 to 2:30 | Live physics twin | Boxer cutaway (firing order 1-3-4-2), thermal heatmaps, spread matrix (healthy CHT spread under 3 C, EGT under 12 C), virtual sensors labeled MODEL-DERIVED ESTIMATE |
| 2:30 to 4:00 | Fault injection | Click **Injector Clog (Cyl 3)**: banner shows controlled test fault, Cyl 3 EGT rises +65 C, card isolates `INJECTOR_COKING (Cylinder #3)`, work order `ATA-73-10-02`. Then **Oil Shear / Pressure Drop**: oil pressure falls below 2.0 bar, `OIL_THERMAL_SHEAR`, work order `ATA-79-10-04`. Then **Reset to Healthy Baseline** |
| 4:00 to 5:00 | RUL and compliance | P10 / P50 / P90 RUL (P10 for abort vs proceed), MAVLink `EFI_STATUS` panel, DO-178C traceability |

All injected faults are executed at a controlled software boundary and labeled `CONTROLLED_FAULT_INJECTION`. They are not natural physical wear.

---

## 11. Anticipated Questions

**Was this run on a physical Rotax engine?**
The pipeline ingests live Linux SocketCAN traffic from a physical Rotax ECU. The repository includes 57,000 raw frames in `candump -L` format from healthy baseline profiles. Fault cases are controlled software injections, labeled as such.

**Why a PINN instead of an end-to-end LSTM or Transformer?**
Pure data-driven models can produce non-physical states and need large run-to-failure datasets that do not exist for military aero engines. We use first-principles physics for nominal prediction and constrain the residual network with mass conservation, energy balance and wear monotonicity terms.

---

## Team

**Majdhaar**
