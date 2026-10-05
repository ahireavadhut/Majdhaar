# Compliance and Traceability

**Team:** Majdhaar | **Project:** Aero Piston Engine Digital Twin (SIH26054, DRDO)
**Platform:** DRDO TAPAS-BH-201 MALE UAV, Rotax 914 F / 915 iS

## Contents

1. [Compliance Position](#1-compliance-position)
2. [Standards Coverage](#2-standards-coverage)
3. [Requirements Traceability Matrix](#3-requirements-traceability-matrix)
4. [DO-178C Level C Design Evidence](#4-do-178c-level-c-design-evidence)
5. [ATA-100 Maintenance Mapping](#5-ata-100-maintenance-mapping)
6. [MIL-STD-810H Environmental Profile](#6-mil-std-810h-environmental-profile)
7. [Integrity Rules](#7-integrity-rules)

---

## 1. Compliance Position

This codebase is an engineering prototype. It follows a **design-for-compliance** approach and produces certification-style evidence: traceability, coding standards, deterministic timing and test coverage.

**It makes no claim of FAA, EASA or CEMILAC airworthiness certification.**

---

## 2. Standards Coverage

| Standard | Role in this project | Where it is addressed |
|----------|---------------------|-----------------------|
| DO-178C Level C | Software verification and design evidence | Sections 3 and 4 |
| ATA-100 | Maintenance chapter and work order mapping for every diagnosed fault | Section 5 |
| MIL-STD-810H | High-altitude and temperature envelope modelled in the twin | Section 6 |
| SAE J1939 / DroneCAN | 29-bit CAN telemetry decoding | SIH-REQ-001, 002 |
| ARINC 825 | Avionics bus alignment of telemetry architecture | Roadmap standard |
| MAVLink v2 | `EFI_STATUS` (Msg 225) downlink | SIH-REQ-011 |

---

## 3. Requirements Traceability Matrix

Bidirectional trace from problem statement to architecture, source, and test.

| ID | Requirement | Component | Source | Verification | Status |
|----|-------------|-----------|--------|--------------|--------|
| SIH-REQ-001 | High-rate 29-bit CAN acquisition (J1939 / DroneCAN) | Telemetry ingestion | `core/telemetry/decoder.py` | `tests/test_telemetry_decoder.py::test_engine_dynamics_encoding_decoding` | PASS |
| SIH-REQ-002 | Data lineage, signal name to raw bytes | Schema layer | `core/telemetry_schema/schemas.py` | `tests/test_telemetry_decoder.py::test_thermal_matrix_provenance` | PASS |
| SIH-REQ-003 | Immutable raw recorder and run metadata YAML | Storage layer | `core/telemetry/recorder.py` | `scripts/verify_all.py::[LEVEL 1]` | PASS |
| SIH-REQ-004 | Independent reduced-order twin (MVEM + 4-node CHT) | Twin physics kernel | `core/twin_core/physics_model.py` | `tests/test_twin_physics.py::test_twin_step_and_residuals` | PASS |
| SIH-REQ-005 | Healthy-baseline calibration before fault detection | Twin calibrator | `core/twin_core/calibrator.py` | `scripts/calibrate_baseline.py` | PASS |
| SIH-REQ-006 | Residual observer with mass, energy, monotonicity loss | PINN observer | `pinn/model.py`, `core/observer/` | `tests/test_pinn_observer.py::test_pinn_observer_inference` | PASS |
| SIH-REQ-007 | Virtual sensors (IMEP, blowby, oil film h_min) with confidence intervals | Analytical sensors | `core/virtual_sensors/estimators.py` | `tests/test_virtual_sensors.py::test_virtual_sensor_bounds_and_uncertainty` | PASS |
| SIH-REQ-008 | Controlled software fault injection with audit tagging | Test rig | `core/telemetry/fault_injector.py` | `scripts/verify_all.py::[LEVEL 3]` | PASS |
| SIH-REQ-009 | Multi-cylinder fault isolation and ATA-100 work orders | Diagnostics engine | `core/diagnostics/fault_classifier.py` | `tests/test_diagnostics_ata100.py::test_diagnostics_all_ata_chapters` | PASS |
| SIH-REQ-010 | Wiener process RUL with inverse-Gaussian FHT | Prognostics engine | `core/prognostics/wiener_rul.py` | `tests/test_wiener_rul.py::test_wiener_rul_quantiles` | PASS |
| SIH-REQ-011 | MAVLink v2 `EFI_STATUS` (Msg 225) downlink bridge | Avionics bridge | `core/mavlink_bridge/efi_status.py` | `tests/test_mavlink_efi.py::test_mavlink_encoding_decoding` | PASS |
| SIH-REQ-012 | Defense GCS with live thermal boxer rendering and XAI | Operator UI | `gcs_frontend/`, `gcs_api/` | `tests/test_end_to_end_pipeline.py` | PASS |

**Conclusion:** all 12 requirements trace from the problem statement to implementation and to automated verification.

---

## 4. DO-178C Level C Design Evidence

### Determinism and robustness
- Deterministic step cycle under 20 ms, supporting 50 Hz loops.
- Pydantic v2 typed schemas with strict boundary clipping on all physical state vectors.
- No unhandled runtime exceptions in the live telemetry path. Fallback modes cover external dependency loss and interrupted CAN frames.
- Telemetry decoding and the physics kernel share no models or equations (no circular validation).

### Verification
- Pytest unit and integration suite in `tests/`: 100% pass across 9 test suites.
- Static discipline: PEP-8, typed function signatures, explicit docstrings.
- Repeatability: every reported metric (MAE, RMSE, detection latency, RUL quantiles) is reproducible from named run files in `data/raw/` and `data/decoded/`.

### Reproduce the evidence

```powershell
python -m pytest tests -v
python scripts/verify_all.py
python scripts/calibrate_baseline.py
```

---

## 5. ATA-100 Maintenance Mapping

Every diagnostic isolation flag maps to an ATA-100 chapter and a work order, supporting depot and line maintenance for the TAPAS-BH-201.

| Chapter | Fault flag | Precursor signature | Root cause | Work order |
|---------|-----------|---------------------|-----------|-----------|
| 72-30 Cylinder and piston rings | `RING_PACK_SCUFFING` | Crankcase pressure above 8.0 mbar, virtual blowby above 14.0 L/min, raised RMS vibration | Ring micro-welding and bore scuffing, blowby into crankcase | `ATA-72-30-01` |
| 72-40 Valvetrain and heads | `EXHAUST_VALVE_RECESSION` | Single-cylinder EGT +60 to +85 C, CHT +20 to +35 C, local vibration surge | Valve seat erosion and micro-welding, combustion blowby | `ATA-72-40-02` |
| 73-10 Fuel injection and metering | `INJECTOR_COKING` | Single-cylinder EGT +35 to +75 C, normal CHT spread | Carbon buildup at injector orifice restricting fuel flow | `ATA-73-10-02` |
| 77-20 Temperature indicating | `SENSOR_DRIFT` | Stationary or creeping offset on one CHT thermocouple, no correlation with EGT or load | Probe tip oxidation or harness contact resistance | `ATA-77-20-03` |
| 79-10 Oil storage and distribution | `OIL_THERMAL_SHEAR` | Oil pressure below 2.05 bar at cruise RPM, sump above 110 C, h_min below 1.5 um | Viscosity breakdown from thermal shear, pump cavitation | `ATA-79-10-04` |
| 81-10 Turbocharger | `TURBO_BEARING_COKING` | MAP boost deficit above 15 kPa at high throttle | Journal bearing coking and radial drag on turbine | `ATA-81-10-01` |

### Work order procedures

**ATA-72-30-01** (ring pack)
1. Remove spark plugs and run a leak-down test.
2. Borescope the cylinder barrels for vertical gouging or micro-seizure marks.
3. Inspect the crankcase breather oil separator and one-way check valve.

**ATA-72-40-02** (exhaust valve)
1. Check hydraulic lash compensators and mechanical lash clearances.
2. Measure valve stem installed height.
3. Borescope the exhaust valve margin and seat face.

**ATA-73-10-02** (injector)
1. Remove the suspect injector and mount it on a calibrated flow bench.
2. Measure spray cone pattern and delivery volume.
3. Ultrasonically clean the nozzle or replace with a certified unit.

**ATA-77-20-03** (sensor drift)
1. Measure probe cold-junction resistance and loop impedance.
2. Run a 2-point calibration against a reference dry-well.
3. Inspect harness shielding and ground loop bond.

**ATA-79-10-04** (oil)
1. Inspect the magnetic chip detector plug for metal particles.
2. Send oil for spectrographic analysis (SOAP) for tin and lead bearing material.
3. Drain oil, replace the filter, inspect the pressure relief valve spring.

**ATA-81-10-01** (turbo)
1. Check compressor and turbine wheels for radial and axial end play.
2. Inspect the oil feed line and return scavenge pump.
3. Calibrate the wastegate pneumatic actuator and pressure control servo.

---

## 6. MIL-STD-810H Environmental Profile

### Operating envelope (TAPAS-BH-201)

| Parameter | Value |
|-----------|-------|
| Service ceiling | Up to 9,000 m (about 30,000 ft) |
| Ambient temperature | -55 C (high-altitude cruise) to +55 C (hot-day desert ground ops) |
| Pressure model | ISA barometric formula |

$$P_{\text{amb}}(h) = P_0 \cdot \left(1 - 2.25577 \times 10^{-5} \cdot h\right)^{5.25588}, \quad P_0 = 101.325\ \text{kPa}$$

### Stress factors modelled in the twin
1. **Intake density lapse.** At 9,000 m, air density is about 30% of sea level. The turbo wastegate closes to hold rated MAP.
2. **Cooling mass flow reduction.** Thin air lowers convective heat transfer across the air-cooled barrels:

$$R_{\text{ambient}}(h) = R_{\text{ambient,0}} \cdot \left(\frac{\rho_0}{\rho(h)}\right)^{0.6}$$

3. **Turbo bearing thermal soak.** Max continuous power in climb raises turbine inlet temperature and oil coking risk if scavenge lines are restricted. This links to `TURBO_BEARING_COKING` (ATA-81-10).

---

## 7. Integrity Rules

Rules from the implementation plan that this evidence relies on.

| Rule | Statement | Evidence |
|------|-----------|----------|
| 3 | No circular validation: decoder and physics kernel are independent | SIH-REQ-004, Section 4 |
| 6 | Virtual sensors are labeled MODEL-DERIVED ESTIMATE with confidence intervals | SIH-REQ-007 |
| 7 | Design-for-compliance only, no certification claim | Section 1 |
| 8 | Injected faults are labeled `CONTROLLED_FAULT_INJECTION`, never presented as natural wear | SIH-REQ-008 |
| 10 | Every metric is reproducible from named canonical run files | Section 4 |
