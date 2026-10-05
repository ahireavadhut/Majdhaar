"""
Controlled Software-Level Fault Injection Rig
Complies with Section 9.2: Safe software-level fault injection.
Applies explicit test transformations to telemetry streams while guaranteeing
strict labeling (data_label = "CONTROLLED_FAULT_INJECTION").
Never represented as naturally occurring physical wear.
"""
from typing import Optional
from core.telemetry_schema.schemas import NormalizedEngineTelemetry

class ControlledFaultInjector:
    def __init__(self):
        self.active_fault: str = "NONE" # "NONE", "INJECTOR_COKING_CYL3", "EXHAUST_VALVE_RECESSION_CYL2", "RING_SCUFFING_BLOWBY", "TURBO_BEARING_COKING", "OIL_SHEAR_CAVITATION", "SENSOR_DRIFT_CHT1"
        self.severity_pct: float = 0.0 # 0.0 to 100.0%
        self.fault_start_time: Optional[float] = None
        self.drift_accumulator: float = 0.0

    def set_fault(self, fault_type: str, severity_pct: float = 100.0):
        self.active_fault = fault_type
        self.severity_pct = max(0.0, min(100.0, severity_pct))
        self.fault_start_time = None
        self.drift_accumulator = 0.0

    def clear_fault(self):
        self.active_fault = "NONE"
        self.severity_pct = 0.0
        self.fault_start_time = None
        self.drift_accumulator = 0.0

    def apply(self, telemetry: NormalizedEngineTelemetry) -> NormalizedEngineTelemetry:
        """Applies explicit transformation and updates audit tags."""
        if self.active_fault == "NONE" or self.severity_pct <= 0.0:
            return telemetry

        if self.fault_start_time is None:
            self.fault_start_time = telemetry.timestamp

        sev = self.severity_pct / 100.0

        # Clone telemetry
        modified = telemetry.model_copy(deep=True)
        modified.data_label = "CONTROLLED_FAULT_INJECTION"

        if self.active_fault == "INJECTOR_COKING_CYL3":
            # Single-cylinder EGT elevation (+35°C to +80°C), slight lean flutter
            egt_offset = 65.0 * sev
            modified.egt_cyl3_c += egt_offset
            modified.egt_spread_c = max(
                modified.egt_cyl1_c, modified.egt_cyl2_c, modified.egt_cyl3_c, modified.egt_cyl4_c
            ) - min(
                modified.egt_cyl1_c, modified.egt_cyl2_c, modified.egt_cyl3_c, modified.egt_cyl4_c
            )

        elif self.active_fault == "EXHAUST_VALVE_RECESSION_CYL2":
            # Exhaust valve recession: EGT spike, CHT rise, localized vibration RMS surge
            modified.egt_cyl2_c += 85.0 * sev
            modified.cht_cyl2_c += 28.0 * sev
            modified.rms_vibration_g += 1.4 * sev

        elif self.active_fault == "RING_SCUFFING_BLOWBY":
            # Ring scuffing: crankcase overpressurization (> 12 mbar), blowby flow surge
            modified.crankcase_pressure_mbar += 14.5 * sev
            modified.rms_vibration_g += 0.9 * sev

        elif self.active_fault == "TURBO_BEARING_COKING":
            # Turbo bearing drag & coking: MAP boost deficit under load, compressor exit temp rise
            map_deficit = 25.0 * sev
            modified.map_kpa = max(35.0, modified.map_kpa - map_deficit)
            # Slight elevation of EGT across all cylinders due to richer mixture
            modified.egt_cyl1_c += 25.0 * sev
            modified.egt_cyl2_c += 25.0 * sev
            modified.egt_cyl3_c += 25.0 * sev
            modified.egt_cyl4_c += 25.0 * sev

        elif self.active_fault == "OIL_SHEAR_CAVITATION":
            # Viscosity breakdown: Decaying oil pressure, elevated oil temp (> 110°C)
            press_decay = 1.8 * sev
            modified.oil_pressure_bar = max(0.8, modified.oil_pressure_bar - press_decay)
            modified.oil_temp_c += 26.0 * sev

        elif self.active_fault == "SENSOR_DRIFT_CHT1":
            # Thermocouple drift: probe junction oxidation, stationary offset on CHT1 without load correlation
            self.drift_accumulator += 0.05 * sev
            drift_val = min(35.0 * sev, self.drift_accumulator)
            modified.cht_cyl1_c += drift_val
            modified.cht_spread_c = max(
                modified.cht_cyl1_c, modified.cht_cyl2_c, modified.cht_cyl3_c, modified.cht_cyl4_c
            ) - min(
                modified.cht_cyl1_c, modified.cht_cyl2_c, modified.cht_cyl3_c, modified.cht_cyl4_c
            )

        return modified
