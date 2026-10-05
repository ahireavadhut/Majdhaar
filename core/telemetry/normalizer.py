"""
Telemetry Normalizer & State Synchronizer
Aggregates asynchronous CAN frames into synchronized, unit-normalized engine state snapshots.
"""
import math
from typing import Dict, Any, Optional
from core.telemetry_schema.schemas import NormalizedEngineTelemetry, SignalProvenance

def _is_valid_signal(val: Any) -> bool:
    if val is None:
        return False
    if isinstance(val, float) and math.isnan(val):
        return False
    return True

class TelemetryNormalizer:
    def __init__(self, run_id: str = "LIVE", stream_state: str = "LIVE", data_label: str = "PHYSICAL_ENGINE_MEASUREMENT"):
        self.run_id = run_id
        self.stream_state = stream_state
        self.data_label = data_label
        self.seq_num = 0

        # Current state cache with Rotax 914 nominal defaults
        self.rpm = 1400.0
        self.map_kpa = 98.0
        self.throttle_pct = 0.0
        self.bus_voltage_v = 28.0
        self.engine_status_flags = 0

        self.cht_cyl1_c = 85.0
        self.cht_cyl2_c = 85.0
        self.cht_cyl3_c = 85.0
        self.cht_cyl4_c = 85.0

        self.egt_cyl1_c = 720.0
        self.egt_cyl2_c = 720.0
        self.egt_cyl3_c = 720.0
        self.egt_cyl4_c = 720.0

        self.oil_pressure_bar = 3.5
        self.oil_temp_c = 85.0
        self.fuel_flow_lph = 8.5
        self.fuel_pressure_bar = 3.0
        self.coolant_temp_c = 80.0

        self.rms_vibration_g = 0.85
        self.peak_knock_bar = 0.5
        self.crankcase_pressure_mbar = 2.4

        self.ambient_temp_c = 15.0
        self.ambient_pressure_kpa = 101.325
        self.altitude_m = 0.0

        self.current_lineage: Dict[str, SignalProvenance] = {}
        self.last_timestamp = 0.0

    def update_from_decoded(self, decoded_msg: Dict[str, Any], timestamp: float) -> Optional[NormalizedEngineTelemetry]:
        """
        Updates internal cache from decoded CAN message and outputs a synchronized state snapshot.
        """
        if not decoded_msg:
            return None

        self.last_timestamp = timestamp
        signals = decoded_msg.get("signals", {})
        provenance = decoded_msg.get("provenance", {})

        # Merge lineage
        self.current_lineage.update(provenance)

        # Update specific fields
        if "engine_speed_rpm" in signals and _is_valid_signal(signals["engine_speed_rpm"]):
            self.rpm = signals["engine_speed_rpm"]
        if "manifold_air_pressure_kpa" in signals and _is_valid_signal(signals["manifold_air_pressure_kpa"]):
            self.map_kpa = signals["manifold_air_pressure_kpa"]
        if "throttle_position_pct" in signals and _is_valid_signal(signals["throttle_position_pct"]):
            self.throttle_pct = signals["throttle_position_pct"]
        if "bus_voltage_v" in signals and _is_valid_signal(signals["bus_voltage_v"]):
            self.bus_voltage_v = signals["bus_voltage_v"]
        if "engine_status_flags" in signals and _is_valid_signal(signals["engine_status_flags"]):
            self.engine_status_flags = int(signals["engine_status_flags"])

        if "cht_cyl1_deg_c" in signals and _is_valid_signal(signals["cht_cyl1_deg_c"]):
            self.cht_cyl1_c = signals["cht_cyl1_deg_c"]
        if "cht_cyl2_deg_c" in signals and _is_valid_signal(signals["cht_cyl2_deg_c"]):
            self.cht_cyl2_c = signals["cht_cyl2_deg_c"]
        if "cht_cyl3_deg_c" in signals and _is_valid_signal(signals["cht_cyl3_deg_c"]):
            self.cht_cyl3_c = signals["cht_cyl3_deg_c"]
        if "cht_cyl4_deg_c" in signals and _is_valid_signal(signals["cht_cyl4_deg_c"]):
            self.cht_cyl4_c = signals["cht_cyl4_deg_c"]

        if "egt_cyl1_deg_c" in signals and _is_valid_signal(signals["egt_cyl1_deg_c"]):
            self.egt_cyl1_c = signals["egt_cyl1_deg_c"]
        if "egt_cyl2_deg_c" in signals and _is_valid_signal(signals["egt_cyl2_deg_c"]):
            self.egt_cyl2_c = signals["egt_cyl2_deg_c"]
        if "egt_cyl3_deg_c" in signals and _is_valid_signal(signals["egt_cyl3_deg_c"]):
            self.egt_cyl3_c = signals["egt_cyl3_deg_c"]
        if "egt_cyl4_deg_c" in signals and _is_valid_signal(signals["egt_cyl4_deg_c"]):
            self.egt_cyl4_c = signals["egt_cyl4_deg_c"]

        if "oil_pressure_bar" in signals and _is_valid_signal(signals["oil_pressure_bar"]):
            self.oil_pressure_bar = signals["oil_pressure_bar"]
        if "oil_temperature_deg_c" in signals and _is_valid_signal(signals["oil_temperature_deg_c"]):
            self.oil_temp_c = signals["oil_temperature_deg_c"]
        if "fuel_flow_rate_lph" in signals and _is_valid_signal(signals["fuel_flow_rate_lph"]):
            self.fuel_flow_lph = signals["fuel_flow_rate_lph"]
        if "fuel_pressure_bar" in signals and _is_valid_signal(signals["fuel_pressure_bar"]):
            self.fuel_pressure_bar = signals["fuel_pressure_bar"]

        if "rms_vibration_g" in signals and _is_valid_signal(signals["rms_vibration_g"]):
            self.rms_vibration_g = signals["rms_vibration_g"]
        if "peak_knock_intensity_bar" in signals and _is_valid_signal(signals["peak_knock_intensity_bar"]):
            self.peak_knock_bar = signals["peak_knock_intensity_bar"]
        if "crankcase_pressure_mbar" in signals and _is_valid_signal(signals["crankcase_pressure_mbar"]):
            self.crankcase_pressure_mbar = signals["crankcase_pressure_mbar"]
        if "coolant_temperature_deg_c" in signals and _is_valid_signal(signals["coolant_temperature_deg_c"]):
            self.coolant_temp_c = signals["coolant_temperature_deg_c"]

        # Calculate thermal spreads
        chts = [self.cht_cyl1_c, self.cht_cyl2_c, self.cht_cyl3_c, self.cht_cyl4_c]
        egts = [self.egt_cyl1_c, self.egt_cyl2_c, self.egt_cyl3_c, self.egt_cyl4_c]

        cht_mean = sum(chts) / 4.0
        cht_spread = max(chts) - min(chts)
        egt_mean = sum(egts) / 4.0
        egt_spread = max(egts) - min(egts)

        self.seq_num += 1

        return NormalizedEngineTelemetry(
            timestamp=self.last_timestamp,
            seq_num=self.seq_num,
            run_id=self.run_id,
            stream_state=self.stream_state,
            data_label=self.data_label,
            rpm=round(self.rpm, 1),
            map_kpa=round(self.map_kpa, 2),
            throttle_pct=round(self.throttle_pct, 1),
            bus_voltage_v=round(self.bus_voltage_v, 2),
            engine_status_flags=self.engine_status_flags,
            cht_cyl1_c=round(self.cht_cyl1_c, 1),
            cht_cyl2_c=round(self.cht_cyl2_c, 1),
            cht_cyl3_c=round(self.cht_cyl3_c, 1),
            cht_cyl4_c=round(self.cht_cyl4_c, 1),
            cht_mean_c=round(cht_mean, 1),
            cht_spread_c=round(cht_spread, 1),
            egt_cyl1_c=round(self.egt_cyl1_c, 1),
            egt_cyl2_c=round(self.egt_cyl2_c, 1),
            egt_cyl3_c=round(self.egt_cyl3_c, 1),
            egt_cyl4_c=round(self.egt_cyl4_c, 1),
            egt_mean_c=round(egt_mean, 1),
            egt_spread_c=round(egt_spread, 1),
            oil_pressure_bar=round(self.oil_pressure_bar, 3),
            oil_temp_c=round(self.oil_temp_c, 1),
            fuel_flow_lph=round(self.fuel_flow_lph, 2),
            fuel_pressure_bar=round(self.fuel_pressure_bar, 3),
            coolant_temp_c=round(self.coolant_temp_c, 1),
            rms_vibration_g=round(self.rms_vibration_g, 3),
            peak_knock_bar=round(self.peak_knock_bar, 3),
            crankcase_pressure_mbar=round(self.crankcase_pressure_mbar, 2),
            ambient_temp_c=self.ambient_temp_c,
            ambient_pressure_kpa=self.ambient_pressure_kpa,
            altitude_m=self.altitude_m,
            lineage=dict(self.current_lineage)
        )
