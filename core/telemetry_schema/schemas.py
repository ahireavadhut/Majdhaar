"""
Normalized Telemetry Schema & Data Lineage Tracking
Adheres to DO-178C deterministic data model and SIH 26054 specification.
"""
from typing import Dict, List, Optional, Any
from pydantic import BaseModel, Field
import time

class CanRawFrame(BaseModel):
    """Raw immutable CAN frame from physical bus or candump capture."""
    utc_timestamp: float = Field(default_factory=time.time)
    monotonic_ts: float = Field(default_factory=time.monotonic)
    interface: str = "can0"
    can_id: int
    dlc: int
    payload_hex: str
    seq_num: int = 0

class SignalProvenance(BaseModel):
    """Data lineage tracking for an individual engineering quantity."""
    signal_name: str
    frame_id_hex: str
    pgn: int
    spn: int
    raw_slice_hex: str
    decoded_value: Optional[float] = None
    unit: str
    timestamp: float
    source_ecu: str = "ECU-CAN0"

class NormalizedEngineTelemetry(BaseModel):
    """
    Synchronized, normalized engine state.
    Every reported quantity has strict engineering units and valid ranges.
    """
    timestamp: float
    seq_num: int
    run_id: str = "LIVE"
    stream_state: str = "LIVE" # "LIVE", "REPLAY", "UNAVAILABLE"
    data_label: str = "PHYSICAL_ENGINE_MEASUREMENT" # or "CONTROLLED_FAULT_INJECTION"

    # Engine Dynamics
    rpm: float = Field(..., description="Crankshaft rotational speed (RPM)")
    map_kpa: float = Field(..., description="Manifold Absolute Pressure (kPa)")
    throttle_pct: float = Field(..., description="Throttle valve opening percentage (0-100%)")
    bus_voltage_v: float = Field(28.0, description="Avionics DC bus voltage (V)")
    engine_status_flags: int = 0

    # Cylinder Thermal Matrix (Rotax 4-cylinder Boxer)
    cht_cyl1_c: float
    cht_cyl2_c: float
    cht_cyl3_c: float
    cht_cyl4_c: float
    cht_mean_c: float = 0.0
    cht_spread_c: float = 0.0

    # Exhaust Gas Temperatures
    egt_cyl1_c: float
    egt_cyl2_c: float
    egt_cyl3_c: float
    egt_cyl4_c: float
    egt_mean_c: float = 0.0
    egt_spread_c: float = 0.0

    # Fluids & Lubrication
    oil_pressure_bar: float
    oil_temp_c: float
    fuel_flow_lph: float
    fuel_pressure_bar: float
    coolant_temp_c: float

    # Vibration & Blowby Sensing
    rms_vibration_g: float
    peak_knock_bar: float
    crankcase_pressure_mbar: float

    # Flight / Environmental Context
    ambient_temp_c: float = 15.0
    ambient_pressure_kpa: float = 101.325
    altitude_m: float = 0.0

    # Data lineage audit trail
    lineage: Dict[str, SignalProvenance] = Field(default_factory=dict)

class TwinPhysicsPrediction(BaseModel):
    """
    Independent Reduced-Order Physics Model prediction (0D/1D MVEM + CHT network).
    Plant and twin do NOT share identical code to prevent circular validation.
    """
    timestamp: float
    predicted_rpm: float
    predicted_map_kpa: float
    predicted_cht_cyl: List[float]
    predicted_egt_cyl: List[float]
    predicted_oil_pressure_bar: float
    predicted_oil_temp_c: float
    predicted_coolant_temp_c: float

    # Residuals: r(t) = Y_sensor(t) - Y_phys(t)
    residual_rpm: float
    residual_map_kpa: float
    residual_cht_cyl: List[float]
    residual_egt_cyl: List[float]
    residual_oil_pressure_bar: float
    residual_oil_temp_c: float

class VirtualSensorEstimate(BaseModel):
    """
    Virtual analytical sensor outputs with governing physics and explicit confidence intervals.
    """
    # 1. Indicated Mean Effective Pressure (IMEP) per cylinder (bar)
    imep_cyl_bar: List[float]
    imep_mean_bar: float
    imep_uncertainty_bar: float = 0.35

    # 2. Crankcase Blowby Flow Rate (L/min)
    blowby_flow_lpm: float
    blowby_uncertainty_lpm: float = 1.2

    # 3. Hydrodynamic Oil Film Thickness h_min (microns)
    oil_film_thickness_um: float
    oil_film_uncertainty_um: float = 0.45

    # Labeling as per Rule 6:
    classification: str = "MODEL_DERIVED_ESTIMATE"
    governing_equations: Dict[str, str] = Field(default_factory=lambda: {
        "IMEP": "IMEP_i = (P_fuel_i * eta_th * 120) / (V_d * RPM)",
        "Blowby": "m_dot_blowby = k_ring * sqrt(max(0, Delta_P_case))",
        "Oil_Film": "h_min = 2.8 * (mu_oil * N_rpm^0.5 * P_oil^0.5)"
    })

class ObserverInference(BaseModel):
    """Hybrid PINN Residual Observer outputs."""
    timestamp: float
    corrected_latent_state: List[float]
    anomaly_score: float
    pinn_physics_loss: float
    thermal_residual_norm: float
    lube_residual_norm: float
    combustion_residual_norm: float

class AnomalyDiagnostic(BaseModel):
    """Fault isolation, XAI root-cause attribution, and ATA-100 work orders."""
    fault_detected: bool
    fault_type: str = "NOMINAL"
    fault_label: str = "SYSTEM_HEALTHY"
    severity: str = "NORMAL" # NORMAL, ADVISORY, CAUTION, CRITICAL
    affected_cylinder: Optional[int] = None
    ata100_chapter: str = "ATA-72 (Engine General)"
    precursor_signature: str = "Nominal operating envelope"
    work_order_text: str = "Routine inspection at scheduled interval."
    detection_latency_ms: float = 0.0
    feature_attributions: Dict[str, float] = Field(default_factory=dict)
    test_fault_injected: bool = False
    fault_injection_mode: str = "NONE"

class PrognosticsRUL(BaseModel):
    """
    Wiener Process Remaining Useful Life (RUL) with inverse-Gaussian FHT PDF.
    Rule 5 & 10: Explicit model assumptions and limitation disclosure.
    """
    timestamp: float
    health_index: float # 1.0 = brand new, 0.0 = TBO reached
    load_severity_factor: float
    drift_mu: float
    diffusion_sigma: float
    failure_threshold_d: float = 1.0
    current_degradation_x: float

    # Quantiles in equivalent flight hours
    rul_hours_p10: float
    rul_hours_p50: float
    rul_hours_p90: float
    estimated_tbo_hours: float = 1500.0

    # Limitation & lineage disclosure statement
    methodology: str = "Stochastic Wiener Process with state-dependent severity drift & inverse-Gaussian FHT"
    validation_status: str = "MODEL_DERIVED_PROGNOSTICS (No destructive physical wear datasets fabricated)"

class FullGcsTelemetryPacket(BaseModel):
    """Unified telemetry packet pushed to GCS operator interface and MAVLink bridge."""
    timestamp: float
    packet_id: int
    telemetry: NormalizedEngineTelemetry
    twin: TwinPhysicsPrediction
    virtual_sensors: VirtualSensorEstimate
    observer: ObserverInference
    diagnostic: AnomalyDiagnostic
    prognostics: PrognosticsRUL
