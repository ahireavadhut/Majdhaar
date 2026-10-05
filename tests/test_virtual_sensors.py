import pytest
from core.virtual_sensors.estimators import VirtualSensors
from core.telemetry_schema.schemas import NormalizedEngineTelemetry

def test_virtual_sensor_bounds_and_uncertainty():
    vs = VirtualSensors()
    telemetry = NormalizedEngineTelemetry(
        timestamp=100.0,
        seq_num=1,
        rpm=5200.0,
        map_kpa=108.0,
        throttle_pct=80.0,
        bus_voltage_v=28.0,
        cht_cyl1_c=106.0,
        cht_cyl2_c=105.0,
        cht_cyl3_c=107.0,
        cht_cyl4_c=105.0,
        egt_cyl1_c=815.0,
        egt_cyl2_c=810.0,
        egt_cyl3_c=820.0,
        egt_cyl4_c=812.0,
        oil_pressure_bar=3.9,
        oil_temp_c=92.0,
        fuel_flow_lph=24.0,
        fuel_pressure_bar=3.2,
        coolant_temp_c=82.0,
        rms_vibration_g=0.9,
        peak_knock_bar=0.4,
        crankcase_pressure_mbar=2.6
    )
    est = vs.estimate(telemetry)
    assert len(est.imep_cyl_bar) == 4
    assert 1.0 <= est.imep_mean_bar <= 18.0
    assert est.blowby_flow_lpm > 0.0
    assert est.oil_film_thickness_um > 0.0
    assert est.classification == "MODEL_DERIVED_ESTIMATE"
    assert "IMEP" in est.governing_equations

def test_virtual_sensors_zero_and_negative_edge_cases():
    vs = VirtualSensors()
    telemetry = NormalizedEngineTelemetry(
        timestamp=100.0,
        seq_num=1,
        rpm=-500.0, # Negative RPM
        map_kpa=-10.0, # Negative MAP
        throttle_pct=0.0,
        bus_voltage_v=28.0,
        cht_cyl1_c=-50.0,
        cht_cyl2_c=-50.0,
        cht_cyl3_c=-50.0,
        cht_cyl4_c=-50.0,
        egt_cyl1_c=-50.0, # Negative EGTs
        egt_cyl2_c=-50.0,
        egt_cyl3_c=-50.0,
        egt_cyl4_c=-50.0,
        oil_pressure_bar=0.0, # Loss of oil pressure
        oil_temp_c=-280.0, # Extreme low temperature
        fuel_flow_lph=0.0, # Zero fuel flow
        fuel_pressure_bar=0.0,
        coolant_temp_c=-40.0,
        rms_vibration_g=0.0,
        peak_knock_bar=0.0,
        crankcase_pressure_mbar=-5.0 # Negative crankcase pressure
    )
    est = vs.estimate(telemetry)
    # IMEP clamped to minimum physical bound
    assert all(1.0 <= imep <= 18.0 for imep in est.imep_cyl_bar)
    assert 1.0 <= est.imep_mean_bar <= 18.0
    # Blowby must not throw domain error on negative pressure
    assert est.blowby_flow_lpm >= 0.0
    # Zero oil pressure must indicate complete loss of hydrodynamic oil film
    assert est.oil_film_thickness_um == 0.0
