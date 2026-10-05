import pytest
import math
from core.twin_core.physics_model import RotaxIndependentTwin
from core.telemetry_schema.schemas import NormalizedEngineTelemetry

def test_twin_step_and_residuals():
    twin = RotaxIndependentTwin("configs/engine.yaml")
    telemetry = NormalizedEngineTelemetry(
        timestamp=100.0,
        seq_num=1,
        rpm=5000.0,
        map_kpa=100.0,
        throttle_pct=75.0,
        bus_voltage_v=28.0,
        cht_cyl1_c=105.0,
        cht_cyl2_c=104.0,
        cht_cyl3_c=106.0,
        cht_cyl4_c=105.0,
        egt_cyl1_c=810.0,
        egt_cyl2_c=805.0,
        egt_cyl3_c=815.0,
        egt_cyl4_c=808.0,
        oil_pressure_bar=3.8,
        oil_temp_c=90.0,
        fuel_flow_lph=22.0,
        fuel_pressure_bar=3.2,
        coolant_temp_c=82.0,
        rms_vibration_g=0.9,
        peak_knock_bar=0.4,
        crankcase_pressure_mbar=2.5
    )
    pred = twin.step(telemetry)
    assert pred.predicted_rpm == 5000.0
    assert len(pred.predicted_cht_cyl) == 4
    assert len(pred.predicted_egt_cyl) == 4
    assert isinstance(pred.residual_map_kpa, float)
    assert isinstance(pred.residual_oil_pressure_bar, float)

def test_twin_division_by_zero_and_negative_roots_edge_cases():
    twin = RotaxIndependentTwin("configs/engine.yaml")
    # Extreme sensor anomalies / unphysical edge inputs
    telemetry = NormalizedEngineTelemetry(
        timestamp=100.0,
        seq_num=1,
        rpm=-500.0, # Negative RPM
        map_kpa=-20.0, # Negative MAP
        throttle_pct=0.0, # Fully closed throttle
        bus_voltage_v=28.0,
        cht_cyl1_c=-50.0,
        cht_cyl2_c=-50.0,
        cht_cyl3_c=-50.0,
        cht_cyl4_c=-50.0,
        egt_cyl1_c=-100.0,
        egt_cyl2_c=-100.0,
        egt_cyl3_c=-100.0,
        egt_cyl4_c=-100.0,
        oil_pressure_bar=-2.0, # Negative oil pressure
        oil_temp_c=-300.0, # Below absolute zero
        fuel_flow_lph=0.0, # Zero fuel flow
        fuel_pressure_bar=0.0,
        coolant_temp_c=-40.0,
        rms_vibration_g=0.0,
        peak_knock_bar=0.0,
        crankcase_pressure_mbar=-10.0,
        ambient_pressure_kpa=-5.0, # Negative ambient pressure
        ambient_temp_c=-300.0 # Below absolute zero
    )
    pred = twin.step(telemetry)
    # Must execute safely without domain errors or ZeroDivisionError
    assert 28.0 <= pred.predicted_map_kpa <= 150.0
    assert all(15.0 <= c <= 160.0 for c in pred.predicted_cht_cyl)
    assert all(600.0 <= e <= 920.0 for e in pred.predicted_egt_cyl)
    assert 1.2 <= pred.predicted_oil_pressure_bar <= 5.0
    assert not math.isnan(pred.predicted_map_kpa)
    assert not math.isinf(pred.predicted_map_kpa)

def test_twin_mvem_numerical_stability_extreme_transient():
    twin = RotaxIndependentTwin("configs/engine.yaml")
    twin.estimated_map_kpa = 35.0 # Idle MAP
    
    # Test 1: Step from idle to 100% WOT with varying dt (0.005s, 0.05s, 0.1s, 0.2s)
    for dt in [0.005, 0.05, 0.1, 0.2]:
        t = 0.0
        map_history = []
        for step in range(15):
            t += dt
            telemetry = NormalizedEngineTelemetry(
                timestamp=t,
                seq_num=step,
                rpm=5500.0,
                map_kpa=115.0,
                throttle_pct=100.0,
                bus_voltage_v=28.0,
                cht_cyl1_c=105.0,
                cht_cyl2_c=105.0,
                cht_cyl3_c=105.0,
                cht_cyl4_c=105.0,
                egt_cyl1_c=810.0,
                egt_cyl2_c=810.0,
                egt_cyl3_c=810.0,
                egt_cyl4_c=810.0,
                oil_pressure_bar=3.8,
                oil_temp_c=90.0,
                fuel_flow_lph=26.0,
                fuel_pressure_bar=3.2,
                coolant_temp_c=82.0,
                rms_vibration_g=0.9,
                peak_knock_bar=0.4,
                crankcase_pressure_mbar=2.5,
                ambient_pressure_kpa=101.3,
                ambient_temp_c=25.0
            )
            pred = twin.step(telemetry)
            map_history.append(pred.predicted_map_kpa)
            assert 28.0 <= pred.predicted_map_kpa <= 150.0

        # Verify no numerical bounce between clamp limits (e.g. 28 and 150)
        # End of transient must settle stably
        assert abs(map_history[-1] - map_history[-2]) < 2.0
        assert map_history[-1] > 100.0 # Physical boost achieved
