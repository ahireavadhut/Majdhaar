import pytest
from core.observer.observer_engine import PinnResidualObserver
from core.twin_core.physics_model import RotaxIndependentTwin
from core.telemetry_schema.schemas import NormalizedEngineTelemetry

def test_pinn_observer_inference():
    observer = PinnResidualObserver("core/observer/pinn_observer.pt")
    twin = RotaxIndependentTwin("configs/engine.yaml")
    telemetry = NormalizedEngineTelemetry(
        timestamp=100.0,
        seq_num=1,
        rpm=4800.0,
        map_kpa=95.0,
        throttle_pct=70.0,
        bus_voltage_v=28.0,
        cht_cyl1_c=102.0,
        cht_cyl2_c=101.0,
        cht_cyl3_c=103.0,
        cht_cyl4_c=102.0,
        egt_cyl1_c=800.0,
        egt_cyl2_c=795.0,
        egt_cyl3_c=805.0,
        egt_cyl4_c=800.0,
        oil_pressure_bar=3.7,
        oil_temp_c=89.0,
        fuel_flow_lph=20.0,
        fuel_pressure_bar=3.2,
        coolant_temp_c=80.0,
        rms_vibration_g=0.85,
        peak_knock_bar=0.35,
        crankcase_pressure_mbar=2.2
    )
    pred = twin.step(telemetry)
    obs = observer.evaluate(telemetry, pred)
    assert 0.0 <= obs.anomaly_score <= 1.0
    assert obs.pinn_physics_loss >= 0.0
    assert len(obs.corrected_latent_state) == 4
