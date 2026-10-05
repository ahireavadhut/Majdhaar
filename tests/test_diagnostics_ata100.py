import pytest
from core.diagnostics.fault_classifier import FaultDiagnosticsEngine
from core.telemetry_schema.schemas import NormalizedEngineTelemetry, TwinPhysicsPrediction, VirtualSensorEstimate, ObserverInference

def test_diagnostics_all_ata_chapters():
    diag = FaultDiagnosticsEngine()
    
    # 1. Test Injector Coking (ATA-73)
    t_coking = NormalizedEngineTelemetry(
        timestamp=10.0, seq_num=1, rpm=4900.0, map_kpa=98.0, throttle_pct=75.0, bus_voltage_v=28.0,
        cht_cyl1_c=105.0, cht_cyl2_c=104.0, cht_cyl3_c=106.0, cht_cyl4_c=105.0, cht_mean_c=105.0, cht_spread_c=2.0,
        egt_cyl1_c=805.0, egt_cyl2_c=800.0, egt_cyl3_c=875.0, egt_cyl4_c=805.0, egt_mean_c=821.2, egt_spread_c=75.0,
        oil_pressure_bar=3.8, oil_temp_c=92.0, fuel_flow_lph=22.0, fuel_pressure_bar=3.2, coolant_temp_c=82.0,
        rms_vibration_g=0.9, peak_knock_bar=0.4, crankcase_pressure_mbar=2.5
    )
    twin_nom = TwinPhysicsPrediction(
        timestamp=10.0, predicted_rpm=4900.0, predicted_map_kpa=98.0, predicted_cht_cyl=[105.0]*4, predicted_egt_cyl=[805.0]*4,
        predicted_oil_pressure_bar=3.8, predicted_oil_temp_c=92.0, predicted_coolant_temp_c=82.0, residual_rpm=0.0,
        residual_map_kpa=0.0, residual_cht_cyl=[0.0]*4, residual_egt_cyl=[0.0, -5.0, 70.0, 0.0], residual_oil_pressure_bar=0.0, residual_oil_temp_c=0.0
    )
    virt_nom = VirtualSensorEstimate(imep_cyl_bar=[6.5]*4, imep_mean_bar=6.5, blowby_flow_lpm=5.5, oil_film_thickness_um=4.2)
    obs_nom = ObserverInference(timestamp=10.0, corrected_latent_state=[0.0]*4, anomaly_score=0.4, pinn_physics_loss=0.05, thermal_residual_norm=12.0, lube_residual_norm=0.2, combustion_residual_norm=0.1)

    rep = diag.diagnose(t_coking, twin_nom, virt_nom, obs_nom)
    assert rep.fault_detected is True
    assert rep.fault_type == "INJECTOR_COKING"
    assert "ATA-73" in rep.ata100_chapter
