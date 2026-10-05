import math
from unittest.mock import patch
import numpy as np
import pytest
from scipy.stats import invgauss

from core.prognostics.wiener_rul import WienerPrognosticsEngine
from core.telemetry_schema.schemas import (
    NormalizedEngineTelemetry,
    TwinPhysicsPrediction,
    VirtualSensorEstimate,
    ObserverInference,
    PrognosticsRUL
)


def create_sample_packets(
    timestamp: float = 100.0,
    rpm: float = 5000.0,
    cht_mean: float = 105.0,
    cht_spread: float = 2.0,
    egt_mean: float = 809.5,
    egt_spread: float = 10.0,
    oil_pressure: float = 3.8,
    oil_temp: float = 90.0,
    blowby: float = 5.5,
    vibration: float = 0.9,
    anomaly_score: float = 0.05
):
    telemetry = NormalizedEngineTelemetry(
        timestamp=timestamp,
        seq_num=1,
        rpm=rpm,
        map_kpa=100.0,
        throttle_pct=75.0,
        bus_voltage_v=28.0,
        cht_cyl1_c=cht_mean,
        cht_cyl2_c=cht_mean,
        cht_cyl3_c=cht_mean,
        cht_cyl4_c=cht_mean,
        cht_mean_c=cht_mean,
        cht_spread_c=cht_spread,
        egt_cyl1_c=egt_mean,
        egt_cyl2_c=egt_mean,
        egt_cyl3_c=egt_mean,
        egt_cyl4_c=egt_mean,
        egt_mean_c=egt_mean,
        egt_spread_c=egt_spread,
        oil_pressure_bar=oil_pressure,
        oil_temp_c=oil_temp,
        fuel_flow_lph=22.0,
        fuel_pressure_bar=3.2,
        coolant_temp_c=82.0,
        rms_vibration_g=vibration,
        peak_knock_bar=0.4,
        crankcase_pressure_mbar=2.5
    )
    twin = TwinPhysicsPrediction(
        timestamp=timestamp,
        predicted_rpm=rpm,
        predicted_map_kpa=100.0,
        predicted_cht_cyl=[cht_mean] * 4,
        predicted_egt_cyl=[egt_mean] * 4,
        predicted_oil_pressure_bar=oil_pressure,
        predicted_oil_temp_c=oil_temp,
        predicted_coolant_temp_c=82.0,
        residual_rpm=0.0,
        residual_map_kpa=0.0,
        residual_cht_cyl=[0.0] * 4,
        residual_egt_cyl=[0.0] * 4,
        residual_oil_pressure_bar=0.0,
        residual_oil_temp_c=0.0
    )
    virt = VirtualSensorEstimate(
        imep_cyl_bar=[6.5] * 4,
        imep_mean_bar=6.5,
        blowby_flow_lpm=blowby,
        oil_film_thickness_um=4.2
    )
    obs = ObserverInference(
        timestamp=timestamp,
        corrected_latent_state=[0.0] * 4,
        anomaly_score=anomaly_score,
        pinn_physics_loss=0.01,
        thermal_residual_norm=0.5,
        lube_residual_norm=0.2,
        combustion_residual_norm=0.1
    )
    return telemetry, twin, virt, obs


def test_wiener_rul_quantiles():
    """Baseline test ensuring valid health index and quantile ordering."""
    rul_engine = WienerPrognosticsEngine(tbo_hours=1500.0)
    telem, twin, virt, obs = create_sample_packets()
    rul = rul_engine.predict_rul(telem, twin, virt, obs)

    assert 0.0 < rul.health_index <= 1.0
    assert rul.rul_hours_p10 <= rul.rul_hours_p50 <= rul.rul_hours_p90
    assert "Stochastic Wiener Process" in rul.methodology


def test_fht_scipy_parameterization_mathematical_exactness():
    """
    Exhaustive mathematical verification of First Hitting Time parameter mapping in scipy.

    For dX(t) = mu dt + sigma dW(t) hitting threshold D from current degradation x:
        rem = D - x > 0
        Theoretical Mean = rem / mu
        Theoretical Variance = rem * sigma^2 / mu^3

    Scipy invgauss(mu, loc=0, scale) has:
        mean = mu_scipy * scale
        variance = mu_scipy^3 * scale^2

    Proof:
        scale = rem^2 / sigma^2 = lambda_fht
        mu_scipy = Mean / scale = sigma^2 / (rem * mu)
    """
    rem = 0.5
    mu = 0.001
    sigma = 0.008

    # Theoretical FHT moments
    expected_mean = rem / mu
    expected_var = (rem * (sigma ** 2)) / (mu ** 3)

    # Scipy mapping
    scale = (rem ** 2) / (sigma ** 2)
    mu_scipy = expected_mean / scale

    dist = invgauss(mu=mu_scipy, loc=0, scale=scale)
    scipy_mean = dist.mean()
    scipy_var = dist.var()

    # Verify moments match theoretical values to machine precision (< 1e-10 relative error)
    rel_err_mean = abs(scipy_mean - expected_mean) / expected_mean
    rel_err_var = abs(scipy_var - expected_var) / expected_var
    assert rel_err_mean < 1e-10, f"Mean mismatch: {scipy_mean} vs {expected_mean}"
    assert rel_err_var < 1e-10, f"Variance mismatch: {scipy_var} vs {expected_var}"

    # Verify engine method produces exact quantiles
    engine = WienerPrognosticsEngine(tbo_hours=1500.0)
    p10, p50, p90 = engine.compute_fht_quantiles(rem, mu, sigma)

    assert p10 <= p50 <= p90
    # For right-skewed Inverse Gaussian, median is strictly less than mean
    assert p50 < expected_mean
    # Numerical values check
    assert abs(p50 - 470.2) < 1.0
    assert abs(p10 - 301.1) < 1.0
    assert abs(p90 - 737.1) < 1.0


def test_boundary_failure_threshold_reached_or_exceeded():
    """
    Boundary audit:
    When degradation x >= failure_threshold_d (rem <= 0),
    RUL must immediately report 0.0 hours (not negative, not NaN, not crash).
    """
    engine = WienerPrognosticsEngine(tbo_hours=1500.0, failure_threshold_d=1.0)

    # 1. Direct computation with rem = 0.0 and rem < 0.0
    assert engine.compute_fht_quantiles(0.0, 0.001, 0.008) == (0.0, 0.0, 0.0)
    assert engine.compute_fht_quantiles(-0.15, 0.001, 0.008) == (0.0, 0.0, 0.0)

    # 2. Simulate engine reaching failure threshold through accumulated wear
    engine.accumulated_degradation = 1.0
    engine.health_index = 0.0
    telem, twin, virt, obs = create_sample_packets(timestamp=200.0)
    rul = engine.predict_rul(telem, twin, virt, obs)

    assert rul.rul_hours_p10 == 0.0
    assert rul.rul_hours_p50 == 0.0
    assert rul.rul_hours_p90 == 0.0
    assert rul.health_index == 0.0
    assert not math.isnan(rul.rul_hours_p10)
    assert not math.isnan(rul.rul_hours_p50)
    assert not math.isnan(rul.rul_hours_p90)
    assert "Failure threshold D reached or exceeded" in rul.validation_status

    # 3. Simulate degradation exceeding threshold (x > 1.0)
    engine.accumulated_degradation = 1.05
    rul_exceeded = engine.predict_rul(telem, twin, virt, obs)
    assert rul_exceeded.rul_hours_p10 == 0.0
    assert rul_exceeded.rul_hours_p50 == 0.0
    assert rul_exceeded.rul_hours_p90 == 0.0
    assert rul_exceeded.health_index == 0.0


def test_boundary_zero_and_negative_drift():
    """
    Boundary audit:
    Guard against drift mu <= 0 or close to 0 to prevent division by zero or NaN.
    """
    engine = WienerPrognosticsEngine(tbo_hours=1500.0)

    # Drift exactly 0
    p10_z, p50_z, p90_z = engine.compute_fht_quantiles(rem_margin=0.5, mu=0.0, sigma=0.008)
    assert p10_z <= p50_z <= p90_z
    assert not math.isnan(p50_z)
    assert not math.isinf(p50_z)
    assert p10_z >= 0.0

    # Negative drift
    p10_n, p50_n, p90_n = engine.compute_fht_quantiles(rem_margin=0.5, mu=-0.002, sigma=0.008)
    assert p10_n <= p50_n <= p90_n
    assert not math.isnan(p50_n)
    assert not math.isinf(p50_n)
    assert p10_n >= 0.0

    # Extremely small positive drift (near machine epsilon)
    p10_e, p50_e, p90_e = engine.compute_fht_quantiles(rem_margin=0.5, mu=1e-15, sigma=0.008)
    assert p10_e <= p50_e <= p90_e
    assert not math.isnan(p50_e)


def test_scipy_numerical_fallback_on_invalid_ppf():
    """
    Numerical robustness audit:
    Verify that if scipy invgauss.ppf raises an exception or returns NaN,
    the engine gracefully falls back to deterministic asymptotic moment-matching quantiles.
    """
    engine = WienerPrognosticsEngine(tbo_hours=1500.0)

    with patch("core.prognostics.wiener_rul.invgauss.ppf", side_effect=FloatingPointError("Simulated solver overflow")):
        p10, p50, p90 = engine.compute_fht_quantiles(rem_margin=0.5, mu=0.001, sigma=0.008)
        assert p10 <= p50 <= p90
        assert not math.isnan(p10)
        assert not math.isnan(p50)
        assert not math.isnan(p90)
        assert p10 > 0.0
        assert abs(p50 - 500.0) < 1.0  # Mean RUL is 500.0 hr

    with patch("core.prognostics.wiener_rul.invgauss.ppf", return_value=float("nan")):
        p10_nan, p50_nan, p90_nan = engine.compute_fht_quantiles(rem_margin=0.5, mu=0.001, sigma=0.008)
        assert p10_nan <= p50_nan <= p90_nan
        assert not math.isnan(p10_nan)
        assert not math.isnan(p50_nan)
        assert not math.isnan(p90_nan)


def test_monotonic_filtering_prevents_artificial_rejuvenation():
    """
    Physical irreversibility audit:
    Ensure monotonic filtering prevents transient sensor cooling or throttling
    from causing artificial rejuvenation (negative wear / increasing health index).
    """
    engine = WienerPrognosticsEngine(tbo_hours=1500.0)

    # Step 1: Initial state
    t0 = 100.0
    telem_init, twin_init, virt_init, obs_init = create_sample_packets(timestamp=t0)
    engine.update_health_index(telem_init, twin_init, virt_init, obs_init)
    h_init = engine.health_index
    deg_init = engine.accumulated_degradation

    # Step 2: Severe fault causes degradation
    t1 = t0 + 10.0
    telem_fault, twin_fault, virt_fault, obs_fault = create_sample_packets(
        timestamp=t1,
        cht_mean=140.0,
        oil_pressure=1.2,
        blowby=28.0,
        anomaly_score=0.90
    )
    engine.update_health_index(telem_fault, twin_fault, virt_fault, obs_fault)
    h_fault = engine.health_index
    deg_fault = engine.accumulated_degradation

    assert deg_fault > deg_init, "Degradation should have increased during fault"
    assert h_fault < h_init, "Health index should have decayed during fault"

    # Step 3: Engine is throttled down to ground idle with pristine readings
    # Raw distress is 0.0. Verify that health index does NOT rebound upwards!
    curr_h = h_fault
    curr_deg = deg_fault
    for step in range(1, 60):
        t_idle = t1 + step * 1.0
        telem_idle, twin_idle, virt_idle, obs_idle = create_sample_packets(
            timestamp=t_idle,
            rpm=2000.0,
            cht_mean=80.0,
            oil_pressure=4.2,
            blowby=3.0,
            vibration=0.3,
            anomaly_score=0.0
        )
        engine.update_health_index(telem_idle, twin_idle, virt_idle, obs_idle)

        assert engine.health_index <= curr_h, f"Health index rejuvenated at step {step}!"
        assert engine.accumulated_degradation >= curr_deg, f"Degradation decreased at step {step}!"
        curr_h = engine.health_index
        curr_deg = engine.accumulated_degradation


def test_distress_high_frequency_noise_filter():
    """
    Signal filtering audit:
    Verify that an isolated 1-sample spike in distress is smoothed by the EMA filter
    rather than causing an abrupt discontinuous jump in wear rate.
    """
    engine = WienerPrognosticsEngine(tbo_hours=1500.0, ema_alpha=0.15)

    # Step 1: Nominal baseline
    telem_nom, twin_nom, virt_nom, obs_nom = create_sample_packets(timestamp=100.0)
    engine.update_health_index(telem_nom, twin_nom, virt_nom, obs_nom)
    filtered_nom = engine.filtered_distress

    # Step 2: 1-sample noise spike
    telem_spike, twin_spike, virt_spike, obs_spike = create_sample_packets(
        timestamp=101.0,
        cht_mean=145.0,
        blowby=30.0,
        anomaly_score=1.0
    )
    engine.update_health_index(telem_spike, twin_spike, virt_spike, obs_spike)
    filtered_spike = engine.filtered_distress

    # Filtered distress must be substantially lower than raw distress (~1.0)
    assert filtered_spike < 0.40, f"EMA filter failed to dampen spike: {filtered_spike}"
    assert filtered_spike > filtered_nom


def test_online_parameter_estimation_mle():
    """
    Online estimation audit:
    Verify that Maximum Likelihood Estimation on rolling increments recovers
    known drift and diffusion parameters accurately.
    """
    engine = WienerPrognosticsEngine(tbo_hours=1500.0, history_window_size=100)

    # Feed synthetic increments with known drift rate mu_true = 0.003 / hr
    mu_true = 0.003
    dt_hr = 1.0 / 3600.0  # 1 second intervals

    for _ in range(80):
        engine.history.append((mu_true * dt_hr, dt_hr))

    mu_mle, sigma_mle = engine.estimate_parameters_from_history()
    assert abs(mu_mle - mu_true) < 1e-6, f"MLE drift mismatch: {mu_mle} vs {mu_true}"
    # Uniform increments produce negligible variance
    assert sigma_mle < 1e-3


def test_online_adaptive_drift_tracking():
    """
    Adaptive tracking audit:
    Verify that persistent accelerated wear dynamically updates the drift parameter
    and accelerates RUL quantile reduction.
    """
    engine = WienerPrognosticsEngine(tbo_hours=1500.0, prior_sample_weight=20.0)

    # Baseline health prediction
    telem_nom, twin_nom, virt_nom, obs_nom = create_sample_packets(timestamp=100.0)
    rul_init = engine.predict_rul(telem_nom, twin_nom, virt_nom, obs_nom)

    # Run 40 steps of severe accelerated degradation
    for i in range(1, 41):
        t = 100.0 + i * 1.0
        telem_fault, twin_fault, virt_fault, obs_fault = create_sample_packets(
            timestamp=t,
            rpm=5500.0,
            cht_mean=135.0,
            oil_pressure=1.5,
            blowby=25.0,
            anomaly_score=0.85
        )
        rul_fault = engine.predict_rul(telem_fault, twin_fault, virt_fault, obs_fault)

    # Adapted drift mu should have increased significantly over baseline
    assert rul_fault.drift_mu > rul_init.drift_mu
    # RUL P50 should have decreased substantially
    assert rul_fault.rul_hours_p50 < rul_init.rul_hours_p50
    assert rul_fault.rul_hours_p10 <= rul_fault.rul_hours_p50 <= rul_fault.rul_hours_p90
