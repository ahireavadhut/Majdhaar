"""
Stochastic Wiener Degradation Process & Inverse-Gaussian FHT RUL Estimator
Implements Section 10 & 11 of Implementation Plan and Section 5 of Research Report.
Adheres strictly to Rule 5: Explicit model assumptions and limitation disclosures.

Mathematical Foundation:
------------------------
Degradation SDE:
    dX(t) = mu * dt + sigma * dW(t)

First Passage Time T to threshold D starting from degradation x (rem = D - x > 0):
    E[T] = rem / mu
    Var[T] = (rem * sigma^2) / mu^3

Scipy Parameterization (invgauss):
    scipy.stats.invgauss(mu, loc=0, scale) has:
        mean = mu_scipy * scale
        var = mu_scipy^3 * scale^2
    Equating to FHT:
        mu_scipy * scale = rem / mu
        mu_scipy^3 * scale^2 = (rem * sigma^2) / mu^3
    Dividing var by (mean)^2:
        mu_scipy = sigma^2 / (rem * mu)
    Therefore:
        scale = mean / mu_scipy = (rem^2) / (sigma^2) = lambda_fht
        mu_scipy = mean / scale = sigma^2 / (rem * mu)
"""
import math
from collections import deque
from typing import Tuple, Optional
import numpy as np
from scipy.stats import invgauss
from core.telemetry_schema.schemas import (
    NormalizedEngineTelemetry,
    TwinPhysicsPrediction,
    VirtualSensorEstimate,
    ObserverInference,
    PrognosticsRUL
)


class WienerPrognosticsEngine:
    def __init__(
        self,
        tbo_hours: float = 1500.0,
        diffusion_sigma_w: float = 0.008,
        failure_threshold_d: float = 1.0,
        history_window_size: int = 120,
        ema_alpha: float = 0.15,
        prior_sample_weight: float = 30.0
    ):
        self.tbo_hours = max(1.0, float(tbo_hours))
        self.failure_threshold_d = float(failure_threshold_d)
        self.sigma_w = max(1e-6, float(diffusion_sigma_w))

        # Baseline drift rate: ~1.0 / 1500 hr = 0.000667 / hr
        self.mu_0 = 1.0 / self.tbo_hours
        self.beta_1 = 0.015  # Arrhenius thermal degradation coefficient
        self.t_ref_cht = 100.0  # deg C reference temperature

        # Running health index (1.0 = pristine, 0.0 = failure threshold reached)
        self.health_index = 0.985
        self.accumulated_degradation = 0.015  # 1.0 - health_index
        self.operational_hours = 22.5  # Initial hours on airframe

        # High-frequency noise filter for distress factor
        self.filtered_distress = 0.0
        self.ema_alpha = float(ema_alpha)
        self.last_timestamp: Optional[float] = None

        # Rolling history buffer for online parameter estimation: deque of (dx, dt_hr)
        self.history_window_size = int(history_window_size)
        self.history: deque = deque(maxlen=self.history_window_size)
        self.prior_sample_weight = float(prior_sample_weight)

        # Current estimated/adapted drift and diffusion
        self.current_drift_mu = self.mu_0
        self.current_diffusion_sigma = self.sigma_w

    def estimate_parameters_from_history(self) -> Tuple[float, float]:
        """
        Online Maximum Likelihood Estimation (MLE) of drift mu and diffusion sigma
        from the rolling history of degradation increments.
        Returns:
            (mu_mle, sigma_mle) in units [degradation / flight hour] and [degradation / sqrt(flight hour)]
        """
        m = len(self.history)
        if m == 0:
            return self.mu_0, self.sigma_w

        total_dx = sum(item[0] for item in self.history)
        total_dt = sum(item[1] for item in self.history)

        if total_dt <= 1e-9:
            return self.mu_0, self.sigma_w

        # Empirical MLE drift rate: delta_x_sum / delta_t_sum
        mu_mle = max(0.0, total_dx / total_dt)

        # Empirical MLE diffusion variance: (1/M) * sum((dx_i - mu_mle * dt_i)^2 / dt_i)
        if m >= 2:
            var_mle = sum(((dx - mu_mle * dt) ** 2) / dt for dx, dt in self.history) / m
            sigma_mle = math.sqrt(max(0.0, var_mle))
            # Bound within physical domain
            sigma_mle = max(1e-4, min(0.05, sigma_mle))
        else:
            sigma_mle = self.sigma_w

        return mu_mle, sigma_mle

    def compute_fht_quantiles(
        self,
        rem_margin: float,
        mu: float,
        sigma: float
    ) -> Tuple[float, float, float]:
        """
        Computes First Hitting Time (FHT) quantiles (P10, P50, P90) using the
        mathematically exact Inverse Gaussian (Wald) distribution via scipy.stats.invgauss.

        Derivation of exact mapping:
            Mean = rem / mu
            Var  = rem * sigma^2 / mu^3
            scale = lambda_fht = rem^2 / sigma^2
            mu_scipy = Mean / scale = sigma^2 / (rem * mu)

        Edge cases:
            - If rem_margin <= 0: Engine has reached failure threshold D -> returns (0.0, 0.0, 0.0).
            - If mu <= 0: Protected by epsilon floor to guard against division by zero.
            - If invgauss numerical inversion returns NaN, Inf, or raises exception:
              Falls back to deterministic asymptotic moment-matching quantiles.
        """
        if rem_margin <= 0.0:
            return 0.0, 0.0, 0.0

        # Guard against zero/negative drift and non-positive diffusion
        mu_safe = max(1e-9, float(mu))
        sigma_safe = max(1e-6, float(sigma))

        mean_rul_hours = rem_margin / mu_safe
        lambda_fht = (rem_margin ** 2) / (sigma_safe ** 2)

        # Exact scipy parameter mapping
        scale_param = lambda_fht
        mu_scipy = mean_rul_hours / scale_param

        try:
            p10 = float(invgauss.ppf(0.10, mu=mu_scipy, scale=scale_param))
            p50 = float(invgauss.ppf(0.50, mu=mu_scipy, scale=scale_param))
            p90 = float(invgauss.ppf(0.90, mu=mu_scipy, scale=scale_param))

            if any(math.isnan(p) or math.isinf(p) or p < 0.0 for p in (p10, p50, p90)):
                raise FloatingPointError("Numerical instability detected in invgauss.ppf")
        except Exception:
            # Deterministic moment-matching fallback
            std_rul = math.sqrt(max(0.0, rem_margin * (sigma_safe ** 2) / (mu_safe ** 3)))
            p50 = mean_rul_hours
            p10 = max(0.0, mean_rul_hours - 1.28155 * std_rul)
            p90 = mean_rul_hours + 1.28155 * std_rul

        # Quantile ordering & physical boundary guarantees for operational airframe
        p10 = max(0.1, round(p10, 1))
        p50 = max(p10, round(p50, 1))
        p90 = max(p50, round(p90, 1))

        return p10, p50, p90

    def update_health_index(
        self,
        telemetry: NormalizedEngineTelemetry,
        twin: TwinPhysicsPrediction,
        virtual: VirtualSensorEstimate,
        observer: ObserverInference
    ) -> float:
        """
        Derives Health Index H(t) from multi-sensor evidence.
        Features: thermal spread, lubrication residuals, blowby, and PINN anomaly score.

        Monotonic filtering and noise reduction:
        1. Applies Exponential Moving Average (EMA) low-pass filtering to instantaneous distress
           to reject high-frequency sensor jitter.
        2. Strictly enforces monotonic degradation: accumulated degradation X(t) is non-decreasing,
           preventing transient sensor cooling or throttling from creating artificial engine rejuvenation.
        3. Records degradation increments into a rolling history buffer for online drift/diffusion estimation.
        """
        # Thermal penalty
        cht_penalty = max(0.0, (telemetry.cht_mean_c - 110.0) / 25.0) + (telemetry.cht_spread_c / 40.0)
        egt_penalty = max(0.0, (telemetry.egt_spread_c - 40.0) / 100.0)

        # Lubrication penalty
        oil_p_penalty = max(0.0, (2.8 - telemetry.oil_pressure_bar) / 1.5)
        oil_t_penalty = max(0.0, (telemetry.oil_temp_c - 105.0) / 25.0)

        # Blowby & Vibration penalty
        blowby_penalty = max(0.0, (virtual.blowby_flow_lpm - 12.0) / 15.0)
        vib_penalty = max(0.0, (telemetry.rms_vibration_g - 1.5) / 3.0)

        # Total instantaneous distress factor [0.0 to 1.0]
        raw_distress = (
            0.25 * cht_penalty +
            0.20 * egt_penalty +
            0.25 * oil_p_penalty +
            0.10 * oil_t_penalty +
            0.10 * blowby_penalty +
            0.10 * vib_penalty +
            0.20 * observer.anomaly_score
        )
        raw_distress = max(0.0, min(1.0, raw_distress))

        # 1. High-frequency noise filtering via EMA
        if self.last_timestamp is None:
            self.filtered_distress = raw_distress
            dt_sec = 1.0
        else:
            dt_sec = telemetry.timestamp - self.last_timestamp
            if dt_sec <= 0.0 or dt_sec > 3600.0:
                dt_sec = 1.0
            self.filtered_distress = (1.0 - self.ema_alpha) * self.filtered_distress + self.ema_alpha * raw_distress

        self.last_timestamp = telemetry.timestamp
        dt_hr = dt_sec / 3600.0

        # Dynamic health index decay step
        decay_step = (self.mu_0 * (1.0 + 12.0 * self.filtered_distress)) * dt_hr

        # 2. Strictly monotonic filtering: accumulated degradation cannot decrease
        old_degradation = self.accumulated_degradation
        new_degradation = max(old_degradation, old_degradation + decay_step)
        self.accumulated_degradation = min(self.failure_threshold_d, new_degradation)

        # Health index is monotonically non-increasing
        if self.accumulated_degradation >= self.failure_threshold_d:
            self.health_index = 0.0
        else:
            self.health_index = max(0.0, min(self.health_index, 1.0 - self.accumulated_degradation))

        # Record degradation increment in rolling history buffer for online parameter estimation
        dx = self.accumulated_degradation - old_degradation
        if dx >= 0.0 and dt_hr > 0.0:
            self.history.append((dx, dt_hr))

        return self.health_index

    def predict_rul(
        self,
        telemetry: NormalizedEngineTelemetry,
        twin: TwinPhysicsPrediction,
        virtual: VirtualSensorEstimate,
        observer: ObserverInference
    ) -> PrognosticsRUL:
        """
        Calculates P10/P50/P90 RUL quantiles using the First Hitting Time (FHT)
        density function of the Wiener process with online Bayesian / MLE adaptive drift.
        """
        h_idx = self.update_health_index(telemetry, twin, virtual, observer)
        x_t = self.accumulated_degradation

        # Load Severity Multiplier:
        # mu(Severity) = mu_0 * (N / N_rated)^2 * exp(beta_1 * (T_CHT - T_ref)) * (1 + 3 * (1 - HealthIndex))
        n_ratio = telemetry.rpm / 5500.0
        t_cht = telemetry.cht_mean_c
        thermal_term = math.exp(self.beta_1 * max(0.0, t_cht - self.t_ref_cht))
        health_wear_mult = 1.0 + 3.0 * (1.0 - h_idx)

        severity = (n_ratio ** 2) * thermal_term * health_wear_mult
        severity = max(0.2, min(8.0, severity))
        mu_physics = self.mu_0 * severity

        # Online Bayesian / MLE parameter estimation from rolling history (Section 10)
        mu_mle, sigma_mle = self.estimate_parameters_from_history()
        m = len(self.history)
        weight = m / (m + self.prior_sample_weight)

        # Adaptive drift tracking: blends physics-based prior with empirical rolling wear
        mu_adapted = (1.0 - weight) * mu_physics + weight * mu_mle
        mu_adapted = max(1e-9, mu_adapted)

        # Adaptive diffusion tracking:
        sigma_adapted = math.sqrt((1.0 - weight) * (self.sigma_w ** 2) + weight * (sigma_mle ** 2))
        sigma_adapted = max(1e-4, min(0.05, sigma_adapted))

        self.current_drift_mu = mu_adapted
        self.current_diffusion_sigma = sigma_adapted

        # Remaining distance to failure threshold D
        rem_margin = self.failure_threshold_d - x_t

        if rem_margin <= 0.0:
            return PrognosticsRUL(
                timestamp=telemetry.timestamp,
                health_index=0.0,
                load_severity_factor=round(severity, 2),
                drift_mu=round(mu_adapted, 6),
                diffusion_sigma=round(sigma_adapted, 6),
                failure_threshold_d=self.failure_threshold_d,
                current_degradation_x=round(x_t, 4),
                rul_hours_p10=0.0,
                rul_hours_p50=0.0,
                rul_hours_p90=0.0,
                estimated_tbo_hours=self.tbo_hours,
                methodology="Stochastic Wiener Process with state-dependent severity drift & inverse-Gaussian FHT",
                validation_status="MODEL_DERIVED_PROGNOSTICS (Failure threshold D reached or exceeded - TBO expired)"
            )

        p10, p50, p90 = self.compute_fht_quantiles(rem_margin, mu_adapted, sigma_adapted)

        return PrognosticsRUL(
            timestamp=telemetry.timestamp,
            health_index=round(h_idx, 4),
            load_severity_factor=round(severity, 2),
            drift_mu=round(mu_adapted, 6),
            diffusion_sigma=round(sigma_adapted, 6),
            failure_threshold_d=self.failure_threshold_d,
            current_degradation_x=round(x_t, 4),
            rul_hours_p10=p10,
            rul_hours_p50=p50,
            rul_hours_p90=p90,
            estimated_tbo_hours=self.tbo_hours,
            methodology="Stochastic Wiener Process with state-dependent severity drift & inverse-Gaussian FHT",
            validation_status="MODEL_DERIVED_PROGNOSTICS (No destructive physical wear datasets fabricated)"
        )
