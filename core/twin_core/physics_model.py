"""
Independent Digital Twin Core (0D/1D Thermodynamic & Mean Value Engine Model)
Complies with Section 5: Reduced-order physics estimation model.
Equations are independently testable and parameterized for Rotax 914/915 iS.
"""
import math
import yaml
from pathlib import Path
from typing import Dict, Any, List, Tuple
from core.telemetry_schema.schemas import NormalizedEngineTelemetry, TwinPhysicsPrediction

class RotaxIndependentTwin:
    def __init__(self, config_path: str = "configs/engine.yaml"):
        self.config_path = Path(config_path)
        self._load_config()

        # Internal model states
        self.estimated_map_kpa = 98.0
        self.estimated_cht_c = [95.0, 95.0, 95.0, 95.0]
        self.estimated_egt_c = [780.0, 780.0, 780.0, 780.0]
        self.estimated_oil_pressure_bar = 3.5
        self.estimated_oil_temp_c = 88.0
        self.last_timestamp = None

    def _load_config(self):
        if not self.config_path.exists():
            raise FileNotFoundError(f"Config not found: {self.config_path}")
        with open(self.config_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)

        # Geometry
        geo = cfg.get("geometry", {})
        self.bore_m = geo.get("bore_m", 0.0795)
        self.stroke_m = geo.get("stroke_m", 0.0610)
        self.num_cylinders = geo.get("cylinders", 4)
        self.displacement_m3 = geo.get("displacement_m3", 0.001211) # 1.211 Liters
        self.comp_ratio = geo.get("compression_ratio", 9.0)

        # Thermal Network
        th = cfg.get("lumped_thermal_network", {})
        self.c_head = th.get("c_head_j_per_k", 450.0)
        self.r_coolant = th.get("r_coolant_k_per_w", 0.085)
        self.r_ambient = th.get("r_ambient_k_per_w", 0.22)
        self.woschni_c1 = th.get("woschni_c1", 3.26)
        self.woschni_p_exp = th.get("woschni_p_exp", 0.8)
        self.woschni_t_exp = th.get("woschni_t_exp", -0.55)
        self.woschni_w_exp = th.get("woschni_w_exp", 0.8)

        # Gas constants
        self.R_air = 287.05 # J/(kg*K)
        self.T_man_k = 310.0 # 37 deg C manifold air temp
        self.V_man_m3 = 0.0025 # Intake manifold plenum volume

        # Calibrated baseline parameters for Rotax 914/915 iS
        self.throttle_discharge_cd = 0.65
        self.throttle_area_max_m2 = 0.0012 # 40-42 mm throttle bore
        self.idle_leak_area_m2 = 3.6e-5 # Idle bypass passage (~36 mm^2)
        self.volumetric_efficiency_base = 0.88
        self.combustion_efficiency = 0.34
        self.fuel_heating_value_j_per_kg = 44.0e6 # Gasoline LHV 44 MJ/kg
        self.fuel_density_kg_per_l = 0.745
        self.max_boost_kpa = 18.0 # Turbocharger boost margin above ambient

    def calibrate(self, calibration_params: Dict[str, float]):
        """Updates physics parameters from healthy baseline run calibration."""
        for k, v in calibration_params.items():
            if hasattr(self, k):
                setattr(self, k, v)

    def step(self, telemetry: NormalizedEngineTelemetry) -> TwinPhysicsPrediction:
        """
        Advances the independent physics twin by dt and computes residuals:
        r(t) = Y_sensor(t) - Y_phys(t)
        """
        now = telemetry.timestamp
        if self.last_timestamp is None or now <= self.last_timestamp:
            dt = 0.02 # 50 Hz default
        else:
            dt = min(0.2, max(0.001, now - self.last_timestamp))
        self.last_timestamp = now

        rpm = max(800.0, min(7000.0, telemetry.rpm))
        throttle_ratio = max(0.0, min(1.0, telemetry.throttle_pct / 100.0))
        t_amb_k = max(213.15, min(340.0, telemetry.ambient_temp_c + 273.15))
        p_amb_kpa = max(20.0, min(120.0, telemetry.ambient_pressure_kpa))
        t_coolant_c = telemetry.coolant_temp_c

        # 1. MVEM Intake Plenum Dynamics
        # Rotax 914/915 TCU wastegate & turbocharger compressor boost
        boost_kpa = self.max_boost_kpa * max(0.0, (rpm - 2800.0) / 3000.0) * max(0.0, (throttle_ratio - 0.45) / 0.55)**1.5
        p_upstream = p_amb_kpa + boost_kpa

        # Effective throttle flow area (butterfly valve geometry with idle bypass)
        throttle_geom = max(0.0, 1.0 - math.cos(throttle_ratio * math.pi / 2.0))
        a_eff = self.idle_leak_area_m2 + self.throttle_area_max_m2 * (throttle_geom**1.3)
        gamma = 1.4

        # Unconditionally stable sub-stepped integrator (sub_dt <= 2 ms)
        # Prevents stiffness blowups across arbitrary dt (up to 0.2s) and extreme throttle steps
        n_substeps = max(1, int(math.ceil(dt / 0.002)))
        sub_dt = dt / n_substeps

        for _ in range(n_substeps):
            if self.estimated_map_kpa >= p_upstream:
                psi = 0.0
            else:
                p_ratio = max(0.05, min(0.999, self.estimated_map_kpa / max(10.0, p_upstream)))
                if p_ratio >= 0.528: # Subsonic
                    diff = max(0.0, p_ratio**(2.0 / gamma) - p_ratio**((gamma + 1.0) / gamma))
                    psi = math.sqrt((2.0 * gamma / (gamma - 1.0)) * diff)
                else: # Sonic / Choked
                    psi = math.sqrt(gamma * (2.0 / (gamma + 1.0))**((gamma + 1.0) / (gamma - 1.0)))

            m_dot_air_in = self.throttle_discharge_cd * a_eff * (p_upstream * 1000.0) / math.sqrt(self.R_air * t_amb_k) * psi

            eta_v = self.volumetric_efficiency_base * (1.0 + 0.10 * (self.estimated_map_kpa / 100.0) - 0.06 * (rpm / 5500.0)**2)
            eta_v = max(0.60, min(1.15, eta_v))
            m_dot_air_out = (eta_v * self.displacement_m3 * (rpm / 60.0) * (self.estimated_map_kpa * 1000.0)) / (2.0 * self.R_air * self.T_man_k)

            dp_map_dt = (self.R_air * self.T_man_k / max(1e-5, self.V_man_m3)) * (m_dot_air_in - m_dot_air_out) / 1000.0
            self.estimated_map_kpa += dp_map_dt * sub_dt
            self.estimated_map_kpa = max(28.0, min(150.0, self.estimated_map_kpa))

        # 2. In-Cylinder Combustion Power & Heat Release (Wiebe proxy)
        fuel_flow_lph = max(0.0, telemetry.fuel_flow_lph)
        fuel_flow_kg_s = (fuel_flow_lph * self.fuel_density_kg_per_l) / 3600.0
        q_dot_total_fuel_w = fuel_flow_kg_s * self.fuel_heating_value_j_per_kg

        # Thermal power allocated to cylinder head metal node (~1.5% - 3% of fuel power)
        head_fraction = 0.0155 * (1.0 + 1.05 * math.exp(-rpm / 2000.0))
        q_dot_head_per_cyl = (q_dot_total_fuel_w * head_fraction) / 4.0

        # Convective cooling scaling: water pump flow rate scales with RPM; cowl ram air scales with flight speed
        r_coolant_eff = self.r_coolant / max(0.5, (0.4 + 0.6 * (rpm / 4000.0)))
        speed_scale = max(0.2, rpm / 3000.0)**0.6
        r_ambient_eff = self.r_ambient / max(0.5, speed_scale)

        # 3. 4-Node CHT Thermal Balance
        predicted_chts = []
        for i in range(4):
            t_curr = self.estimated_cht_c[i]
            q_coolant = (t_curr - t_coolant_c) / max(1e-4, r_coolant_eff)
            q_ambient = (t_curr - telemetry.ambient_temp_c) / max(1e-4, r_ambient_eff)
            dq_net = q_dot_head_per_cyl - q_coolant - q_ambient
            dt_cht = (dq_net / max(1.0, self.c_head)) * dt
            self.estimated_cht_c[i] = max(15.0, min(160.0, t_curr + dt_cht))
            predicted_chts.append(round(self.estimated_cht_c[i], 1))

        # 4. Exhaust Gas Temperature (EGT) Model
        air_per_cyl_kg_s = max(0.0005, m_dot_air_out / 4.0)
        fuel_per_cyl_kg_s = max(0.00002, fuel_flow_kg_s / 4.0)
        actual_afr = air_per_cyl_kg_s / fuel_per_cyl_kg_s
        lam = actual_afr / 14.7

        base_egt = 675.0 + 95.0 * (self.estimated_map_kpa / 100.0) + 50.0 * (rpm / 5000.0) - 40.0 * abs(lam - 1.0)
        egt_target = max(600.0, min(920.0, base_egt))

        # Unconditionally stable analytical exponential relaxation (alpha in [0, 1))
        tau_egt = 1.2
        alpha_egt = 1.0 - math.exp(-dt / max(0.01, tau_egt))
        predicted_egts = []
        for i in range(4):
            self.estimated_egt_c[i] += (egt_target - self.estimated_egt_c[i]) * alpha_egt
            predicted_egts.append(round(self.estimated_egt_c[i], 1))

        # 5. Lubrication Circuit Dynamics (Oil Pressure & Temperature)
        t_oil_k = max(213.15, min(420.0, telemetry.oil_temp_c + 273.15))
        visc_factor = math.exp(650.0 / t_oil_k - 650.0 / 363.15)
        nominal_oil_press = 2.3 + 1.8 * (rpm / 5500.0) * min(1.2, max(0.6, visc_factor))
        self.estimated_oil_pressure_bar = round(min(5.0, max(1.2, nominal_oil_press)), 3)
        self.estimated_oil_temp_c = round(t_coolant_c + 2.0 + 11.0 * (rpm / 5500.0), 1)

        # Residual Calculation: r(t) = Y_sensor(t) - Y_phys(t)
        res_rpm = round(telemetry.rpm - rpm, 1)
        res_map = round(telemetry.map_kpa - self.estimated_map_kpa, 2)
        res_cht = [
            round(telemetry.cht_cyl1_c - predicted_chts[0], 1),
            round(telemetry.cht_cyl2_c - predicted_chts[1], 1),
            round(telemetry.cht_cyl3_c - predicted_chts[2], 1),
            round(telemetry.cht_cyl4_c - predicted_chts[3], 1),
        ]
        res_egt = [
            round(telemetry.egt_cyl1_c - predicted_egts[0], 1),
            round(telemetry.egt_cyl2_c - predicted_egts[1], 1),
            round(telemetry.egt_cyl3_c - predicted_egts[2], 1),
            round(telemetry.egt_cyl4_c - predicted_egts[3], 1),
        ]
        res_oil_press = round(telemetry.oil_pressure_bar - self.estimated_oil_pressure_bar, 3)
        res_oil_temp = round(telemetry.oil_temp_c - self.estimated_oil_temp_c, 1)

        return TwinPhysicsPrediction(
            timestamp=now,
            predicted_rpm=rpm,
            predicted_map_kpa=round(self.estimated_map_kpa, 2),
            predicted_cht_cyl=predicted_chts,
            predicted_egt_cyl=predicted_egts,
            predicted_oil_pressure_bar=self.estimated_oil_pressure_bar,
            predicted_oil_temp_c=self.estimated_oil_temp_c,
            predicted_coolant_temp_c=round(t_coolant_c, 1),
            residual_rpm=res_rpm,
            residual_map_kpa=res_map,
            residual_cht_cyl=res_cht,
            residual_egt_cyl=res_egt,
            residual_oil_pressure_bar=res_oil_press,
            residual_oil_temp_c=res_oil_temp
        )
