"""
Physics-Informed Neural Network (PINN) Residual Observer Architecture
Embeds mass conservation, energy balance, and monotonicity loss terms.
"""
import torch
import torch.nn as nn
import torch.nn.functional as F

class PhysicsInformedResidualObserver(nn.Module):
    def __init__(self, input_dim: int = 16, hidden_dim: int = 64, output_dim: int = 6):
        super().__init__()
        # Input features:
        # [rpm, map_kpa, throttle_pct, cht1..4, egt1..4, oil_press, oil_temp, fuel_flow, res_map, res_oil]
        self.fc1 = nn.Linear(input_dim, hidden_dim)
        self.ln1 = nn.LayerNorm(hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.ln2 = nn.LayerNorm(hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, hidden_dim // 2)

        # Heads:
        # 1. State correction head (CHT/EGT/MAP corrections)
        self.correction_head = nn.Linear(hidden_dim // 2, 4)
        # 2. Latent wear / anomaly score head (0.0 to 1.0)
        self.anomaly_head = nn.Linear(hidden_dim // 2, 1)
        # 3. Virtual latent efficiency factor
        self.efficiency_head = nn.Linear(hidden_dim // 2, 1)

    def forward(self, x: torch.Tensor):
        h = F.gelu(self.ln1(self.fc1(x)))
        h = F.gelu(self.ln2(self.fc2(h))) + h # Residual skip connection
        feat = F.gelu(self.fc3(h))

        corrections = self.correction_head(feat)
        anomaly_score = torch.sigmoid(self.anomaly_head(feat))
        latent_eff = torch.sigmoid(self.efficiency_head(feat)) * 0.15 + 0.28 # 28% - 43%

        return corrections, anomaly_score, latent_eff

class PhysicsInformedLoss(nn.Module):
    """
    L_total = L_data + lambda_mass * L_mass + lambda_energy * L_energy + lambda_mono * L_mono
    """
    def __init__(self, lambda_mass: float = 0.05, lambda_energy: float = 0.05, lambda_mono: float = 0.02):
        super().__init__()
        self.lambda_mass = lambda_mass
        self.lambda_energy = lambda_energy
        self.lambda_mono = lambda_mono
        self.mse = nn.MSELoss()

    def forward(self, corrections, anomaly_score, latent_eff, target_corrections, raw_inputs):
        # 1. Data Loss
        l_data = self.mse(corrections, target_corrections)

        # 2. Mass Conservation Loss: Plenum air balance
        # raw_inputs[:, 1] = map_kpa, raw_inputs[:, 2] = throttle_pct, raw_inputs[:, 0] = rpm
        map_kpa = torch.clamp(raw_inputs[:, 1], min=0.0)
        rpm = torch.clamp(raw_inputs[:, 0], min=0.0)
        fuel_flow = torch.clamp(raw_inputs[:, 11], min=0.0) # L/h

        # Estimated cylinder air induction vs fuel flow stoichiometry
        expected_air = (0.88 * 0.001211 * (rpm / 60.0) * (map_kpa * 1000.0)) / (2.0 * 287.05 * 310.0) # kg/s
        fuel_kg_s = (fuel_flow * 0.745) / 3600.0
        stoich_air = fuel_kg_s * 14.7
        # Fix: relu applied BEFORE squaring to penalize unphysical over-fueling (phi > 1.35)
        l_mass = torch.mean(torch.relu(stoich_air - expected_air * 1.35)**2)

        # 3. Energy Balance Loss (First Law of Thermodynamics)
        # Fuel power input = Shaft work + Head heat dissipation + Exhaust enthalpy
        fuel_power_kw = fuel_kg_s * 44000.0 # 44 MJ/kg
        eff = latent_eff.view(-1)
        shaft_power_kw = fuel_power_kw * eff
        # Non-negative power & thermodynamic efficiency consistency
        l_energy_over = torch.relu(shaft_power_kw - fuel_power_kw * 0.45)
        l_energy_under = torch.relu(fuel_power_kw * 0.20 - shaft_power_kw) * (fuel_power_kw > 1.0).float()
        l_energy = torch.mean(l_energy_over**2 + l_energy_under**2)

        # 4. Wear Monotonicity Loss: time derivative of anomaly score should be non-negative
        anom = anomaly_score.view(-1)
        if anom.shape[0] > 1:
            diff = anom[1:] - anom[:-1]
            l_mono = torch.mean(torch.relu(-diff)**2) # Penalize decreasing degradation
        else:
            l_mono = torch.tensor(0.0, device=anomaly_score.device)

        l_total = l_data + self.lambda_mass * l_mass + self.lambda_energy * l_energy + self.lambda_mono * l_mono
        return l_total, l_data, l_mass, l_energy, l_mono
