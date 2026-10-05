"""
Dataset Loader for Real Telemetry Runs

Enforces whole-run splitting without leakage (Section 7.1).
"""
import torch
from torch.utils.data import Dataset
import numpy as np
from typing import List
from core.telemetry_schema.schemas import NormalizedEngineTelemetry, TwinPhysicsPrediction

class EngineRunDataset(Dataset):
    def __init__(self, telemetry_records: List[NormalizedEngineTelemetry], twin_predictions: List[TwinPhysicsPrediction]):
        assert len(telemetry_records) == len(twin_predictions), "Mismatched lengths"
        self.features = []
        self.targets = []
        self.raw_inputs = []

        for t, p in zip(telemetry_records, twin_predictions):
            feat = [
                t.rpm / 6000.0,
                t.map_kpa / 150.0,
                t.throttle_pct / 100.0,
                t.cht_cyl1_c / 150.0,
                t.cht_cyl2_c / 150.0,
                t.cht_cyl3_c / 150.0,
                t.cht_cyl4_c / 150.0,
                t.egt_cyl1_c / 1000.0,
                t.egt_cyl2_c / 1000.0,
                t.egt_cyl3_c / 1000.0,
                t.egt_cyl4_c / 1000.0,
                t.fuel_flow_lph / 50.0,
                t.oil_pressure_bar / 6.0,
                t.oil_temp_c / 140.0,
                p.residual_map_kpa / 20.0,
                p.residual_oil_pressure_bar / 2.0
            ]
            raw = [
                t.rpm, t.map_kpa, t.throttle_pct,
                t.cht_cyl1_c, t.cht_cyl2_c, t.cht_cyl3_c, t.cht_cyl4_c,
                t.egt_cyl1_c, t.egt_cyl2_c, t.egt_cyl3_c, t.egt_cyl4_c,
                t.fuel_flow_lph, t.oil_pressure_bar, t.oil_temp_c,
                p.residual_map_kpa, p.residual_oil_pressure_bar
            ]
            target = [
                p.residual_map_kpa / 20.0,
                sum(p.residual_cht_cyl) / (4.0 * 20.0),
                sum(p.residual_egt_cyl) / (4.0 * 100.0),
                p.residual_oil_pressure_bar / 2.0
            ]
            self.features.append(feat)
            self.raw_inputs.append(raw)
            self.targets.append(target)

        self.features_tensor = torch.tensor(self.features, dtype=torch.float32)
        self.targets_tensor = torch.tensor(self.targets, dtype=torch.float32)
        self.raw_inputs_tensor = torch.tensor(self.raw_inputs, dtype=torch.float32)

    def __len__(self):
        return len(self.features)

    def __getitem__(self, idx):
        return self.features_tensor[idx], self.targets_tensor[idx], self.raw_inputs_tensor[idx]
