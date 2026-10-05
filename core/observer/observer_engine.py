"""
Residual Observer Engine (Edge Inference via TorchScript JIT / ONNX)
Computes real-time latent corrections and physics consistency metrics.
"""
import numpy as np
import torch
from pathlib import Path
from typing import Optional
from core.telemetry_schema.schemas import NormalizedEngineTelemetry, TwinPhysicsPrediction, ObserverInference

class PinnResidualObserver:
    def __init__(self, model_path: str = "core/observer/pinn_observer.pt"):
        self.model_path = Path(model_path)
        self.jit_model = None
        self._init_model()

    def _init_model(self):
        if self.model_path.exists():
            try:
                self.jit_model = torch.jit.load(str(self.model_path), map_location="cpu")
                self.jit_model.eval()
            except Exception as e:
                print(f"Warning: Could not load JIT model: {e}")

    def evaluate(self, telemetry: NormalizedEngineTelemetry, twin: TwinPhysicsPrediction) -> ObserverInference:
        """
        Executes residual observer inference.
        """
        feats_list = [
            telemetry.rpm / 6000.0,
            telemetry.map_kpa / 150.0,
            telemetry.throttle_pct / 100.0,
            telemetry.cht_cyl1_c / 150.0,
            telemetry.cht_cyl2_c / 150.0,
            telemetry.cht_cyl3_c / 150.0,
            telemetry.cht_cyl4_c / 150.0,
            telemetry.egt_cyl1_c / 1000.0,
            telemetry.egt_cyl2_c / 1000.0,
            telemetry.egt_cyl3_c / 1000.0,
            telemetry.egt_cyl4_c / 1000.0,
            telemetry.fuel_flow_lph / 50.0,
            telemetry.oil_pressure_bar / 6.0,
            telemetry.oil_temp_c / 140.0,
            twin.residual_map_kpa / 20.0,
            twin.residual_oil_pressure_bar / 2.0
        ]

        # Residual norms
        thermal_norm = float(np.sqrt(np.mean(np.array(twin.residual_cht_cyl)**2) + 0.1 * np.mean(np.array(twin.residual_egt_cyl)**2)))
        lube_norm = float(abs(twin.residual_oil_pressure_bar) * 10.0 + abs(twin.residual_oil_temp_c) * 0.2)
        combustion_norm = float(abs(twin.residual_map_kpa))

        if self.jit_model is not None:
            with torch.no_grad():
                feats_tensor = torch.tensor([feats_list], dtype=torch.float32)
                corrections_t, anomaly_score_t, latent_eff_t = self.jit_model(feats_tensor)
                corrections = corrections_t[0].tolist()
                anomaly_score = float(anomaly_score_t[0][0])
                latent_eff = float(latent_eff_t[0][0])
        else:
            corrections = [
                float(twin.residual_map_kpa * 0.1),
                float(np.mean(twin.residual_cht_cyl) * 0.05),
                float(np.mean(twin.residual_egt_cyl) * 0.02),
                float(twin.residual_oil_pressure_bar * 0.2)
            ]
            anomaly_score = float(np.clip((thermal_norm / 35.0 + lube_norm / 15.0 + combustion_norm / 25.0) / 3.0, 0.0, 1.0))
            latent_eff = 0.34

        physics_loss = float(np.clip(0.01 * (thermal_norm + lube_norm + combustion_norm), 0.001, 0.5))

        return ObserverInference(
            timestamp=telemetry.timestamp,
            corrected_latent_state=[round(c, 4) for c in corrections],
            anomaly_score=round(anomaly_score, 4),
            pinn_physics_loss=round(physics_loss, 4),
            thermal_residual_norm=round(thermal_norm, 2),
            lube_residual_norm=round(lube_norm, 2),
            combustion_residual_norm=round(combustion_norm, 2)
        )
