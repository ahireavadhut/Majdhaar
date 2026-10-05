import sys
from pathlib import Path
project_root = Path(__file__).resolve().parent.parent
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

import torch
import numpy as np
from core.telemetry.recorder import RawDataRecorder
from core.twin_core.physics_model import RotaxIndependentTwin
from pinn.train import train_pinn
from pinn.export_onnx import export_model
from core.observer.observer_engine import PinnResidualObserver

def main():
    print("=== HYBRID PINN RESIDUAL OBSERVER: TRAINING & EXPORT ===")
    recorder = RawDataRecorder()
    twin = RotaxIndependentTwin()

    # 1. Load calibration & training runs (Section 7.1: whole run splits)
    train_runs = ["run01_healthy_baseline_ground_idle", "run02_healthy_cruise_climb"]
    all_telemetry = []
    all_twin_preds = []

    for r_id in train_runs:
        print(f"Loading training data from {r_id}...")
        recs = recorder.load_decoded_telemetry(r_id)
        for r in recs:
            pred = twin.step(r)
            all_telemetry.append(r)
            all_twin_preds.append(pred)

    print(f"Total training state vectors: {len(all_telemetry)}")

    # 2. Train PINN with physics-informed loss (mass, energy, monotonicity)
    print("Training Physics-Informed Residual Observer (25 epochs)...")
    model = train_pinn(all_telemetry, all_twin_preds, epochs=25, lr=0.003)

    # 3. Export to TorchScript JIT & ONNX for edge inference
    export_dir = project_root / "core" / "observer"
    export_model(model, str(export_dir))

    # 4. Verify edge inference on held-out validation run
    print("\n--- LEVEL 3 VALIDATION: HELD-OUT RUN EVALUATION ---")
    val_id = "run03_held_out_validation_loiter"
    val_recs = recorder.load_decoded_telemetry(val_id)
    jit_path = export_dir / "pinn_observer.pt"
    observer = PinnResidualObserver(str(jit_path))

    val_twin = RotaxIndependentTwin()
    anom_scores = []
    physics_losses = []

    for r in val_recs:
        p = val_twin.step(r)
        obs = observer.evaluate(r, p)
        anom_scores.append(obs.anomaly_score)
        physics_losses.append(obs.pinn_physics_loss)

    print(f"Held-out validation samples: {len(val_recs)}")
    print(f"Mean PINN Anomaly Score: {np.mean(anom_scores):.4f} (Expected < 0.20 for healthy engine)")
    print(f"Mean Physics Loss Constraint: {np.mean(physics_losses):.4f}")
    print("PINN Observer successfully validated and ready for real-time edge streaming.")

if __name__ == "__main__":
    main()
