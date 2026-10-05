"""
Model Exporter & Inference Engine Validator
Exports PyTorch PINN model to TorchScript (.pt) and ONNX for edge deployment.
"""
import torch
from pathlib import Path
from pinn.model import PhysicsInformedResidualObserver

def export_model(model: PhysicsInformedResidualObserver, export_dir: str = "core/observer"):
    out_dir = Path(export_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    model.eval()

    dummy_input = torch.randn(1, 16, dtype=torch.float32)

    # 1. Export to TorchScript JIT for high-performance deterministic edge inference
    jit_path = out_dir / "pinn_observer.pt"
    traced_model = torch.jit.trace(model, dummy_input)
    torch.jit.save(traced_model, str(jit_path))
    print(f"PINN successfully exported to TorchScript JIT: {jit_path}")

    # 2. Save PyTorch state dict
    weights_path = out_dir / "pinn_weights.pt"
    torch.save(model.state_dict(), str(weights_path))
    print(f"PINN state dict saved: {weights_path}")

    # 3. Attempt ONNX export if onnx package is available
    onnx_path = out_dir / "pinn_observer.onnx"
    try:
        import onnx
        torch.onnx.export(
            model,
            dummy_input,
            str(onnx_path),
            dynamo=False,
            input_names=["telemetry_residual_features"],
            output_names=["corrections", "anomaly_score", "latent_efficiency"]
        )
        print(f"PINN successfully exported to ONNX: {onnx_path}")
    except Exception as e:
        print(f"Note: ONNX export skipped ({e}). Using TorchScript JIT for edge inference.")

if __name__ == "__main__":
    m = PhysicsInformedResidualObserver()
    export_model(m)
