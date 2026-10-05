import sys
from pathlib import Path
project_root = Path(__file__).resolve().parent.parent
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from core.telemetry.recorder import RawDataRecorder
from core.twin_core.calibrator import BaselineCalibrator

def run_calibration():
    recorder = RawDataRecorder()
    calibrator = BaselineCalibrator()

    run_id = "run02_healthy_cruise_climb"
    print(f"Loading {run_id} for healthy-baseline calibration...")
    records = recorder.load_decoded_telemetry(run_id)

    report = calibrator.calibrate(run_id, records)
    print("\n--- BASELINE CALIBRATION REPORT ---")
    print(f"Run ID: {report['calibration_run_id']} (Samples: {report['sample_count']})")
    print("Calibrated Parameters:")
    for k, v in report['calibrated_parameters'].items():
        print(f"  {k}: {v}")
    print("\nResidual Performance Metrics (Level 2 Baseline):")
    for sig, met in report['residual_statistics'].items():
        print(f"  {sig:18s} | MAE: {met['mae']:7.3f} | RMSE: {met['rmse']:7.3f} | Bias: {met['bias']:7.3f}")
    print(f"\nAudit Verification Status: {report['audit_verification']}")

if __name__ == "__main__":
    run_calibration()
