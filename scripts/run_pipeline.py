"""
Primary Execution Entry Point
Aero Piston Engine Digital Twin (SIH 26054) - DRDO TAPAS-BH-201 MALE UAV
"""
import sys
import argparse
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

def main():
    parser = argparse.ArgumentParser(description="SIH 26054 Aero Piston Engine Digital Twin Pipeline")
    parser.add_argument("--mode", choices=["server", "verify", "calibrate", "replay"], default="server", help="Execution mode")
    parser.add_argument("--port", type=int, default=8000, help="GCS API HTTP/WebSocket port")
    parser.add_argument("--host", type=str, default="127.0.0.1", help="GCS Host bind address")
    parser.add_argument("--run", type=str, default="run02_healthy_cruise_climb", help="Canonical run ID for replay")
    args = parser.parse_args()

    if args.mode == "server":
        import uvicorn
        print("================================================================================")
        print("     AERO PISTON ENGINE DIGITAL TWIN (SIH 26054) - GCS CONSOLE ONLINE           ")
        print(f"     Target Platform: DRDO TAPAS-BH-201 MALE UAV (Rotax 914/915 iS Boxer)      ")
        print(f"     Serving GCS Dashboard at: http://{args.host}:{args.port}/                 ")
        print(f"     WebSocket Downlink at: ws://{args.host}:{args.port}/ws/telemetry           ")
        print("================================================================================")
        uvicorn.run("gcs_api.server:app", host=args.host, port=args.port, log_level="info")

    elif args.mode == "verify":
        from scripts.verify_all import verify_system
        verify_system()

    elif args.mode == "calibrate":
        from scripts.calibrate_baseline import run_calibration
        run_calibration()

    elif args.mode == "replay":
        from core.pipeline import DigitalTwinPipeline
        from core.telemetry.recorder import RawDataRecorder
        recorder = RawDataRecorder()
        pipeline = DigitalTwinPipeline(run_id=args.run, stream_state="REPLAY")
        records = recorder.load_decoded_telemetry(args.run)
        print(f"Replaying {len(records)} records from {args.run}...")
        for i, r in enumerate(records):
            pkt = pipeline.process_telemetry(r)
            if i % 25 == 0:
                print(f"[{i:04d}] RPM: {pkt.telemetry.rpm:.0f} | MAP: {pkt.telemetry.map_kpa:.1f} kPa | CHT1: {pkt.telemetry.cht_cyl1_c:.1f}°C | IMEP: {pkt.virtual_sensors.imep_mean_bar:.1f} bar | RUL P50: {pkt.prognostics.rul_hours_p50:.1f}h | Health: {pkt.prognostics.health_index*100:.1f}% | Status: {pkt.diagnostic.fault_type}")

if __name__ == "__main__":
    main()
