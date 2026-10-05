"""
Ground Control Station (GCS) Telemetry Backend & WebSocket Streamer
Implements FastAPI REST API and high-rate WebSocket telemetry downlink.
"""
import sys
import asyncio
import json
from pathlib import Path
from typing import Dict, Any, Optional

project_root = Path(__file__).resolve().parent.parent
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import JSONResponse, FileResponse
from pydantic import BaseModel

from core.pipeline import DigitalTwinPipeline
from core.telemetry.recorder import RawDataRecorder
from core.telemetry_schema.schemas import FullGcsTelemetryPacket

app = FastAPI(
    title="Aero Piston Engine Digital Twin (SIH 26054) - GCS API",
    description="Real-telemetry Digital Twin & Prognostics Engine for Rotax 914/915 iS (DRDO TAPAS-BH-201 UAV)",
    version="2.0.0"
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"]
)

# Global State
recorder = RawDataRecorder()
active_run_id = "run02_healthy_cruise_climb"
pipeline = DigitalTwinPipeline(run_id=active_run_id, stream_state="REPLAY")
active_telemetry_cache = recorder.load_decoded_telemetry(active_run_id)
current_index = 0
streaming_task: Optional[asyncio.Task] = None
connected_websockets = set()
playback_speed = 1.0
flight_envelope = {
    "throttle_override": None,
    "altitude_m": 0.0,
    "ambient_temp_c": 15.0
}

class FaultInjectionRequest(BaseModel):
    fault_type: str
    severity_pct: float = 100.0

class FlightEnvelopeRequest(BaseModel):
    throttle_pct: Optional[float] = None
    altitude_m: Optional[float] = 0.0
    ambient_temp_c: Optional[float] = 15.0

class RunSelectionRequest(BaseModel):
    run_id: str

@app.get("/api/status")
async def get_status():
    meta = recorder.get_metadata(active_run_id) or {}
    return {
        "active_run_id": active_run_id,
        "stream_state": pipeline.normalizer.stream_state,
        "platform": "DRDO TAPAS-BH-201 MALE UAV",
        "engine": "Rotax 914 F / Rotax 915 iS Turbocharged Boxer",
        "active_fault": pipeline.fault_injector.active_fault,
        "fault_severity": pipeline.fault_injector.severity_pct,
        "envelope": flight_envelope,
        "metadata": meta
    }

@app.get("/api/runs")
async def list_runs():
    meta_dir = Path("data/metadata")
    runs = []
    if meta_dir.exists():
        for f in meta_dir.glob("*.yaml"):
            r_id = f.stem
            m = recorder.get_metadata(r_id)
            if m:
                runs.append({
                    "run_id": r_id,
                    "dataset_type": m.get("dataset_type", "UNKNOWN"),
                    "data_lineage_label": m.get("data_lineage_label", "PHYSICAL"),
                    "operating_profile": m.get("operating_profile", ""),
                    "total_can_frames": m.get("total_can_frames", 0),
                    "duration_seconds": m.get("duration_seconds", 0)
                })
    return sorted(runs, key=lambda x: x["run_id"])

@app.post("/api/runs/select")
async def select_run(req: RunSelectionRequest):
    global active_run_id, active_telemetry_cache, current_index, pipeline
    try:
        data = recorder.load_decoded_telemetry(req.run_id)
        active_run_id = req.run_id
        active_telemetry_cache = data
        current_index = 0
        pipeline = DigitalTwinPipeline(run_id=active_run_id, stream_state="REPLAY")
        return {"status": "SUCCESS", "active_run_id": active_run_id, "samples": len(data)}
    except FileNotFoundError:
        raise HTTPException(status_code=404, detail="Run not found")

@app.post("/api/fault_injection")
async def inject_fault(req: FaultInjectionRequest):
    if req.fault_type == "NONE":
        pipeline.fault_injector.clear_fault()
    else:
        pipeline.fault_injector.set_fault(req.fault_type, req.severity_pct)
    return {
        "status": "UPDATED",
        "active_fault": pipeline.fault_injector.active_fault,
        "severity_pct": pipeline.fault_injector.severity_pct,
        "lineage_label": "CONTROLLED_FAULT_INJECTION" if req.fault_type != "NONE" else "PHYSICAL_ENGINE_MEASUREMENT"
    }

@app.post("/api/flight_envelope")
async def set_envelope(req: FlightEnvelopeRequest):
    if req.throttle_pct is not None:
        flight_envelope["throttle_override"] = max(0.0, min(100.0, req.throttle_pct))
    if req.altitude_m is not None:
        flight_envelope["altitude_m"] = max(0.0, min(9000.0, req.altitude_m))
    if req.ambient_temp_c is not None:
        flight_envelope["ambient_temp_c"] = req.ambient_temp_c
    return {"status": "SUCCESS", "flight_envelope": flight_envelope}

@app.get("/api/verification")
async def get_verification_report():
    path = Path("data/reports/system_verification_report.json")
    if not path.exists():
        raise HTTPException(status_code=404, detail="Report not generated")
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)

@app.get("/api/calibration")
async def get_calibration_report():
    path = Path("data/reports/calibration_report_run02_healthy_cruise_climb.json")
    if not path.exists():
        raise HTTPException(status_code=404, detail="Calibration report not found")
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)

@app.get("/api/compliance")
async def get_compliance_docs():
    compliance_dir = Path("compliance")
    docs = {}
    if compliance_dir.exists():
        for f in compliance_dir.glob("*.md"):
            with open(f, "r", encoding="utf-8") as file:
                docs[f.stem] = file.read()
    return docs

@app.get("/api/mavlink/sample")
async def get_mavlink_sample():
    if not active_telemetry_cache:
        raise HTTPException(status_code=404, detail="No telemetry available")
    pkt = pipeline.process_telemetry(active_telemetry_cache[0])
    mav_bytes = pipeline.generate_mavlink_packet(pkt)
    decoded = pipeline.mavlink_bridge.decode_efi_status(mav_bytes)
    return {
        "raw_packet_hex": mav_bytes.hex(),
        "bytes_count": len(mav_bytes),
        "protocol": "MAVLink v2 EFI_STATUS (Msg ID 225)",
        "decoded": decoded
    }

async def stream_engine_telemetry():
    """Background broadcast loop pushing telemetry at 20 Hz."""
    global current_index
    while True:
        if connected_websockets and active_telemetry_cache:
            raw_rec = active_telemetry_cache[current_index % len(active_telemetry_cache)]
            rec = raw_rec.model_copy(deep=True)

            # Apply flight envelope modifications
            if flight_envelope["throttle_override"] is not None:
                rec.throttle_pct = flight_envelope["throttle_override"]
            rec.altitude_m = flight_envelope["altitude_m"]
            rec.ambient_temp_c = flight_envelope["ambient_temp_c"]

            # Altitude lapse on ambient pressure: P(h) = P0 * (1 - 2.25577e-5 * h)^5.25588
            h = flight_envelope["altitude_m"]
            p_amb = 101.325 * ((1.0 - 2.25577e-5 * h) ** 5.25588)
            rec.ambient_pressure_kpa = max(25.0, round(p_amb, 2))

            # Run through digital twin pipeline
            full_packet = pipeline.process_telemetry(rec)
            packet_json = full_packet.model_dump_json()

            # Broadcast to all connected clients
            dead_sockets = set()
            for ws in connected_websockets:
                try:
                    await ws.send_text(packet_json)
                except Exception:
                    dead_sockets.add(ws)
            connected_websockets.difference_update(dead_sockets)

            current_index += 1

        await asyncio.sleep(0.05) # 20 Hz push rate for GCS UI

@app.on_event("startup")
async def startup_event():
    global streaming_task
    streaming_task = asyncio.create_task(stream_engine_telemetry())

@app.websocket("/ws/telemetry")
async def websocket_telemetry(websocket: WebSocket):
    await websocket.accept()
    connected_websockets.add(websocket)
    try:
        while True:
            # Handle incoming commands from UI
            msg_text = await websocket.receive_text()
            try:
                msg = json.loads(msg_text)
                cmd = msg.get("command")
                if cmd == "SET_FAULT":
                    pipeline.fault_injector.set_fault(msg.get("fault_type", "NONE"), msg.get("severity", 100.0))
                elif cmd == "CLEAR_FAULT":
                    pipeline.fault_injector.clear_fault()
                elif cmd == "SET_ENVELOPE":
                    flight_envelope["throttle_override"] = msg.get("throttle_pct")
                    flight_envelope["altitude_m"] = msg.get("altitude_m", 0.0)
            except Exception:
                pass
    except WebSocketDisconnect:
        connected_websockets.discard(websocket)

# Mount static frontend directory and serve index.html at root
frontend_dir = Path("gcs_frontend")
if frontend_dir.exists():
    app.mount("/static", StaticFiles(directory=str(frontend_dir)), name="static")

@app.get("/")
async def get_index():
    index_file = frontend_dir / "index.html"
    if index_file.exists():
        return FileResponse(str(index_file))
    return JSONResponse({"status": "API_ONLINE", "message": "GCS UI files not found"})
