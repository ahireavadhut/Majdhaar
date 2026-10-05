"""
End-to-End Digital Twin Pipeline Engine
Orchestrates:
Physical/Replay CAN -> Normalizer -> Fault Injector -> Independent Twin ->
Virtual Sensors -> PINN Residual Observer -> Diagnostics/ATA-100 ->
Wiener Process RUL -> MAVLink EFI -> Full GCS Telemetry Packet
"""
from typing import Optional, Dict, Any
from core.telemetry.decoder import CanTelemetryDecoder
from core.telemetry.normalizer import TelemetryNormalizer
from core.telemetry.fault_injector import ControlledFaultInjector
from core.twin_core.physics_model import RotaxIndependentTwin
from core.virtual_sensors.estimators import VirtualSensors
from core.observer.observer_engine import PinnResidualObserver
from core.diagnostics.fault_classifier import FaultDiagnosticsEngine
from core.prognostics.wiener_rul import WienerPrognosticsEngine
from core.mavlink_bridge.efi_status import MavlinkEfiBridge
from core.telemetry_schema.schemas import (
    NormalizedEngineTelemetry,
    FullGcsTelemetryPacket,
    CanRawFrame
)

class DigitalTwinPipeline:
    def __init__(
        self,
        engine_config: str = "configs/engine.yaml",
        pgn_map: str = "configs/pgn_map.yaml",
        run_id: str = "LIVE",
        stream_state: str = "LIVE"
    ):
        self.decoder = CanTelemetryDecoder(pgn_map)
        self.normalizer = TelemetryNormalizer(run_id=run_id, stream_state=stream_state)
        self.fault_injector = ControlledFaultInjector()
        self.twin = RotaxIndependentTwin(engine_config)
        self.virtual_sensors = VirtualSensors()
        self.observer = PinnResidualObserver()
        self.diagnostics = FaultDiagnosticsEngine()
        self.prognostics = WienerPrognosticsEngine()
        self.mavlink_bridge = MavlinkEfiBridge()
        self.packet_counter = 0

    def process_can_frame(self, frame: CanRawFrame) -> Optional[FullGcsTelemetryPacket]:
        """Decodes raw CAN frame and, if a state packet is ready, runs full pipeline."""
        decoded = self.decoder.decode_frame(frame)
        if not decoded:
            return None
        telemetry = self.normalizer.update_from_decoded(decoded, frame.utc_timestamp)
        if telemetry is not None:
            return self.process_telemetry(telemetry)
        return None

    def process_telemetry(self, raw_telemetry: NormalizedEngineTelemetry) -> FullGcsTelemetryPacket:
        """
        Executes the complete Digital Twin & Prognostics chain.
        """
        self.packet_counter += 1

        # 1. Controlled test fault injection (if armed)
        telemetry = self.fault_injector.apply(raw_telemetry)

        # 2. Independent Physics Twin (0D/1D MVEM + CHT network)
        twin_pred = self.twin.step(telemetry)

        # 3. Virtual Sensors (IMEP, Blowby, Oil Film Thickness)
        virt_sensors = self.virtual_sensors.estimate(telemetry)

        # 4. Hybrid PINN Residual Observer (TorchScript JIT Edge Inference)
        obs_inference = self.observer.evaluate(telemetry, twin_pred)

        # 5. Fault Diagnostics & ATA-100 Isolation
        diagnostic = self.diagnostics.diagnose(telemetry, twin_pred, virt_sensors, obs_inference)

        # 6. Wiener Process RUL Prognostics (Inverse-Gaussian FHT Quantiles)
        prognostics = self.prognostics.predict_rul(telemetry, twin_pred, virt_sensors, obs_inference)

        # 7. Package full GCS packet
        packet = FullGcsTelemetryPacket(
            timestamp=telemetry.timestamp,
            packet_id=self.packet_counter,
            telemetry=telemetry,
            twin=twin_pred,
            virtual_sensors=virt_sensors,
            observer=obs_inference,
            diagnostic=diagnostic,
            prognostics=prognostics
        )

        return packet

    def generate_mavlink_packet(self, packet: FullGcsTelemetryPacket) -> bytes:
        """Encodes current pipeline state into MAVLink EFI_STATUS bytes."""
        return self.mavlink_bridge.encode_efi_status(packet.telemetry, packet.diagnostic)
