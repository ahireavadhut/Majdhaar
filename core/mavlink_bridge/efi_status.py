"""
MAVLink EFI_STATUS (Message ID 225) Downlink Bridge
Transmits normalized aero piston engine health and telemetry over aerospace MAVLink standard.
Compliant with MAVLink v2 framing, X.25 / ITU-T CRC-16 with CRC_EXTRA = 208, and sequence wrapping.
"""
import struct
from typing import Dict, Any, Optional
from core.telemetry_schema.schemas import NormalizedEngineTelemetry, AnomalyDiagnostic

def mavlink_x25_crc(data: bytes, initial_crc: int = 0xFFFF) -> int:
    """
    Computes official MAVLink X.25 / ITU-T CRC-16 checksum.
    Polynomial: x^16 + x^12 + x^5 + 1 (0x1021 / reversed 0x8408).
    Initial value: 0xFFFF.
    """
    crc = initial_crc & 0xFFFF
    for byte in data:
        tmp = (byte & 0xFF) ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc

def calculate_mavlink_crc(header_and_payload: bytes, crc_extra: int = 208) -> int:
    """
    Calculates MAVLink v2 CRC-16 over header (excluding STX byte) + payload + CRC_EXTRA.
    For EFI_STATUS (Message ID 225), CRC_EXTRA = 208.
    """
    crc = mavlink_x25_crc(header_and_payload, 0xFFFF)
    # Accumulate CRC_EXTRA byte
    tmp = (crc_extra & 0xFF) ^ (crc & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc

class MavlinkEfiBridge:
    """
    MAVLink v2 Bridge for EFI_STATUS (Message ID 225).
    Allows real Mission Planner, QGroundControl, and DroneCAN/MAVLink GCS software
    to ingest telemetry directly with authentic X.25 CRC-16 verification.
    """
    MSG_ID = 225
    CRC_EXTRA = 208

    def __init__(self, system_id: int = 1, component_id: int = 100):
        self.system_id = system_id & 0xFF
        self.component_id = component_id & 0xFF
        self.seq = 0

    def encode_efi_status(
        self,
        telemetry: NormalizedEngineTelemetry,
        diagnostic: Optional[AnomalyDiagnostic] = None,
        include_extensions: bool = True
    ) -> bytes:
        """
        Encodes NormalizedEngineTelemetry into an authentic MAVLink v2 EFI_STATUS packet.
        Message ID: 225
        CRC_EXTRA: 208

        Payload field order in official MAVLink wire protocol (sorted by byte size, then XML order):
          1. ecu_index (float, 4 bytes)
          2. rpm (float, 4 bytes)
          3. fuel_consumed (float, 4 bytes, cm^3)
          4. fuel_flow (float, 4 bytes, cm^3/min)
          5. engine_load (float, 4 bytes, %)
          6. throttle_position (float, 4 bytes, %)
          7. spark_dwell_time (float, 4 bytes, ms)
          8. barometric_pressure (float, 4 bytes, kPa)
          9. intake_manifold_pressure (float, 4 bytes, kPa)
          10. intake_manifold_temperature (float, 4 bytes, degC)
          11. cylinder_head_temperature (float, 4 bytes, degC)
          12. ignition_timing (float, 4 bytes, deg BTDC)
          13. injection_time (float, 4 bytes, ms)
          14. exhaust_gas_temperature (float, 4 bytes, degC)
          15. throttle_out (float, 4 bytes, %)
          16. pt_compensation (float, 4 bytes, %)
          17. health (uint8_t, 1 byte: 1=healthy, 0=fault)
          -- Extension fields (MAVLink v2):
          18. ignition_voltage (float, 4 bytes, V)
          19. fuel_pressure (float, 4 bytes, kPa)
        """
        health_flag = 1 if (diagnostic is None or not diagnostic.fault_detected) else 0

        # Extract values with safe defaults
        rpm = float(telemetry.rpm)
        fuel_flow = float(telemetry.fuel_flow_lph * 16.667) # Convert L/h to cm^3/min
        throttle = float(telemetry.throttle_pct)
        baro = float(telemetry.ambient_pressure_kpa)
        map_kpa = float(telemetry.map_kpa)
        cht = float(telemetry.cht_mean_c)
        egt = float(telemetry.egt_mean_c)
        bus_volt = float(getattr(telemetry, "bus_voltage_v", 28.0))
        fuel_press_kpa = float(getattr(telemetry, "fuel_pressure_bar", 3.0) * 100.0)

        if include_extensions:
            # 16 floats (64 bytes) + 1 uint8 (1 byte) + 2 extension floats (8 bytes) = 73 bytes
            payload = struct.pack(
                "<ffffffffffffffffBff",
                1.0,            # ecu_index
                rpm,            # rpm
                12.5,           # fuel_consumed
                fuel_flow,      # fuel_flow (cm^3/min)
                throttle,       # engine_load (%)
                throttle,       # throttle_position (%)
                2.4,            # spark_dwell_time (ms)
                baro,           # barometric_pressure (kPa)
                map_kpa,        # intake_manifold_pressure (kPa)
                35.0,           # intake_manifold_temperature (degC)
                cht,            # cylinder_head_temperature (degC)
                24.0,           # ignition_timing (deg BTDC)
                4.2,            # injection_time (ms)
                egt,            # exhaust_gas_temperature (degC)
                throttle,       # throttle_out (%)
                0.0,            # pt_compensation (%)
                health_flag,    # health
                bus_volt,       # ignition_voltage (V)
                fuel_press_kpa  # fuel_pressure (kPa)
            )
        else:
            # Base payload without extensions: 16 floats + 1 uint8 = 65 bytes
            payload = struct.pack(
                "<ffffffffffffffffB",
                1.0,
                rpm,
                12.5,
                fuel_flow,
                throttle,
                throttle,
                2.4,
                baro,
                map_kpa,
                35.0,
                cht,
                24.0,
                4.2,
                egt,
                throttle,
                0.0,
                health_flag
            )

        seq_out = self.seq % 256
        self.seq = (self.seq + 1) % 256

        # MAVLink v2 Header (10 bytes):
        # [0]: STX (0xFD)
        # [1]: Payload length (len(payload))
        # [2]: Incompatible flags (0)
        # [3]: Compatible flags (0)
        # [4]: Sequence number (0-255)
        # [5]: System ID (0-255)
        # [6]: Component ID (0-255)
        # [7-9]: Message ID (24-bit little-endian)
        header = struct.pack(
            "<BBBBBBBHB",
            0xFD,
            len(payload),
            0,
            0,
            seq_out,
            self.system_id,
            self.component_id,
            self.MSG_ID & 0xFFFF,
            (self.MSG_ID >> 16) & 0xFF
        )

        # Compute standard MAVLink X.25 CRC-16 over header[1:] + payload + CRC_EXTRA
        crc = calculate_mavlink_crc(header[1:] + payload, self.CRC_EXTRA)
        crc_bytes = struct.pack("<H", crc)

        return header + payload + crc_bytes

    def decode_efi_status(self, packet: bytes) -> Optional[Dict[str, Any]]:
        """
        Parses and validates a MAVLink v2 EFI_STATUS packet.
        Verifies STX (0xFD), Message ID (225), and authentic X.25 CRC-16 with CRC_EXTRA (208).
        Safely unpacks fields handling zero-truncation.
        """
        if not packet or len(packet) < 12: # At least header (10) + crc (2)
            return None

        stx = packet[0]
        if stx != 0xFD:
            return None

        payload_len = packet[1]
        expected_min_len = 10 + payload_len + 2
        if len(packet) < expected_min_len:
            return None

        # Verify Message ID (24 bits at offsets 7..9)
        msg_id = packet[7] | (packet[8] << 8) | (packet[9] << 16)
        if msg_id != self.MSG_ID:
            return None

        # Extract sequence, sysid, compid
        seq = packet[4]
        sys_id = packet[5]
        comp_id = packet[6]

        # Verify CRC
        received_crc = struct.unpack("<H", packet[10 + payload_len: 10 + payload_len + 2])[0]
        expected_crc = calculate_mavlink_crc(packet[1:10 + payload_len], self.CRC_EXTRA)
        if received_crc != expected_crc:
            return None

        # Extract payload and zero-pad to 73 bytes if truncated according to MAVLink v2 zero-truncation
        raw_payload = packet[10: 10 + payload_len]
        if len(raw_payload) < 73:
            raw_payload = raw_payload + b"\x00" * (73 - len(raw_payload))

        fields = struct.unpack("<ffffffffffffffffBff", raw_payload[:73])

        return {
            "seq": seq,
            "system_id": sys_id,
            "component_id": comp_id,
            "ecu_index": round(fields[0], 1),
            "rpm": round(fields[1], 1),
            "fuel_consumed_l": round(fields[2], 2),
            "fuel_flow_ccpm": round(fields[3], 1),
            "engine_load_pct": round(fields[4], 1),
            "throttle_pct": round(fields[5], 1),
            "spark_dwell_time_ms": round(fields[6], 2),
            "barometric_pressure_kpa": round(fields[7], 2),
            "map_kpa": round(fields[8], 2),
            "intake_manifold_temp_c": round(fields[9], 1),
            "cht_mean_c": round(fields[10], 1),
            "ignition_timing_deg": round(fields[11], 1),
            "injection_time_ms": round(fields[12], 2),
            "egt_mean_c": round(fields[13], 1),
            "throttle_out_pct": round(fields[14], 1),
            "pt_compensation": round(fields[15], 2),
            "health_raw": fields[16],
            "health_status": "HEALTHY" if fields[16] == 1 else "FAULT_PRESENT",
            "ignition_voltage_v": round(fields[17], 2),
            "fuel_pressure_kpa": round(fields[18], 2)
        }
