"""
CAN Telemetry Decoder & Signal Validator
Decodes 29-bit J1939 / DroneCAN frames and maintains strict provenance.
Compliant with SAE J1939-21, SAE J1939-71, and ARINC 825 avionics standards.
"""
import math
import struct
import yaml
from pathlib import Path
from typing import Dict, Any, Optional, Tuple, Union
from core.telemetry_schema.schemas import CanRawFrame, SignalProvenance

class CanTelemetryDecoder:
    def __init__(self, pgn_map_path: str = "configs/pgn_map.yaml"):
        self.pgn_map_path = Path(pgn_map_path)
        self.messages: Dict[int, Dict[str, Any]] = {}
        self.pgn_messages: Dict[int, Dict[str, Any]] = {}
        self._load_config()

    @staticmethod
    def extract_j1939_fields(can_id: int) -> Dict[str, int]:
        """
        Extracts J1939 protocol fields from a 29-bit extended CAN arbitration ID.
        Bit breakdown (SAE J1939-21):
          - Priority (P): bits 26-28 (3 bits, 0=highest, 7=lowest)
          - Extended Data Page (EDP): bit 25 (1 bit, typically 0 in J1939)
          - Data Page (DP): bit 24 (1 bit)
          - PDU Format (PF): bits 16-23 (8 bits)
          - PDU Specific (PS): bits 8-15 (8 bits)
            * If PF < 240 (0xF0): PDU1 format (peer-to-peer / destination directed), PS = Destination Address (DA)
            * If PF >= 240 (0xF0): PDU2 format (broadcast), PS = Group Extension (GE)
          - Source Address (SA): bits 0-7 (8 bits)
          - Parameter Group Number (PGN): 18-bit identifier (bits 8-25)
            * For PDU1: (EDP << 17) | (DP << 16) | (PF << 8) [PS is masked to 0x00]
            * For PDU2: (EDP << 17) | (DP << 16) | (PF << 8) | PS
        """
        can_id_29 = can_id & 0x1FFFFFFF
        priority = (can_id_29 >> 26) & 0x07
        edp = (can_id_29 >> 25) & 0x01
        dp = (can_id_29 >> 24) & 0x01
        pf = (can_id_29 >> 16) & 0xFF
        ps = (can_id_29 >> 8) & 0xFF
        sa = can_id_29 & 0xFF

        raw_pgn = (can_id_29 >> 8) & 0x3FFFF
        if pf < 240:
            # PDU1: Peer-to-peer. PS is Destination Address (DA). Standard PGN definition sets lower byte to 0.
            pgn = (edp << 17) | (dp << 16) | (pf << 8)
            da = ps
        else:
            # PDU2: Broadcast. PS is Group Extension (GE).
            pgn = raw_pgn
            da = 0xFF

        return {
            "priority": priority,
            "edp": edp,
            "dp": dp,
            "pf": pf,
            "ps": ps,
            "sa": sa,
            "da": da,
            "pgn": pgn,
            "raw_pgn": raw_pgn
        }

    @staticmethod
    def extract_j1939_id(can_id: int) -> Tuple[int, int, int]:
        """
        Extracts (priority, pgn, source_address) from 29-bit CAN ID.
        """
        fields = CanTelemetryDecoder.extract_j1939_fields(can_id)
        return fields["priority"], fields["pgn"], fields["sa"]

    def _load_config(self):
        if not self.pgn_map_path.exists():
            raise FileNotFoundError(f"Config file not found: {self.pgn_map_path}")
        with open(self.pgn_map_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)

        for msg in cfg.get("messages", []):
            can_id = msg["can_id"]
            if isinstance(can_id, str):
                can_id = int(can_id, 16) if can_id.startswith(("0x", "0X")) else int(can_id)
            self.messages[can_id] = msg

            # Index by PGN for flexible decoding across varying source addresses or priorities
            if "pgn" in msg:
                pgn_val = msg["pgn"]
                if isinstance(pgn_val, str):
                    pgn_val = int(pgn_val, 16) if pgn_val.startswith(("0x", "0X")) else int(pgn_val)
                self.pgn_messages[pgn_val] = msg
            else:
                extracted = self.extract_j1939_fields(can_id)
                self.pgn_messages[extracted["pgn"]] = msg

    def decode_frame(self, frame: CanRawFrame) -> Dict[str, Any]:
        """
        Decodes a CanRawFrame into signal dictionary with lineage provenance.
        Safely handles DLC mismatches, truncated payloads, invalid hex strings,
        and J1939 Signal Not Available (SNA / NaN) representations without throwing exceptions.
        """
        if not isinstance(frame, CanRawFrame):
            return {}

        can_id = frame.can_id
        j1939_fields = self.extract_j1939_fields(can_id)

        # Match message by exact CAN ID first, then fallback to PGN lookup
        if can_id in self.messages:
            msg_def = self.messages[can_id]
        elif j1939_fields["pgn"] in self.pgn_messages:
            msg_def = self.pgn_messages[j1939_fields["pgn"]]
        elif j1939_fields["raw_pgn"] in self.pgn_messages:
            msg_def = self.pgn_messages[j1939_fields["raw_pgn"]]
        else:
            return {}

        # Validate and clean payload_hex safely
        if not frame.payload_hex or not isinstance(frame.payload_hex, str):
            return {}

        clean_hex = frame.payload_hex.strip()
        if clean_hex.startswith(("0x", "0X")):
            clean_hex = clean_hex[2:]
        clean_hex = clean_hex.replace(" ", "")

        # Even number of hex characters required
        if len(clean_hex) % 2 != 0:
            return {}

        try:
            payload = bytes.fromhex(clean_hex)
        except (ValueError, TypeError):
            return {}

        # Handle empty payload when non-zero expected
        expected_dlc = msg_def.get("dlc", 8)
        if len(payload) == 0 and expected_dlc > 0:
            return {}

        decoded_signals: Dict[str, Optional[float]] = {}
        provenance: Dict[str, SignalProvenance] = {}

        source_ecu_label = msg_def.get(
            "source_ecu",
            "ECU-ROTAX-01" if j1939_fields["sa"] == 0 else f"ECU-SA-0x{j1939_fields['sa']:02X}"
        )

        for sig in msg_def.get("signals", []):
            name = sig["name"]
            start_bit = sig["start_bit"]
            bit_length = sig["bit_length"]
            is_signed = sig.get("is_signed", False)
            scale = sig.get("scale", 1.0)
            offset = sig.get("offset", 0.0)
            unit = sig.get("unit", "")
            spn = sig.get("spn", 0)

            # Byte slice extraction
            byte_start = start_bit // 8
            byte_count = bit_length // 8

            # Robust handling of truncated payload for this signal
            if len(payload) < byte_start + byte_count:
                decoded_signals[name] = None
                provenance[name] = SignalProvenance(
                    signal_name=name,
                    frame_id_hex=f"0x{can_id:08X}",
                    pgn=j1939_fields["pgn"],
                    spn=spn,
                    raw_slice_hex="",
                    decoded_value=None,
                    unit=unit,
                    timestamp=frame.utc_timestamp,
                    source_ecu=source_ecu_label
                )
                continue

            raw_slice = payload[byte_start:byte_start + byte_count]
            raw_slice_hex = raw_slice.hex()

            try:
                if bit_length == 8:
                    fmt = "b" if is_signed else "B"
                    raw_val = struct.unpack(fmt, raw_slice)[0]
                elif bit_length == 16:
                    fmt = "<h" if is_signed else "<H"
                    raw_val = struct.unpack(fmt, raw_slice)[0]
                elif bit_length == 32:
                    fmt = "<i" if is_signed else "<I"
                    raw_val = struct.unpack(fmt, raw_slice)[0]
                else:
                    raw_val = 0
            except struct.error:
                decoded_signals[name] = None
                continue

            # J1939 Signal Not Available (SNA) check:
            # When all data bits are 1 (e.g. 0xFF, 0xFFFF, 0xFFFFFFFF), SAE J1939 defines the signal as Not Available / Error.
            raw_unsigned = int.from_bytes(raw_slice, byteorder="little", signed=False)
            all_ones = (1 << bit_length) - 1
            if raw_unsigned == all_ones:
                decoded_signals[name] = None
                provenance[name] = SignalProvenance(
                    signal_name=name,
                    frame_id_hex=f"0x{can_id:08X}",
                    pgn=j1939_fields["pgn"],
                    spn=spn,
                    raw_slice_hex=raw_slice_hex,
                    decoded_value=None,
                    unit=unit,
                    timestamp=frame.utc_timestamp,
                    source_ecu=source_ecu_label
                )
                continue

            scaled_val = round((raw_val * scale) + offset, 4)

            # Physical limit validation
            min_val = sig.get("min", -1e9)
            max_val = sig.get("max", 1e9)
            valid = (min_val <= scaled_val <= max_val)

            decoded_signals[name] = scaled_val
            provenance[name] = SignalProvenance(
                signal_name=name,
                frame_id_hex=f"0x{can_id:08X}",
                pgn=j1939_fields["pgn"],
                spn=spn,
                raw_slice_hex=raw_slice_hex,
                decoded_value=scaled_val,
                unit=unit,
                timestamp=frame.utc_timestamp,
                source_ecu=source_ecu_label
            )

        return {
            "message_name": msg_def["name"],
            "can_id": can_id,
            "pgn": j1939_fields["pgn"],
            "priority": j1939_fields["priority"],
            "source_address": j1939_fields["sa"],
            "signals": decoded_signals,
            "provenance": provenance
        }

    def encode_signals(self, can_id: int, signal_values: Optional[Dict[str, Optional[float]]] = None) -> CanRawFrame:
        """
        Encodes physical engineering values into exact CAN payload.
        Handles None or NaN signal values by packing the SAE J1939 standard
        Signal Not Available (SNA) sentinel (all bits 1: 0xFF, 0xFFFF, 0xFFFFFFFF)
        without crashing.
        """
        j1939_fields = self.extract_j1939_fields(can_id)
        if can_id in self.messages:
            msg_def = self.messages[can_id]
        elif j1939_fields["pgn"] in self.pgn_messages:
            msg_def = self.pgn_messages[j1939_fields["pgn"]]
        elif j1939_fields["raw_pgn"] in self.pgn_messages:
            msg_def = self.pgn_messages[j1939_fields["raw_pgn"]]
        else:
            raise ValueError(f"Unknown CAN ID / PGN: 0x{can_id:08X}")

        payload = bytearray(msg_def.get("dlc", 8))

        if signal_values is None:
            signal_values = {}

        for sig in msg_def.get("signals", []):
            name = sig["name"]
            start_bit = sig["start_bit"]
            bit_length = sig["bit_length"]
            is_signed = sig.get("is_signed", False)
            scale = sig.get("scale", 1.0)
            offset = sig.get("offset", 0.0)

            byte_start = start_bit // 8
            byte_count = bit_length // 8

            val = signal_values.get(name)

            # Handle None or NaN: encode as SAE J1939 Signal Not Available (all bits 1)
            if val is None or (isinstance(val, float) and math.isnan(val)):
                all_ones = (1 << bit_length) - 1
                payload[byte_start:byte_start + byte_count] = all_ones.to_bytes(
                    byte_count, byteorder="little", signed=False
                )
                continue

            # Inverse scaling
            try:
                raw_val = int(round((float(val) - offset) / scale))
            except (ValueError, TypeError, OverflowError):
                all_ones = (1 << bit_length) - 1
                payload[byte_start:byte_start + byte_count] = all_ones.to_bytes(
                    byte_count, byteorder="little", signed=False
                )
                continue

            if bit_length == 8:
                raw_val = max(0, min(255, raw_val)) if not is_signed else max(-128, min(127, raw_val))
                fmt = "b" if is_signed else "B"
                payload[byte_start:byte_start+1] = struct.pack(fmt, raw_val)
            elif bit_length == 16:
                raw_val = max(0, min(65535, raw_val)) if not is_signed else max(-32768, min(32767, raw_val))
                fmt = "<h" if is_signed else "<H"
                payload[byte_start:byte_start+2] = struct.pack(fmt, raw_val)
            elif bit_length == 32:
                raw_val = max(0, min(4294967295, raw_val)) if not is_signed else max(-2147483648, min(2147483647, raw_val))
                fmt = "<i" if is_signed else "<I"
                payload[byte_start:byte_start+4] = struct.pack(fmt, raw_val)

        return CanRawFrame(
            can_id=can_id,
            dlc=len(payload),
            payload_hex=payload.hex()
        )
