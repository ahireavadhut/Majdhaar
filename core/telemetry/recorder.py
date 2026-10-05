"""
Immutable Raw Data Recorder & Run Metadata Manager
Adheres to Section 4: Immutable raw-data recorder.
"""
import re
import yaml
import json
from pathlib import Path
from typing import Generator, List, Dict, Any, Optional
from core.telemetry_schema.schemas import CanRawFrame, NormalizedEngineTelemetry

import time

CANDUMP_LINE_PATTERN = re.compile(
    r"^\s*(?:\(\s*(?P<ts>\d+(?:\.\d+)?)\s*\))?\s*(?P<iface>[\w\-]+)\s+(?:0[xX])?(?P<id>[0-9A-Fa-f]{1,8})#(?P<data>[0-9A-Fa-f\s]*)\s*$"
)

class RawDataRecorder:
    def __init__(self, raw_dir: str = "data/raw", meta_dir: str = "data/metadata", decoded_dir: str = "data/decoded"):
        self.raw_dir = Path(raw_dir)
        self.meta_dir = Path(meta_dir)
        self.decoded_dir = Path(decoded_dir)
        self.raw_dir.mkdir(parents=True, exist_ok=True)
        self.meta_dir.mkdir(parents=True, exist_ok=True)
        self.decoded_dir.mkdir(parents=True, exist_ok=True)

    def write_metadata(self, run_id: str, metadata: Dict[str, Any]):
        """Writes audit metadata for a given test run."""
        path = self.meta_dir / f"{run_id}.yaml"
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(metadata, f, sort_keys=False)

    def get_metadata(self, run_id: str) -> Optional[Dict[str, Any]]:
        path = self.meta_dir / f"{run_id}.yaml"
        if not path.exists():
            return None
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f)

    def record_raw_frames(self, run_id: str, frames: List[CanRawFrame]):
        """Records raw CAN frames in standard Linux candump -L log format."""
        path = self.raw_dir / f"{run_id}_can.log"
        with open(path, "w", encoding="utf-8") as f:
            for frame in frames:
                # Format: (1725350000.123456) can0 18FEE000#0102030405060708
                f.write(f"({frame.utc_timestamp:.6f}) {frame.interface} {frame.can_id:08X}#{frame.payload_hex}\n")

    def parse_candump_line(self, line: str, seq_num: int = 0) -> Optional[CanRawFrame]:
        """
        Parses a single candump line robustly without throwing unhandled exceptions.
        Handles:
          - Variable whitespace, tabs, leading and trailing padding
          - Lowercase and uppercase hex arbitration IDs and payloads
          - Optional '0x' prefix on CAN ID
          - Optional or variable precision timestamps
          - Embedded spaces between data bytes
          - DLC=0 empty payloads ('...#')
          - Corrupted or invalid lines (safely returns None)
        """
        if not line or not isinstance(line, str):
            return None

        line = line.strip()
        if not line or line.startswith("#"):
            return None

        match = CANDUMP_LINE_PATTERN.match(line)
        if not match:
            return None

        try:
            ts_str = match.group("ts")
            ts = float(ts_str) if ts_str is not None else time.time()
            iface = match.group("iface")
            can_id = int(match.group("id"), 16)
            raw_data = match.group("data") or ""
            clean_data = raw_data.replace(" ", "").strip()

            # Hex string length must be even (full bytes)
            if len(clean_data) % 2 != 0:
                return None

            # Verify byte conversion without crashing
            if clean_data:
                bytes.fromhex(clean_data)

            dlc = len(clean_data) // 2

            return CanRawFrame(
                utc_timestamp=ts,
                interface=iface,
                can_id=can_id,
                dlc=dlc,
                payload_hex=clean_data.lower(),
                seq_num=seq_num
            )
        except Exception:
            return None

    def read_raw_frames(self, run_id: str) -> Generator[CanRawFrame, None, None]:
        """Reads raw frames from an immutable candump log."""
        path = self.raw_dir / f"{run_id}_can.log"
        if not path.exists():
            raise FileNotFoundError(f"Raw CAN log not found: {path}")

        seq = 0
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            for line in f:
                frame = self.parse_candump_line(line, seq_num=seq)
                if frame is not None:
                    yield frame
                    seq += 1

    def save_decoded_telemetry(self, run_id: str, telemetry_records: List[NormalizedEngineTelemetry]):
        """Saves decoded, normalized engineering telemetry to data/decoded/."""
        path = self.decoded_dir / f"{run_id}.json"
        data = [rec.model_dump() for rec in telemetry_records]
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

    def load_decoded_telemetry(self, run_id: str) -> List[NormalizedEngineTelemetry]:
        path = self.decoded_dir / f"{run_id}.json"
        if not path.exists():
            raise FileNotFoundError(f"Decoded run not found: {path}")
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return [NormalizedEngineTelemetry(**item) for item in data]
