import math
import pytest
from core.telemetry.decoder import CanTelemetryDecoder
from core.telemetry.recorder import RawDataRecorder
from core.telemetry_schema.schemas import CanRawFrame

@pytest.fixture
def decoder():
    return CanTelemetryDecoder("configs/pgn_map.yaml")

def test_engine_dynamics_encoding_decoding(decoder):
    signals = {
        "engine_speed_rpm": 5200.0,
        "manifold_air_pressure_kpa": 115.5,
        "throttle_position_pct": 85.2,
        "bus_voltage_v": 28.0,
        "engine_status_flags": 3
    }
    raw_frame = decoder.encode_signals(0x18FEE000, signals)
    assert raw_frame.can_id == 0x18FEE000
    assert raw_frame.dlc == 8

    decoded = decoder.decode_frame(raw_frame)
    assert decoded["message_name"] == "ENGINE_DYNAMICS"
    dec_sigs = decoded["signals"]
    assert abs(dec_sigs["engine_speed_rpm"] - 5200.0) < 0.2
    assert abs(dec_sigs["manifold_air_pressure_kpa"] - 115.5) < 0.05
    assert abs(dec_sigs["throttle_position_pct"] - 85.2) < 0.5
    assert abs(dec_sigs["bus_voltage_v"] - 28.0) < 0.2

def test_thermal_matrix_provenance(decoder):
    signals = {
        "cht_cyl1_deg_c": 105.4,
        "cht_cyl2_deg_c": 104.2,
        "cht_cyl3_deg_c": 107.1,
        "cht_cyl4_deg_c": 103.8
    }
    raw_frame = decoder.encode_signals(0x18FEE100, signals)
    decoded = decoder.decode_frame(raw_frame)
    prov = decoded["provenance"]
    assert "cht_cyl1_deg_c" in prov
    assert prov["cht_cyl1_deg_c"].unit == "degC"
    assert prov["cht_cyl1_deg_c"].spn == 1137
    assert prov["cht_cyl1_deg_c"].frame_id_hex == "0x18FEE100"

def test_j1939_bitfield_extraction(decoder):
    """
    Exhaustively audits J1939 Priority, PGN, and Source Address extraction
    from 29-bit extended CAN arbitration IDs (SAE J1939-21 standard).
    """
    # Test Broadcast Frame (PDU2 format, PF >= 240):
    # CAN ID: 0x18FEE000
    # Binary: 0001 1000 1111 1110 1110 0000 0000 0000
    # Priority: bits 26-28 = 6 (0b110)
    # EDP: bit 25 = 0
    # DP: bit 24 = 0
    # PF: bits 16-23 = 0xFE (254 >= 240 -> PDU2)
    # PS: bits 8-15 = 0xE0 (Group Extension)
    # SA: bits 0-7 = 0x00
    # PGN: (EDP << 17) | (DP << 16) | (PF << 8) | PS = 0xFEE0 (65248)
    fields = CanTelemetryDecoder.extract_j1939_fields(0x18FEE000)
    assert fields["priority"] == 6
    assert fields["edp"] == 0
    assert fields["dp"] == 0
    assert fields["pf"] == 0xFE
    assert fields["ps"] == 0xE0
    assert fields["sa"] == 0x00
    assert fields["pgn"] == 0xFEE0
    assert fields["da"] == 0xFF

    priority, pgn, sa = CanTelemetryDecoder.extract_j1939_id(0x18FEE000)
    assert priority == 6
    assert pgn == 0xFEE0
    assert sa == 0x00

    # Test Destination-Specific Frame (PDU1 format, PF < 240):
    # Example: PGN 59904 (0xEA00) sent to Destination Address 0x05 from Source Address 0x2A with Priority 3
    # CAN ID: (3 << 26) | (0xEA << 16) | (0x05 << 8) | 0x2A = 0x0CEA052A
    pdu1_id = (3 << 26) | (0xEA << 16) | (0x05 << 8) | 0x2A
    f_pdu1 = CanTelemetryDecoder.extract_j1939_fields(pdu1_id)
    assert f_pdu1["priority"] == 3
    assert f_pdu1["pf"] == 0xEA
    assert f_pdu1["da"] == 0x05
    assert f_pdu1["sa"] == 0x2A
    assert f_pdu1["pgn"] == 0xEA00  # PGN has PS masked to 0 for PDU1

def test_decode_with_varying_source_address_and_priority(decoder):
    """
    Verifies that frames sent with different priority or source address
    are matched and decoded correctly via their 18-bit J1939 PGN.
    """
    # PGN 0xFEE0 (ENGINE_DYNAMICS) sent by ECU at Source Address 0x28 with Priority 3:
    # 0x0CFEE028
    signals = {
        "engine_speed_rpm": 4800.0,
        "manifold_air_pressure_kpa": 105.0,
        "throttle_position_pct": 75.0,
        "bus_voltage_v": 27.8,
        "engine_status_flags": 1
    }
    encoded = decoder.encode_signals(0x18FEE000, signals)
    custom_frame = CanRawFrame(
        can_id=0x0CFEE028,
        dlc=8,
        payload_hex=encoded.payload_hex
    )
    decoded = decoder.decode_frame(custom_frame)
    assert decoded["message_name"] == "ENGINE_DYNAMICS"
    assert decoded["pgn"] == 0xFEE0
    assert decoded["priority"] == 3
    assert decoded["source_address"] == 0x28
    assert abs(decoded["signals"]["engine_speed_rpm"] - 4800.0) < 0.2
    assert "0x28" in decoded["provenance"]["engine_speed_rpm"].source_ecu

def test_dlc_mismatch_and_truncated_payload(decoder):
    """
    Verifies robust handling when frame DLC != 8 or payload is truncated,
    without raising unhandled struct.error exceptions.
    """
    # Truncated payload: only 2 bytes instead of 8 bytes
    truncated_frame = CanRawFrame(
        can_id=0x18FEE000,
        dlc=2,
        payload_hex="A280"  # Only 2 bytes (RPM only)
    )
    decoded = decoder.decode_frame(truncated_frame)
    assert decoded["message_name"] == "ENGINE_DYNAMICS"
    # First signal (engine_speed_rpm) decoded
    assert decoded["signals"]["engine_speed_rpm"] is not None
    # Subsequent signals truncated safely to None without throwing struct.error
    assert decoded["signals"]["manifold_air_pressure_kpa"] is None
    assert decoded["signals"]["throttle_position_pct"] is None

    # Empty payload
    empty_frame = CanRawFrame(
        can_id=0x18FEE000,
        dlc=0,
        payload_hex=""
    )
    decoded_empty = decoder.decode_frame(empty_frame)
    assert decoded_empty == {}

def test_invalid_hex_string_handling(decoder):
    """
    Verifies decoder rejects corrupted hex strings gracefully without throwing.
    """
    # Non-hex characters
    frame_bad_hex = CanRawFrame(can_id=0x18FEE000, dlc=8, payload_hex="GGHHZZ1122334455")
    assert decoder.decode_frame(frame_bad_hex) == {}

    # Odd length hex string
    frame_odd = CanRawFrame(can_id=0x18FEE000, dlc=8, payload_hex="123")
    assert decoder.decode_frame(frame_odd) == {}

    # Spaces and 0x prefix handled safely
    frame_spaces = CanRawFrame(can_id=0x18FEE000, dlc=8, payload_hex="0x 3f 2d 61 10 0c ff 01 00")
    decoded = decoder.decode_frame(frame_spaces)
    assert decoded != {}
    assert "engine_speed_rpm" in decoded["signals"]

def test_nan_and_none_signal_handling(decoder):
    """
    Verifies that NaN and None signal values are safely encoded as SAE J1939
    Signal Not Available (SNA, all bits 1) and decoded back to None without crashing.
    """
    signals_with_nan_none = {
        "engine_speed_rpm": None,
        "manifold_air_pressure_kpa": float("nan"),
        "throttle_position_pct": 50.0,
        "bus_voltage_v": 28.0,
        "engine_status_flags": 0
    }
    # Must NOT raise TypeError or ValueError: cannot convert float NaN to integer
    raw_frame = decoder.encode_signals(0x18FEE000, signals_with_nan_none)
    assert raw_frame is not None

    # Bytes 0-1 (RPM) and Bytes 2-3 (MAP) should be all 1s (0xFFFF)
    payload_bytes = bytes.fromhex(raw_frame.payload_hex)
    assert payload_bytes[0:2] == b"\xFF\xFF"  # RPM is SNA
    assert payload_bytes[2:4] == b"\xFF\xFF"  # MAP is SNA

    # Decoding frame with 0xFFFF should report signals as None (Not Available)
    decoded = decoder.decode_frame(raw_frame)
    assert decoded["signals"]["engine_speed_rpm"] is None
    assert decoded["signals"]["manifold_air_pressure_kpa"] is None
    # Other valid signals should decode normally
    assert abs(decoded["signals"]["throttle_position_pct"] - 50.0) < 0.5
    assert abs(decoded["signals"]["bus_voltage_v"] - 28.0) < 0.2

def test_recorder_candump_parsing():
    """
    Audits raw candump reader: variable whitespace, lowercase/uppercase hex,
    missing timestamps, spaces in data, and corrupted lines.
    """
    rec = RawDataRecorder()

    # Standard candump -L line
    line1 = "(1725350400.000000) can0 18FEE000#3f2d61100cff0100\n"
    f1 = rec.parse_candump_line(line1)
    assert f1 is not None
    assert f1.can_id == 0x18FEE000
    assert f1.interface == "can0"
    assert f1.payload_hex == "3f2d61100cff0100"
    assert abs(f1.utc_timestamp - 1725350400.0) < 1e-4

    # Variable whitespace, tabs, uppercase hex, spaces in payload
    line2 = "\t ( 1725350400.123 )   vcan0   0x18FEE000#3F 2D 61 10 0C FF 01 00  "
    f2 = rec.parse_candump_line(line2)
    assert f2 is not None
    assert f2.can_id == 0x18FEE000
    assert f2.interface == "vcan0"
    assert f2.payload_hex == "3f2d61100cff0100"

    # Missing trailing newline at EOF
    line3 = "(1725350400.5) can-0 123#0102"
    f3 = rec.parse_candump_line(line3)
    assert f3 is not None
    assert f3.can_id == 0x123
    assert f3.dlc == 2

    # DLC=0 empty payload
    line4 = "(1725350400.0) can0 18FEE000#"
    f4 = rec.parse_candump_line(line4)
    assert f4 is not None
    assert f4.dlc == 0
    assert f4.payload_hex == ""

    # Corrupted / invalid lines (must return None without raising unhandled exceptions)
    assert rec.parse_candump_line("# Log started on Wed Sep 3") is None
    assert rec.parse_candump_line("CORRUPTED_CAN_LINE_GARBAGE") is None
    assert rec.parse_candump_line("(1725350400.0) can0 18FEE000#ZZZZ") is None
    assert rec.parse_candump_line("(1725350400.0) can0 18FEE000#123") is None  # odd length
    assert rec.parse_candump_line("") is None
    assert rec.parse_candump_line(None) is None

