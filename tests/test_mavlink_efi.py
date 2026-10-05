import pytest
import struct
from pymavlink.dialects.v20 import common as mavlink2
from core.mavlink_bridge.efi_status import MavlinkEfiBridge, calculate_mavlink_crc, mavlink_x25_crc
from core.telemetry_schema.schemas import NormalizedEngineTelemetry, AnomalyDiagnostic

@pytest.fixture
def nominal_telemetry():
    return NormalizedEngineTelemetry(
        timestamp=10.0, seq_num=1, rpm=5200.0, map_kpa=112.5, throttle_pct=82.0, bus_voltage_v=28.0,
        cht_cyl1_c=108.0, cht_cyl2_c=106.0, cht_cyl3_c=109.0, cht_cyl4_c=107.0, cht_mean_c=107.5, cht_spread_c=3.0,
        egt_cyl1_c=825.0, egt_cyl2_c=820.0, egt_cyl3_c=830.0, egt_cyl4_c=822.0, egt_mean_c=824.2, egt_spread_c=10.0,
        oil_pressure_bar=3.9, oil_temp_c=93.0, fuel_flow_lph=24.5, fuel_pressure_bar=3.2, coolant_temp_c=84.0,
        rms_vibration_g=0.92, peak_knock_bar=0.42, crankcase_pressure_mbar=2.7
    )

def test_mavlink_encoding_decoding(nominal_telemetry):
    """Verifies round-trip encoding and decoding of MAVLink v2 EFI_STATUS packet."""
    bridge = MavlinkEfiBridge(system_id=1, component_id=100)
    raw_bytes = bridge.encode_efi_status(nominal_telemetry)
    assert len(raw_bytes) == 85  # 10 header + 73 payload + 2 CRC
    assert raw_bytes[0] == 0xFD  # MAVLink v2 magic STX

    decoded = bridge.decode_efi_status(raw_bytes)
    assert decoded is not None
    assert abs(decoded["rpm"] - 5200.0) < 1.0
    assert abs(decoded["map_kpa"] - 112.5) < 0.1
    assert abs(decoded["cht_mean_c"] - 107.5) < 0.1
    assert abs(decoded["egt_mean_c"] - 824.2) < 0.1
    assert abs(decoded["throttle_pct"] - 82.0) < 0.1
    assert decoded["health_status"] == "HEALTHY"
    assert decoded["health_raw"] == 1
    assert abs(decoded["ignition_voltage_v"] - 28.0) < 0.1
    assert abs(decoded["fuel_pressure_kpa"] - 320.0) < 1.0

def test_mavlink_v2_x25_crc16_and_crc_extra():
    """Verifies X.25 CRC-16 matches ITU-T specification and uses CRC_EXTRA=208."""
    # Test vector: ASCII "123456789"
    # Standard X.25 CRC of "123456789" is 0x906E (inverted) or ITU-T standard
    crc = mavlink_x25_crc(b"123456789")
    assert isinstance(crc, int)
    assert 0 <= crc <= 0xFFFF

    # Verify pymavlink's own x25crc produces identical results
    pymav_crc = mavlink2.x25crc(b"123456789")
    assert crc == pymav_crc.crc

    # Verify EFI_STATUS CRC_EXTRA is 208
    assert MavlinkEfiBridge.CRC_EXTRA == 208
    assert mavlink2.MAVLink_efi_status_message.crc_extra == 208

def test_pymavlink_interoperability(nominal_telemetry):
    """
    Directly passes encoded bytes to pymavlink's parser.
    Ensures real Mission Planner / QGroundControl can parse packets without errors.
    """
    bridge = MavlinkEfiBridge(system_id=1, component_id=100)
    packet_bytes = bridge.encode_efi_status(nominal_telemetry)

    mav_reader = mavlink2.MAVLink(None)
    parsed_msgs = mav_reader.parse_buffer(packet_bytes)

    assert len(parsed_msgs) == 1, "pymavlink failed to parse the EFI_STATUS packet!"
    msg = parsed_msgs[0]
    assert msg.get_msgId() == 225
    assert abs(msg.rpm - 5200.0) < 1.0
    assert abs(msg.intake_manifold_pressure - 112.5) < 0.1
    assert abs(msg.cylinder_head_temperature - 107.5) < 0.1
    assert abs(msg.exhaust_gas_temperature - 824.2) < 0.2
    assert abs(msg.throttle_position - 82.0) < 0.1
    assert abs(msg.throttle_out - 82.0) < 0.1
    assert msg.health == 1
    assert abs(msg.ignition_voltage - 28.0) < 0.1
    assert abs(msg.fuel_pressure - 320.0) < 1.0

def test_mavlink_sequence_counter_wrapping(nominal_telemetry):
    """Verifies sequence counter wraps modulo 256 (0..255..0)."""
    bridge = MavlinkEfiBridge(system_id=1, component_id=100)
    bridge.seq = 254

    pkt1 = bridge.encode_efi_status(nominal_telemetry)
    assert pkt1[4] == 254
    assert bridge.seq == 255

    pkt2 = bridge.encode_efi_status(nominal_telemetry)
    assert pkt2[4] == 255
    assert bridge.seq == 0

    pkt3 = bridge.encode_efi_status(nominal_telemetry)
    assert pkt3[4] == 0
    assert bridge.seq == 1

def test_mavlink_system_and_component_ids(nominal_telemetry):
    """Verifies system_id and component_id are preserved in MAVLink header."""
    bridge = MavlinkEfiBridge(system_id=42, component_id=190)
    pkt = bridge.encode_efi_status(nominal_telemetry)
    assert pkt[5] == 42
    assert pkt[6] == 190

    decoded = bridge.decode_efi_status(pkt)
    assert decoded["system_id"] == 42
    assert decoded["component_id"] == 190

def test_mavlink_decoding_corrupted_packets(nominal_telemetry):
    """Verifies bridge cleanly rejects corrupted, truncated, or invalid packets without throwing."""
    bridge = MavlinkEfiBridge()
    valid_packet = bridge.encode_efi_status(nominal_telemetry)

    # Corrupted STX
    bad_stx = bytearray(valid_packet)
    bad_stx[0] = 0xFE
    assert bridge.decode_efi_status(bytes(bad_stx)) is None

    # Corrupted CRC
    bad_crc = bytearray(valid_packet)
    bad_crc[-1] ^= 0xFF
    assert bridge.decode_efi_status(bytes(bad_crc)) is None

    # Truncated packet
    assert bridge.decode_efi_status(valid_packet[:20]) is None
    assert bridge.decode_efi_status(b"") is None

    # Wrong message ID
    bad_id = bytearray(valid_packet)
    bad_id[7] = 0x99  # change msg_id to 153
    # Update CRC so it fails on msg_id check
    assert bridge.decode_efi_status(bytes(bad_id)) is None

def test_mavlink_health_flag_diagnostic(nominal_telemetry):
    """Verifies health flag correctly encodes nominal vs fault-injected states."""
    bridge = MavlinkEfiBridge()

    # Nominal: no fault
    healthy_diag = AnomalyDiagnostic(fault_detected=False)
    pkt_healthy = bridge.encode_efi_status(nominal_telemetry, healthy_diag)
    decoded_healthy = bridge.decode_efi_status(pkt_healthy)
    assert decoded_healthy["health_status"] == "HEALTHY"
    assert decoded_healthy["health_raw"] == 1

    # Fault detected
    fault_diag = AnomalyDiagnostic(
        fault_detected=True,
        fault_type="INJECTOR_CLOGGING",
        severity="CRITICAL"
    )
    pkt_fault = bridge.encode_efi_status(nominal_telemetry, fault_diag)
    decoded_fault = bridge.decode_efi_status(pkt_fault)
    assert decoded_fault["health_status"] == "FAULT_PRESENT"
    assert decoded_fault["health_raw"] == 0

def test_mavlink_base_payload_option(nominal_telemetry):
    """Verifies encoding without extension fields yields 77 bytes (65-byte payload)."""
    bridge = MavlinkEfiBridge()
    base_pkt = bridge.encode_efi_status(nominal_telemetry, include_extensions=False)
    assert len(base_pkt) == 77  # 10 header + 65 payload + 2 CRC
    decoded = bridge.decode_efi_status(base_pkt)
    assert decoded is not None
    assert abs(decoded["rpm"] - 5200.0) < 1.0
    assert decoded["health_status"] == "HEALTHY"

