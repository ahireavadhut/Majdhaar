import pytest
from core.pipeline import DigitalTwinPipeline
from core.telemetry.recorder import RawDataRecorder

def test_pipeline_end_to_end_on_real_run():
    recorder = RawDataRecorder()
    pipeline = DigitalTwinPipeline(run_id="run02_healthy_cruise_climb", stream_state="REPLAY")

    records = recorder.load_decoded_telemetry("run02_healthy_cruise_climb")
    assert len(records) > 0

    packets = []
    for r in records[:50]:
        pkt = pipeline.process_telemetry(r)
        packets.append(pkt)

    assert len(packets) == 50
    p = packets[-1]
    assert p.telemetry.rpm > 4000.0
    assert len(p.virtual_sensors.imep_cyl_bar) == 4
    assert p.prognostics.rul_hours_p10 <= p.prognostics.rul_hours_p50
