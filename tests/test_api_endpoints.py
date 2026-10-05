import pytest
from fastapi.testclient import TestClient
from gcs_api.server import app

@pytest.fixture
def client():
    return TestClient(app)

def test_api_status(client):
    res = client.get("/api/status")
    assert res.status_code == 200
    data = res.json()
    assert "platform" in data
    assert "DRDO TAPAS" in data["platform"]
    assert "engine" in data

def test_api_runs(client):
    res = client.get("/api/runs")
    assert res.status_code == 200
    runs = res.json()
    assert len(runs) >= 9
    run_ids = [r["run_id"] for r in runs]
    assert "run02_healthy_cruise_climb" in run_ids
    assert "run04_fault_injection_injector_coking" in run_ids

def test_api_verification(client):
    res = client.get("/api/verification")
    assert res.status_code == 200
    data = res.json()
    assert data["level1_signal_integrity"]["status"] == "PASS"
    assert data["level2_twin_baseline"]["status"] == "PASS"
    assert data["level5_virtual_sensors"]["status"] == "PASS"

def test_api_mavlink_sample(client):
    res = client.get("/api/mavlink/sample")
    assert res.status_code == 200
    data = res.json()
    assert "decoded" in data
    assert "rpm" in data["decoded"]
    assert data["bytes_count"] in (85, 77)

def test_frontend_root_index(client):
    res = client.get("/")
    assert res.status_code == 200
    assert "DRDO TAPAS-BH-201" in res.text
    # Verify new time-series rolling charts and High-DPI canvas elements
    assert "thermal-trend-canvas" in res.text
    assert "rul-fan-canvas" in res.text
    assert "boxer-canvas" in res.text
    # Verify operator enhancements & resilience elements
    assert "conn-alert-banner" in res.text
    assert "btn-reconnect-now" in res.text
    assert "gcs-toast-container" in res.text
    assert "hotkey-badge" in res.text

def test_frontend_static_assets(client):
    res_js = client.get("/static/app.js")
    assert res_js.status_code == 200
    assert "setupHighDpiCanvas" in res_js.text
    assert "renderThermalTrendChart" in res_js.text
    assert "renderRulFanChart" in res_js.text
    assert "card-critical-pulse" in res_js.text

    res_css = client.get("/static/style.css")
    assert res_css.status_code == 200
    assert "card-caution-pulse" in res_css.text
    assert "card-critical-pulse" in res_css.text
    assert "hotkey-badge" in res_css.text
    assert "gcs-toast" in res_css.text

def test_api_compliance(client):
    res = client.get("/api/compliance")
    assert res.status_code == 200
    docs = res.json()
    assert "traceability_matrix" in docs
    assert "SIH-REQ-001" in docs["traceability_matrix"]

