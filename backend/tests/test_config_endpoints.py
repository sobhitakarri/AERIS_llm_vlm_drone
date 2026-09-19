"""
Unit tests for AERIS dynamic runtime configuration endpoints.
"""
import pytest
from fastapi.testclient import TestClient

from backend.api.server import create_app
from backend.api.routes import app_context


@pytest.fixture
def client():
    app = create_app(drone_mode="sim", model_mode="gemini")
    return TestClient(app)


def test_get_config(client):
    res = client.get("/api/config")
    assert res.status_code == 200
    data = res.json()
    assert data["framework"] == "AERIS"
    assert "model_mode" in data
    assert "stream_source" in data
    assert "drone_backend" in data
    assert "drone_uri" in data


def test_set_model_config(client):
    # Switch to Gemini
    res = client.post("/api/config/model", json={"model_mode": "gemini"})
    assert res.status_code == 200
    assert res.json()["model_mode"] == "gemini"

    # Switch to Local Split
    res2 = client.post("/api/config/model", json={"model_mode": "local_split"})
    assert res2.status_code == 200
    assert res2.json()["model_mode"] == "local_split"


def test_set_stream_config(client):
    res = client.post("/api/config/stream", json={"stream_source": "webcam", "camera_index": 0})
    assert res.status_code == 200
    assert res.json()["stream_source"] == "webcam"
    assert app_context["stream_source"] == "webcam"


def test_set_drone_config_sim(client):
    res = client.post("/api/config/drone", json={"drone_backend": "sim"})
    assert res.status_code == 200
    data = res.json()
    assert data["success"] is True
    assert data["drone_backend"] == "sim"
