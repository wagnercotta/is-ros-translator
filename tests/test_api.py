from fastapi.testclient import TestClient
from is_ros_translator_v2.api import create_app


def test_http_auth(settings, gateway, monkeypatch):
    monkeypatch.setenv("GATEWAY_API_KEY", "sample-secret")
    settings.security.require_api_key = True
    app = create_app(settings, gateway)
    with TestClient(app) as client:
        assert client.get("/health/live").status_code == 200
        assert client.get("/v1/robots").status_code == 401
        result = client.get("/v1/robots", headers={"X-API-Key": "sample-secret"})
        assert result.status_code == 200
        assert len(result.json()) == 2
        result = client.post("/v1/commands", headers={"X-API-Key": "sample-secret"},
            json={"robot_id": "r1", "operation": "navigate", "args": {"position": {"x": 1, "y": 2}}})
        assert result.status_code == 200, result.text
        assert result.json()["ok"]
        forbidden = client.post("/v1/commands", headers={"X-API-Key": "sample-secret"},
            json={"robot_id": "r1", "operation": "ros.publish", "args": {"endpoint": "/cmd_vel", "payload": {}}})
        assert forbidden.status_code == 403
