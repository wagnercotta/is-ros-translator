"""External E2E port is fixed in the lab; internal API stays on port 8080."""
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_gateway_host_port_and_e2e_intra_network_ports_are_fixed():
    cfg = yaml.safe_load((ROOT / "deploy/e2e/compose.yaml").read_text())
    services = cfg["services"]
    assert services["gateway"]["ports"] == ["127.0.0.1:18080:8080"]
    assert services["e2e"]["environment"]["GATEWAY_URL"] == "http://gateway:8080"
    assert "127.0.0.1:8080/health/ready" in " ".join(services["gateway"]["healthcheck"]["test"])
    assert not (ROOT / "deploy/e2e/.env.example").exists()


def test_gateway_port_is_only_published_to_loopback():
    cfg = yaml.safe_load((ROOT / "deploy/e2e/compose.yaml").read_text())
    for spec in cfg["services"]["gateway"]["ports"]:
        assert spec.startswith("127.0.0.1:")
