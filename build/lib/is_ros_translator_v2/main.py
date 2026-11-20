"""Gateway entrypoint: python -m is_ros_translator_v2.main --config configs/vm-demo.yaml"""
import argparse
import logging
import os
import uvicorn
from .api import create_app
from .config import load_settings
from .iswire_transport import ISWireTransport


def main():
    parser = argparse.ArgumentParser(description="PrIS / ROS 1 / ROS 2 gateway")
    parser.add_argument("--config", default=os.getenv("GATEWAY_CONFIG", "configs/vm-demo.yaml"))
    parser.add_argument("--log-level", default=os.getenv("LOG_LEVEL", "info"))
    args = parser.parse_args()
    logging.basicConfig(level=args.log_level.upper())
    settings = load_settings(args.config)
    logging.info("is-ros-translator v0.5.0: AMQP handled exclusively by is-wire/is-msgs; enabled=%s", settings.iswire.enabled)
    if not settings.http.enabled:
        parser.error("HTTP must be enabled; can bind to localhost for AMQP-only operation")
    transports = []
    if settings.iswire.enabled:
        transports.append(ISWireTransport(settings))
    app = create_app(settings, orchestrators=transports)
    uvicorn.run(app, host=settings.http.host, port=settings.http.port, log_level=args.log_level)


if __name__ == "__main__":
    main()
