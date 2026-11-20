"""FastAPI control plane, probe endpoints, REST commands and event websocket."""
from __future__ import annotations
import json
import secrets
from contextlib import asynccontextmanager
from fastapi import FastAPI, Header, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
from pydantic import ValidationError
from .config import Settings
from .errors import GatewayError
from .core.gateway import Gateway
from .models import Command, SubscriptionSpec


def create_app(settings: Settings, gateway: Gateway | None = None, orchestrators=None) -> FastAPI:
    gateway = gateway or Gateway(settings)
    orchestrators = list(orchestrators or [])
    api_key = settings.api_key()

    @asynccontextmanager
    async def lifespan(app):
        await gateway.start()
        for runner in orchestrators:
            await runner.start(gateway)
        yield
        for runner in reversed(orchestrators):
            await runner.stop()
        await gateway.stop()

    app = FastAPI(title="is-ros-translator v2", version="0.5.0", lifespan=lifespan)
    app.state.gateway = gateway
    app.state.transports = orchestrators

    @app.middleware("http")
    async def max_request_and_auth(request: Request, next_handler):
        # Health probes remain unauthenticated; all operational paths require the key.
        if request.url.path not in ("/health/live", "/health/ready"):
            if settings.security.require_api_key and not secrets.compare_digest(
                request.headers.get("x-api-key", ""), api_key):
                return JSONResponse({"error": "unauthorised"}, status_code=401)
        size = request.headers.get("content-length")
        if size:
            try:
                declared = int(size)
            except ValueError:
                return JSONResponse({"error": "invalid_content_length"}, status_code=400)
            if declared > settings.security.max_request_bytes:
                return JSONResponse({"error": "request_too_large"}, status_code=413)
        return await next_handler(request)

    @app.exception_handler(GatewayError)
    async def handle_gateway_error(request, exc: GatewayError):
        return JSONResponse({"ok": False, "error": exc.code, "detail": str(exc)}, status_code=exc.status)

    @app.exception_handler(ValidationError)
    async def handle_pydantic(request, exc):
        return JSONResponse({"ok": False, "error": "validation_error", "detail": str(exc)}, status_code=422)

    @app.get("/health/live")
    async def live():
        return {"status": "alive"}

    @app.get("/health/ready")
    async def ready():
        transport_health = [runner.health() for runner in orchestrators if hasattr(runner, "health")]
        return {"status": "ready" if all(t["connected"] for t in transport_health) else "degraded",
                "robots": {rid: bridge.connected() for rid, bridge in gateway._bridges.items()},
                "transports": transport_health, "events_dropped": gateway.events.dropped}

    @app.get("/v1/robots")
    async def robots():
        return [{"id": rid, "ros_version": cfg.ros_version, "connected": gateway.bridge(rid).connected(),
                 "bridge_id": cfg.bridge_id,
                 "distribution": (settings.bridges[cfg.bridge_id].distribution if cfg.bridge_id else None),
                 "navigation_enabled": cfg.navigation.enabled,
                 "capabilities": sorted(cfg.capabilities)} for rid, cfg in settings.robots.items()]

    @app.get("/v1/robots/{robot_id}/capabilities")
    async def robot_capabilities(robot_id: str):
        cfg = gateway.robot(robot_id)
        return {"robot_id": robot_id,
                "capabilities": {name: definition.model_dump()
                                 for name, definition in cfg.capabilities.items()}}

    @app.post("/v1/commands")
    async def command(cmd: Command):
        return await gateway.execute(cmd)

    @app.get("/v1/subscriptions")
    async def list_subscriptions():
        return gateway.subscriptions.summary()

    @app.post("/v1/subscriptions")
    async def add_subscription(spec: SubscriptionSpec):
        return await gateway.subscriptions.add(spec)

    @app.delete("/v1/subscriptions")
    async def remove_subscription(spec: SubscriptionSpec):
        return await gateway.subscriptions.remove(spec)

    @app.websocket("/v1/events")
    async def stream_events(ws: WebSocket):
        if settings.security.require_api_key and not secrets.compare_digest(ws.headers.get("x-api-key", ""), api_key):
            await ws.close(code=1008)
            return
        await ws.accept()
        key, queue = gateway.events.register()
        try:
            while True:
                await ws.send_json(await queue.get())
        except (WebSocketDisconnect, RuntimeError):
            pass
        finally:
            gateway.events.unregister(key)
    return app
