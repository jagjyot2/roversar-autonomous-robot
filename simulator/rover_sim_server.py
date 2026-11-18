"""FastAPI + WebSocket `/ws/rover` autonomous navigation simulator & HUD server."""

from __future__ import annotations

import asyncio
import contextlib
import json
from pathlib import Path
from typing import Literal

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from simulator.camera_feed import render_fpv_frame_jpeg
from simulator.config import RoverConfig
from simulator.engine import RoverSimulationEngine
from simulator.safety import DriveMode

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DASHBOARD_DIST = PROJECT_ROOT / "dashboard" / "dist"


class VelocityCommand(BaseModel):
    linear: float = Field(0.0, ge=-1.0, le=1.0, description="Linear velocity m/s")
    angular: float = Field(0.0, ge=-4.0, le=4.0, description="Angular velocity rad/s")


class WaypointCommand(BaseModel):
    x: float = Field(..., ge=-4.5, le=4.5)
    y: float = Field(..., ge=-4.5, le=4.5)


class ModeCommand(BaseModel):
    mode: Literal["IDLE", "MANUAL", "AUTONOMOUS", "ESTOP"]


class CommLossCommand(BaseModel):
    active: bool = True


class ConnectionManager:
    """Tracks connected `/ws/rover` dashboard clients."""

    def __init__(self) -> None:
        self.active: set[WebSocket] = set()

    async def connect(self, ws: WebSocket) -> None:
        await ws.accept()
        self.active.add(ws)

    def disconnect(self, ws: WebSocket) -> None:
        self.active.discard(ws)

    async def broadcast_json(self, data: dict) -> None:
        if not self.active:
            return
        msg = json.dumps(data)
        stale: list[WebSocket] = []
        for ws in list(self.active):
            try:
                await ws.send_text(msg)
            except Exception:
                stale.append(ws)
        for ws in stale:
            self.active.discard(ws)


def _apply_ws_message(engine: RoverSimulationEngine, msg: dict) -> None:
    msg_type = msg.get("type", "")
    if msg_type == "estop":
        engine.trigger_estop()
    elif msg_type == "clear_estop":
        engine.clear_estop()
    elif msg_type == "comm_loss":
        if msg.get("active", True):
            engine.inject_comm_loss()
        else:
            engine.restore_comm()
    elif msg_type == "reset":
        engine.reset()
    elif msg_type == "heartbeat":
        engine.feed_heartbeat()
    elif msg_type == "waypoint":
        engine.set_waypoint(float(msg["x"]), float(msg["y"]))
    elif msg_type == "mode":
        engine.set_mode(str(msg["mode"]))
    elif msg_type == "cmd_vel":
        v = float(msg.get("linear", msg.get("v", 0.0)))
        w = float(msg.get("angular", msg.get("w", 0.0)))
        engine.set_manual_velocity(v, w)
    elif "linear" in msg or "angular" in msg:
        v = float(msg.get("linear", 0.0))
        w = float(msg.get("angular", 0.0))
        engine.set_manual_velocity(v, w)


def create_app(config: RoverConfig | None = None) -> FastAPI:
    cfg = config or RoverConfig()
    engine = RoverSimulationEngine(config=cfg)
    ws_manager = ConnectionManager()

    @contextlib.asynccontextmanager
    async def lifespan(app_instance: FastAPI):
        stop_event = asyncio.Event()

        async def sim_worker() -> None:
            dt = 1.0 / cfg.sim_hz
            tick = 0
            while not stop_event.is_set():
                engine.step(dt)
                tick += 1
                # Broadcast full state + grid every 3 ticks (~16 Hz)
                if tick % 3 == 0 and ws_manager.active:
                    await ws_manager.broadcast_json(engine.get_telemetry(include_grid=True))
                await asyncio.sleep(dt)

        task = asyncio.create_task(sim_worker())
        app_instance.state.engine = engine
        app_instance.state.ws_manager = ws_manager
        try:
            yield
        finally:
            stop_event.set()
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await task

    app = FastAPI(
        title="RoverSAR Autonomous Ground Robot Simulator",
        version="1.0.0",
        description="Differential-drive physics, VL53L0X ToF raycaster, 2D occupancy SLAM, and safety watchdog API.",
        lifespan=lifespan,
    )
    app.state.engine = engine
    app.state.ws_manager = ws_manager

    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_methods=["*"],
        allow_headers=["*"],
    )

    if (DASHBOARD_DIST / "assets").is_dir():
        app.mount("/assets", StaticFiles(directory=str(DASHBOARD_DIST / "assets")), name="assets")

    @app.get("/api/health")
    async def health() -> dict:
        return {
            "status": "ok",
            "service": "roversar-simulator",
            "sim_hz": cfg.sim_hz,
            "watchdog_timeout_ms": cfg.watchdog_timeout_ms,
        }

    @app.get("/api/state")
    async def get_state(include_grid: bool = True) -> dict:
        return engine.get_telemetry(include_grid=include_grid)

    @app.post("/api/command")
    async def send_velocity(cmd: VelocityCommand) -> dict:
        engine.set_manual_velocity(cmd.linear, cmd.angular)
        return {"ok": True, "mode": engine.mode.value, "fault": engine.safety.active_fault.value}

    @app.post("/api/mode")
    async def set_mode(cmd: ModeCommand) -> dict:
        engine.set_mode(DriveMode(cmd.mode))
        return {"ok": True, "mode": engine.mode.value, "fault": engine.safety.active_fault.value}

    @app.post("/api/waypoint")
    async def set_waypoint(wp: WaypointCommand) -> dict:
        if engine.world.collides(wp.x, wp.y, 0.08):
            raise HTTPException(status_code=400, detail="Waypoint lies inside an obstacle or wall.")
        engine.set_waypoint(wp.x, wp.y)
        return {"ok": True, "waypoint": {"x": wp.x, "y": wp.y}, "mode": engine.mode.value}

    @app.post("/api/estop")
    async def trigger_estop() -> dict:
        engine.trigger_estop()
        return {"ok": True, "mode": engine.mode.value, "fault": engine.safety.active_fault.value}

    @app.post("/api/clear-estop")
    async def clear_estop() -> dict:
        cleared = engine.clear_estop()
        return {"ok": cleared, "mode": engine.mode.value, "fault": engine.safety.active_fault.value}

    @app.post("/api/comm-loss")
    async def simulate_comm_loss(cmd: CommLossCommand) -> dict:
        if cmd.active:
            engine.inject_comm_loss()
        else:
            engine.restore_comm()
        return {
            "ok": True,
            "comm_loss_injected": engine.safety.comm_loss_injected,
            "fault": engine.safety.active_fault.value,
        }

    @app.post("/api/heartbeat")
    async def heartbeat() -> dict:
        engine.feed_heartbeat()
        return {"ok": True, "watchdog_ms": engine.safety.watchdog.elapsed_ms(engine.sim_time_ms)}

    @app.post("/api/reset")
    async def reset_sim() -> dict:
        engine.reset()
        return {"ok": True, "state": engine.get_telemetry(include_grid=True)}

    @app.get("/api/camera/frame.jpg")
    async def camera_frame() -> Response:
        jpeg = render_fpv_frame_jpeg(engine)
        if not jpeg:
            raise HTTPException(status_code=503, detail="Camera encoder unavailable")
        return Response(content=jpeg, media_type="image/jpeg")

    @app.get("/api/camera/stream.mjpeg")
    async def camera_stream() -> StreamingResponse:
        async def frame_gen():
            while True:
                jpeg = render_fpv_frame_jpeg(engine)
                if jpeg:
                    yield (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        + f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii")
                        + jpeg
                        + b"\r\n"
                    )
                await asyncio.sleep(0.1)

        return StreamingResponse(
            frame_gen(),
            media_type="multipart/x-mixed-replace; boundary=frame",
        )

    @app.websocket("/ws/rover")
    async def rover_ws(ws: WebSocket) -> None:
        await ws_manager.connect(ws)
        try:
            await ws.send_text(json.dumps(engine.get_telemetry(include_grid=True)))
            while True:
                raw = await ws.receive_text()
                try:
                    data = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                if isinstance(data, dict):
                    _apply_ws_message(engine, data)
                    await ws.send_text(json.dumps(engine.get_telemetry(include_grid=True)))
        except WebSocketDisconnect:
            ws_manager.disconnect(ws)

    @app.get("/", response_class=HTMLResponse)
    async def serve_dashboard() -> HTMLResponse:
        index_file = DASHBOARD_DIST / "index.html"
        if index_file.is_file():
            return HTMLResponse(index_file.read_text(encoding="utf-8"))
        return HTMLResponse(
            "<html><body style='background:#0b1120;color:#e2e8f0;font-family:sans-serif;padding:2rem'>"
            "<h1>RoverSAR Control HUD</h1>"
            "<p>Dashboard build missing. Run <code>npm install && npm run build</code> inside <code>dashboard/</code>.</p>"
            "</body></html>",
            status_code=200,
        )

    return app


app = create_app()

if __name__ == "__main__":  # pragma: no cover
    import uvicorn

    uvicorn.run("simulator.rover_sim_server:app", host="0.0.0.0", port=8050, reload=False)
