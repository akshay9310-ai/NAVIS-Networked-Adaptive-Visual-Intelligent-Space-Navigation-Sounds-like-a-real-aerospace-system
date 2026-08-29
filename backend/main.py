"""
NAVIS - FastAPI Backend Entrypoint
Exposes RESTful endpoints, background physics loop, and real-time WebSocket telemetry channel.
"""

from typing import Dict, Any, Optional
import asyncio
import json
import logging
from contextlib import asynccontextmanager

import os
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException, Query
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from pydantic import BaseModel, Field

from backend.simulation.engine import SimulationEngine
from backend.simulation.modes import BenchmarkEvaluator, NavigationMode

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("NAVIS-BACKEND")

# Central Engine Instance
engine = SimulationEngine(seed=42)

# Connected WebSocket Clients
connected_websockets: set[WebSocket] = set()


# Pydantic Request Models
class SpeedRequest(BaseModel):
    multiplier: float = Field(..., ge=0.1, le=10.0)


class ModeRequest(BaseModel):
    mode: str = Field(..., pattern="^(MODE_A|MODE_B|MODE_C)$")


class HazardInjectRequest(BaseModel):
    x: float
    y: float
    radius: float = 28.0
    hazard_type: str = "ROCK_FIELD"


class BenchmarkRequest(BaseModel):
    seed: int = 42


# Physics & Telemetry Background Task
async def simulation_loop():
    """High-frequency background simulation runner (20 Hz)."""
    target_dt = 0.05
    while True:
        start_time = asyncio.get_event_loop().time()
        try:
            if engine.is_running:
                engine.step(custom_dt=target_dt)

            # Broadcast state to WebSocket clients
            if connected_websockets:
                state_data = engine.get_full_state()
                payload = json.dumps(state_data)
                dead_sockets = set()
                for ws in connected_websockets:
                    try:
                        await ws.send_text(payload)
                    except Exception:
                        dead_sockets.add(ws)
                connected_websockets.difference_update(dead_sockets)

        except Exception as e:
            logger.error(f"Error in simulation loop: {e}", exc_info=True)

        elapsed = asyncio.get_event_loop().time() - start_time
        sleep_time = max(0.005, target_dt - elapsed)
        await asyncio.sleep(sleep_time)


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    logger.info("Initializing NAVIS Simulation Subsystems...")
    sim_task = asyncio.create_task(simulation_loop())
    yield
    # Shutdown
    logger.info("Shutting down NAVIS Simulation...")
    sim_task.cancel()
    try:
        await sim_task
    except asyncio.CancelledError:
        pass


app = FastAPI(
    title="NAVIS Space Navigation API",
    description="Networked Adaptive Visual & Intelligent Space Navigation Simulation",
    version="1.0.0",
    lifespan=lifespan,
)

# CORS configuration
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# ==========================
# REST API Endpoints
# ==========================

frontend_dist = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "frontend", "dist"))

@app.get("/")
async def root():
    index_html = os.path.join(frontend_dist, "index.html")
    if os.path.exists(index_html):
        return FileResponse(index_html)
    return {
        "system": "NAVIS",
        "name": "Networked Adaptive Visual & Intelligent Space Navigation",
        "status": "ONLINE",
        "sim_time": engine.sim_time,
        "is_running": engine.is_running,
        "mode": engine.active_mode,
        "message": "Frontend dist not found. Run 'npm run build' in frontend/ or visit Vite dev server on port 5173",
    }


@app.get("/api/health")
async def health():
    return {
        "system": "NAVIS",
        "name": "Networked Adaptive Visual & Intelligent Space Navigation",
        "status": "ONLINE",
        "sim_time": engine.sim_time,
        "is_running": engine.is_running,
        "mode": engine.active_mode,
    }


@app.get("/api/simulation/state")
async def get_simulation_state():
    return engine.get_full_state()


@app.post("/api/simulation/start")
async def start_simulation():
    engine.start()
    return {"status": "SUCCESS", "is_running": True}


@app.post("/api/simulation/pause")
async def pause_simulation():
    engine.pause()
    return {"status": "SUCCESS", "is_running": False}


@app.post("/api/simulation/reset")
async def reset_simulation(seed: Optional[int] = Query(None)):
    engine.reset(seed=seed)
    return {"status": "SUCCESS", "sim_time": 0.0}


@app.post("/api/simulation/speed")
async def set_speed(req: SpeedRequest):
    engine.set_speed(req.multiplier)
    return {"status": "SUCCESS", "speed_multiplier": engine.speed_multiplier}


@app.post("/api/simulation/mode")
async def set_mode(req: ModeRequest):
    engine.set_mode(req.mode)
    return {"status": "SUCCESS", "active_mode": engine.active_mode}


@app.get("/api/rover")
async def get_rover():
    return {
        "rover": engine.rover.to_dict(),
        "mission_status": engine.rover.mission_status,
        "active_path": engine.rover.active_path,
    }


@app.get("/api/satellites")
async def get_satellites():
    return {
        "satellites": engine.constellation.to_list(),
        "global_outage": engine.constellation.global_outage,
    }


@app.get("/api/terrain")
async def get_terrain():
    return engine.terrain.to_dict()


@app.post("/api/satellite/outage")
async def simulate_outage():
    engine.set_satellite_outage(True)
    return {"status": "OUTAGE_ACTIVE", "global_outage": True}


@app.post("/api/satellite/restore")
async def restore_satellite():
    engine.set_satellite_outage(False)
    return {"status": "RESTORED", "global_outage": False}


@app.get("/api/navigation")
async def get_navigation():
    return {
        "ekf": engine.latest_ekf,
        "sensors": engine.latest_sensors,
        "trajectory_prediction": engine.latest_prediction,
    }


@app.get("/api/metrics")
async def get_metrics():
    est_pos = (engine.latest_ekf.get("est_x", engine.rover.x), engine.latest_ekf.get("est_y", engine.rover.y))
    return {
        "metrics": engine.metrics.update(
            engine.sim_time,
            (engine.rover.x, engine.rover.y),
            est_pos,
            engine.latest_ekf.get("uncertainty_m", 1.0),
            engine.rover.speed,
            engine.constellation.to_list(),
        ),
        "time_series": engine.metrics.get_time_series(),
    }


@app.get("/api/decisions")
async def get_decisions():
    return {
        "logs": engine.logs,
        "latest_ai_decision": engine.latest_scheduler_decision,
        "hazard_alert": engine.latest_hazard_alert,
    }


@app.post("/api/hazard/inject")
async def inject_hazard(req: HazardInjectRequest):
    hz = engine.inject_hazard(req.x, req.y, req.radius, req.hazard_type)
    return {"status": "INJECTED", "hazard": hz}


@app.post("/api/demo/run")
async def run_demo():
    engine.demo_runner.start_demo()
    return {"status": "DEMO_STARTED", "demo_info": engine.demo_runner.get_status()}


@app.post("/api/benchmark/run")
async def run_benchmark(req: Optional[BenchmarkRequest] = None):
    seed = req.seed if req else 42
    results = BenchmarkEvaluator.run_benchmark_comparison(terrain_seed=seed)
    return results


# ==========================
# Real-Time WebSocket Channel
# ==========================

@app.websocket("/ws/simulation")
async def websocket_simulation_endpoint(websocket: WebSocket):
    await websocket.accept()
    connected_websockets.add(websocket)
    logger.info(f"WebSocket client connected. Total clients: {len(connected_websockets)}")
    try:
        # Send initial full state immediately
        await websocket.send_text(json.dumps(engine.get_full_state()))

        while True:
            # Handle incoming control commands over WebSocket
            data = await websocket.receive_text()
            try:
                cmd = json.loads(data)
                action = cmd.get("action")
                if action == "start":
                    engine.start()
                elif action == "pause":
                    engine.pause()
                elif action == "reset":
                    engine.reset(seed=cmd.get("seed"))
                elif action == "speed":
                    engine.set_speed(cmd.get("multiplier", 1.0))
                elif action == "mode":
                    engine.set_mode(cmd.get("mode", "MODE_C"))
                elif action == "outage":
                    engine.set_satellite_outage(True)
                elif action == "restore":
                    engine.set_satellite_outage(False)
                elif action == "inject_hazard":
                    engine.inject_hazard(
                        cmd.get("x", 250.0),
                        cmd.get("y", 250.0),
                        cmd.get("radius", 28.0),
                        cmd.get("type", "ROCK_FIELD"),
                    )
                elif action == "demo":
                    engine.demo_runner.start_demo()
            except Exception as e:
                logger.error(f"Error handling WebSocket message: {e}")

    except WebSocketDisconnect:
        if websocket in connected_websockets:
            connected_websockets.remove(websocket)
        logger.info(f"WebSocket client disconnected. Remaining: {len(connected_websockets)}")
    except Exception as e:
        if websocket in connected_websockets:
            connected_websockets.remove(websocket)
        logger.error(f"WebSocket error: {e}")

# ==========================
# Static Frontend Serving
# ==========================

frontend_dist = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "frontend", "dist"))
assets_dir = os.path.join(frontend_dist, "assets")
if os.path.exists(frontend_dist):
    if os.path.exists(assets_dir):
        app.mount("/assets", StaticFiles(directory=assets_dir), name="assets")

    @app.get("/{full_path:path}")
    async def serve_spa(full_path: str):
        # Allow /api and /ws through
        if full_path.startswith("api") or full_path.startswith("ws"):
            raise HTTPException(status_code=404, detail="Not Found")
        target_file = os.path.join(frontend_dist, full_path)
        if os.path.exists(target_file) and os.path.isfile(target_file):
            return FileResponse(target_file)
        return FileResponse(os.path.join(frontend_dist, "index.html"))

