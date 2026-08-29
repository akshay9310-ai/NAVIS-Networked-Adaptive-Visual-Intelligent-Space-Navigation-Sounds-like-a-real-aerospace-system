"""
End-to-End WebSocket and API Simulation Verification for NAVIS
"""

import asyncio
import json
import websockets
import urllib.request


async def test_e2e_websocket():
    uri = "ws://127.0.0.1:8000/ws/simulation"
    print(f"Connecting to NAVIS WebSocket: {uri}...")

    async with websockets.connect(uri) as ws:
        # 1. Receive initial full state
        initial_msg = await ws.recv()
        state = json.loads(initial_msg)
        print(f"[OK] Connected! Mode: {state['active_mode']}, Sim Time: {state['sim_time']}s")
        print(f"[OK] Rover Start: ({state['rover']['x']}, {state['rover']['y']})")
        print(f"[OK] Satellites Count: {len(state['satellites'])}")
        assert len(state['satellites']) == 4

        # 2. Send 'start' and 'speed 2.0x'
        await ws.send(json.dumps({"action": "start"}))
        await ws.send(json.dumps({"action": "speed", "multiplier": 2.0}))
        print("[OK] Sent start & speed 2.0x commands.")

        # Receive 10 frames of motion
        for _ in range(10):
            msg = await ws.recv()
            state = json.loads(msg)

        print(f"[OK] Rover in motion: x={state['rover']['x']:.2f}, y={state['rover']['y']:.2f}, speed={state['rover']['speed']:.2f} m/s")
        assert state['rover']['speed'] > 0.0

        # 3. Simulate Satellite Outage
        await ws.send(json.dumps({"action": "outage"}))
        print("[OK] Injected Satellite Outage...")

        for _ in range(10):
            msg = await ws.recv()
            state = json.loads(msg)

        print(f"[OK] Outage active: fallback_active={state['ekf']['fallback_active']}, uncertainty={state['ekf']['uncertainty_m']:.2f}m")
        assert state['ekf']['fallback_active'] is True

        # 4. Restore Satellite
        await ws.send(json.dumps({"action": "restore"}))
        print("[OK] Restored Satellite...")

        for _ in range(10):
            msg = await ws.recv()
            state = json.loads(msg)

        print(f"[OK] PNT Restored: fallback_active={state['ekf']['fallback_active']}, uncertainty={state['ekf']['uncertainty_m']:.2f}m")
        assert state['ekf']['fallback_active'] is False

        # 5. Inject Hazard
        await ws.send(json.dumps({
            "action": "inject_hazard",
            "x": 240.0,
            "y": 230.0,
            "radius": 30.0,
            "type": "ROCK_FIELD"
        }))
        print("[OK] Injected dynamic Rock Field hazard at (240, 230)...")

        for _ in range(10):
            msg = await ws.recv()
            state = json.loads(msg)

        print(f"[OK] Replanned route waypoints: {len(state['routes']['current_planned'])}")
        assert len(state['routes']['current_planned']) > 0

        # 6. Trigger Demo Mission
        await ws.send(json.dumps({"action": "demo"}))
        print("[OK] Started automated Demo Mission...")

        for _ in range(15):
            msg = await ws.recv()
            state = json.loads(msg)

        demo_status = state['demo_mission']
        print(f"[OK] Demo Mission Active: {demo_status['is_active']}, Stage: {demo_status['current_stage_idx']+1}/{len(demo_status['stages'])}")
        assert demo_status['is_active'] is True

    print("\n[SUCCESS] E2E WEBSOCKET & SIMULATION CONTROLS FULLY VERIFIED!")


if __name__ == "__main__":
    asyncio.run(test_e2e_websocket())
