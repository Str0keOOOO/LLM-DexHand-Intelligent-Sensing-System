import time
import math
import asyncio
import copy

from influxdb_client import Point
from app.database.influx import write_api, INFLUXDB_BUCKET, org
from fastapi import APIRouter, WebSocket, WebSocketDisconnect, Request, HTTPException

from app.hand.bridge import ROSBridgeManager
from app.schemas import SuccessResponse, HandJointsCommand


router = APIRouter()


@router.get("/check")
async def health_check(request: Request):
    bridge: ROSBridgeManager | None = getattr(request.app.state, "ros_bridge", None)
    is_connected = bool(bridge and bridge.is_connected())
    if is_connected:
        return SuccessResponse(success=True)
    else:
        raise HTTPException(status_code=503, detail="ROS Bridge connected but remote robot node is offline")


@router.post("/move")
async def send_control_command(request: Request, cmd: HandJointsCommand) -> SuccessResponse:
    bridge: ROSBridgeManager | None = getattr(request.app.state, "ros_bridge", None)
    if not bridge or not bridge.is_started():
        raise HTTPException(status_code=503, detail="ROS Bridge not initialized")

    joints_deg = cmd.model_dump()
    joints_rad = {k: (float(v) * math.pi / 180.0) for k, v in joints_deg.items()}
    try:
        bridge.send_command(joints_rad)
        return SuccessResponse(success=True)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.websocket("/data")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    bridge: ROSBridgeManager = getattr(websocket.app.state, "ros_bridge", None)

    try:
        while True:
            raw_state = bridge.get_latest_state() if bridge else {}
            state = copy.deepcopy(raw_state)

            is_alive = bool(state and (time.time() - state.get("timestamp", 0) < 1.0))
            ros_active = bool(bridge and bridge.is_started() and is_alive)

            if not ros_active:
                await websocket.send_json({"success": True, "data": {}})
                await asyncio.sleep(1.0)
                continue

            touch = state.get("touch") or {}
            prox = touch.get("proximity")
            if isinstance(prox, list):
                threshold = 10
                step_prox: list[int | None] = []
                for v in prox:
                    try:
                        p = float(v)
                        if not math.isfinite(p):
                            step_prox.append(None)
                        else:
                            step_prox.append(1 if p >= threshold else 0)
                    except (TypeError, ValueError):
                        step_prox.append(None)

                touch["proximity"] = step_prox
                state["touch"] = touch

            positions = state.get("joint", {}).get("position", {})
            if positions:
                pt = Point("dexhand_joints").tag("hand", "right").time(time.time_ns())
                write_api.write(bucket=INFLUXDB_BUCKET, org=org, record=pt)
            await websocket.send_json(
                {
                    "success": True,
                    "data": state,
                }
            )
            await asyncio.sleep(0.05)

    except WebSocketDisconnect:
        print("WS Client disconnected")
    except Exception as e:
        print(f"WS Error: {e}")


@router.get("/reset")
async def reset_robot(request: Request):
    bridge: ROSBridgeManager | None = getattr(request.app.state, "ros_bridge", None)
    if not bridge or not bridge.is_connected():
        raise HTTPException(status_code=503, detail="Robot offline, cannot reset")
    result = bridge._node.call_reset_service()
    if not result["success"]:
        raise HTTPException(status_code=500, detail=result["message"])

    return SuccessResponse(success=True)
