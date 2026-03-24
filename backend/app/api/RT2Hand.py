import time
import math
import asyncio

from influxdb_client import Point
from app.database.influx import write_api, INFLUXDB_BUCKET, org
from fastapi import APIRouter, WebSocket, WebSocketDisconnect, Request, HTTPException

from app.hand.bridge import ROSBridgeManager
from app.schemas import SuccessResponse, HandJointsCommand


router = APIRouter()


@router.get("/check")
async def health_check(request: Request) -> SuccessResponse:
    bridge: ROSBridgeManager | None = getattr(request.app.state, "ros_bridge", None)
    is_connected = bool(bridge and bridge.is_started())
    if is_connected:
        return SuccessResponse(success=True)
    else:
        raise HTTPException(status_code=503, detail="ROS Bridge not initialized")


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
            state = bridge.get_latest_state() if bridge else {}

            is_alive = bool(state and (time.time() - state.get("timestamp", 0) < 1.0))
            ros_active = bool(bridge and bridge.is_started() and is_alive)

            if not ros_active:
                await websocket.send_json(
                    {
                        "success": True,
                        "data": {},
                    }
                )
                await asyncio.sleep(1.0)
                continue

            positions = state.get("joint", {}).get("position", {})
            if positions:
                pt = Point("dexhand_joints").tag("hand", "right").time(time.time_ns())
                for k, v in positions.items():
                    try:
                        pt.field(str(k), float(v))
                    except (ValueError, TypeError):
                        pass
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
async def reset_robot(request: Request) -> SuccessResponse:
    bridge = request.app.state.ros_bridge
    if bridge and bridge._node:
        try:
            success = bridge._node.call_reset_service()
            if success:
                return SuccessResponse(success=True)
            else:
                raise HTTPException(status_code=500, detail="Failed to reset robot")
        except Exception as e:
            raise HTTPException(status_code=503, detail=str(e))
