import json
import asyncio


from fastapi import APIRouter, WebSocket, WebSocketDisconnect, FastAPI, Request, HTTPException

from app.ros.bridge import ROSBridgeManager
from app.schemas import ControlCommand


router = APIRouter()


@router.get("/health")
async def health_check(request: Request):
    bridge: ROSBridgeManager | None = getattr(request.app.state, "ros_bridge", None)
    is_connected = bool(bridge and bridge.is_started())
    return {
        "status": "ok",
        "ros_bridge": "connected" if is_connected else "initializing",
        "mode": "VIRTUAL_DEXHAND",
    }


@router.post("/control")
async def send_control_command(request: Request, cmd: ControlCommand):
    """
    接收前端发送的控制指令并转发给 ROS
    """
    bridge: ROSBridgeManager | None = getattr(request.app.state, "ros_bridge", None)

    if not bridge or not bridge.is_started():
        raise HTTPException(status_code=503, detail="ROS Bridge not initialized")

    if cmd.hand not in ["left", "right"]:
        raise HTTPException(status_code=400, detail="Invalid hand name. Must be 'left' or 'right'")

    bridge.send_command(cmd.hand, cmd.joints)

    return {"status": "success", "sent_to": cmd.hand, "command_count": len(cmd.joints)}


@router.websocket("/robot-data")
async def websocket_endpoint(websocket: WebSocket):
    """
    实时推送所有传感器数据 (Joints, Touch, Motor)
    """
    await websocket.accept()
    bridge = websocket.app.state.ros_bridge

    try:
        while True:
            if not bridge or not bridge.is_started():
                await asyncio.sleep(1.0)
                continue

            data = bridge.get_latest_state()

            if not data:
                await asyncio.sleep(0.1)
                continue

            await websocket.send_text(json.dumps(data))

            await asyncio.sleep(0.016)

    except WebSocketDisconnect:
        print("WS Client disconnected")
    except Exception as e:
        print(f"WS Error: {e}")


@router.post("/reset")
async def reset_robot(request: Request):
    bridge = request.app.state.ros_bridge
    if bridge and bridge._node:
        success = bridge._node.call_reset_service()
        return {"status": "success" if success else "failed"}
    raise HTTPException(status_code=503, detail="Bridge not ready")
