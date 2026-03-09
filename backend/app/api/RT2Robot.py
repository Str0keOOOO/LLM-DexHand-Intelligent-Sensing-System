import json
import time
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

    if cmd.hand != "right":
        raise HTTPException(status_code=400, detail="Invalid hand name. Only 'right' is supported")

    print(cmd)
    bridge.send_command(cmd.hand, cmd.joints)

    return {"status": "success", "sent_to": cmd.hand, "command_count": len(cmd.joints)}


@router.websocket("/robot-data")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    bridge: ROSBridgeManager = getattr(websocket.app.state, "ros_bridge", None)

    try:
        while True:
            # 获取最新状态
            data = bridge.get_latest_state()

            # 【关键修改】：判断 ROS 节点是否启动，并且时间戳是否在最近 1 秒内更新过
            # 如果硬件断连，ROS topic 停止发布，timestamp 将不再更新
            import time

            is_alive = bool(data and (time.time() - data.get("timestamp", 0) < 1.0))
            ros_active = bool(bridge and bridge.is_started() and is_alive)

            if not ros_active:
                await websocket.send_json(
                    {
                        "ros_active": False,
                        "timestamp": time.time(),
                        "right": None,  # 清空旧数据
                    }
                )
                await asyncio.sleep(1.0)
                continue

            # 注入状态并发送
            data["ros_active"] = True
            await websocket.send_json(data)
            await asyncio.sleep(0.05)

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
