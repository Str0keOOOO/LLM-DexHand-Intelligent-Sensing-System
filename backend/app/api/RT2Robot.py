import json
import asyncio
from contextlib import asynccontextmanager
from typing import Dict, Any

from fastapi import APIRouter, WebSocket, WebSocketDisconnect, FastAPI, Request, HTTPException
from pydantic import BaseModel

from app.ros.bridge import ROSBridgeManager


# 定义控制指令的数据模型
class ControlCommand(BaseModel):
    hand: str  # "left" or "right"
    joints: Dict[str, float]  # 例如 {"th_dip": 30.0, "ff_mcp": 10.5}


@asynccontextmanager
async def lifespan(app: FastAPI):
    # 启动时初始化 ROS Bridge
    mgr = getattr(app.state, "ros_bridge", None)
    if mgr is None:
        mgr = ROSBridgeManager()
        app.state.ros_bridge = mgr

    mgr.start()
    yield
    # 关闭时逻辑（如果需要可以添加 rclpy.shutdown）


router = APIRouter(lifespan=lifespan)


@router.get("/health")
async def health_check(request: Request):
    bridge: ROSBridgeManager | None = getattr(request.app.state, "ros_bridge", None)
    is_connected = bool(bridge and bridge.is_started())
    return {
        "status": "ok",
        "ros_bridge": "connected" if is_connected else "initializing",
        "mode": "VIRTUAL_DEXHAND",  # 标记当前模式
    }


@router.post("/control")
async def send_control_command(request: Request, cmd: ControlCommand):
    """
    接收前端发送的控制指令并转发给 ROS
    """
    bridge: ROSBridgeManager | None = getattr(request.app.state, "ros_bridge", None)

    if not bridge or not bridge.is_started():
        raise HTTPException(status_code=503, detail="ROS Bridge not initialized")

    # 验证手部名称
    if cmd.hand not in ["left", "right"]:
        raise HTTPException(status_code=400, detail="Invalid hand name. Must be 'left' or 'right'")

    # 调用 Bridge 发送指令
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

            # 获取 bridge 中聚合的最新全量数据
            data = bridge.get_latest_state()

            # 如果数据为空（ROS节点未启动或未收到数据），发送心跳包或空包
            if not data:
                await asyncio.sleep(0.1)
                continue

            # 直接发送完整数据结构
            await websocket.send_text(json.dumps(data))

            # 控制推送频率 (例如 20Hz = 0.05s, 10Hz = 0.1s)
            # 前端渲染压力大时可以适当调大这个值
            await asyncio.sleep(0.05)

    except WebSocketDisconnect:
        print("WS Client disconnected")
    except Exception as e:
        print(f"WS Error: {e}")
