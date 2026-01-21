import json
import asyncio
from contextlib import asynccontextmanager

from fastapi import APIRouter, WebSocket, WebSocketDisconnect, FastAPI, Request

from app.ros.bridge import ROSBridgeManager


@asynccontextmanager
async def lifespan(app: FastAPI):
    mgr = getattr(app.state, "ros_bridge", None)
    if mgr is None:
        mgr = ROSBridgeManager()
        app.state.ros_bridge = mgr

    mgr.start()
    yield


router = APIRouter(lifespan=lifespan)


@router.get("/health")
async def health_check(request: Request):
    bridge: ROSBridgeManager | None = getattr(request.app.state, "ros_bridge", None)
    is_connected = bool(bridge and bridge.is_started())
    return {"status": "ok", "ros_bridge": "connected" if is_connected else "initializing"}


@router.websocket("/robot-data")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    bridge = websocket.app.state.ros_bridge

    try:
        while True:
            data = bridge.get_latest_state()
            payload = data.get("payload")

            if payload is None:
                await asyncio.sleep(0.1)
                continue

            await websocket.send_text(json.dumps(data))
            await asyncio.sleep(0.1)

    except WebSocketDisconnect:
        print("WS Client disconnected")
    except Exception as e:
        print(f"WS Error: {e}")
