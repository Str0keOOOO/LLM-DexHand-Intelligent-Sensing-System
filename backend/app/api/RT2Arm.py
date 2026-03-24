import time
from fastapi import APIRouter
from app.schemas import ArmJointsCommand, SuccessResponse
from influxdb_client import Point
from app.robotic_arm.bridge import _request
from fastapi import WebSocket, WebSocketDisconnect
from app.database.influx import write_api, INFLUXDB_BUCKET, org
import asyncio

router = APIRouter()


@router.post("/check")
async def connect_robot() -> SuccessResponse:
    return await _request("POST", "/api/check")


@router.websocket("/pos")
async def get_actual_joint_pos_degree_ws(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            data = await _request("GET", "/api/pos")

            if data and isinstance(data, dict):
                pt = Point("arm_joints").time(time.time_ns())
                has_valid_field = False
                for k, v in data.items():
                    try:
                        pt.field(str(k), float(v))
                        has_valid_field = True
                    except (ValueError, TypeError):
                        pass
                if has_valid_field:
                    write_api.write(bucket=INFLUXDB_BUCKET, org=org, record=pt)

            await websocket.send_json(data)
            await asyncio.sleep(0.05)
    except WebSocketDisconnect:
        print("WS Client disconnected")
    except Exception as e:
        print(f"WS Error: {e}")


@router.post("/move")
async def start_jog(payload: dict):
    nb_mapping = {"x": 1, "y": 2, "z": 3, "Rx": 4, "Ry": 5, "Rz": 6}
    dir_mapping = {"positive": 1, "negative": 0}

    payload["nb"] = nb_mapping[payload["nb"]]
    payload["dir"] = dir_mapping[payload["dir"]]

    cmd = ArmJointsCommand(**payload)
    return await _request("POST", "/api/move", json=cmd.model_dump())


@router.post("/reset")
async def reset() -> SuccessResponse:
    return await _request("POST", "/api/reset")
