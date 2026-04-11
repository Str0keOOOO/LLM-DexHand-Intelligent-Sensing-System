import time
from fastapi import APIRouter, HTTPException
from app.schemas import ArmJointsCommand, SuccessResponse
from influxdb_client import Point
from app.robotic_arm.bridge import _request
from fastapi import WebSocket, WebSocketDisconnect
from app.database.influx import write_api, INFLUXDB_BUCKET, org
import asyncio

router = APIRouter()


@router.get("/check")
async def connect_robot() -> SuccessResponse:
    return await _request("get", "/api/check")


@router.websocket("/pos")
async def get_actual_joint_pos_degree_ws(websocket: WebSocket):
    """
    通过 WebSocket 实时获取机械臂位置数据并存入 InfluxDB。
    集成了处理嵌套 'data' 字段及过滤 'success' 标识的修复逻辑。
    """
    await websocket.accept()
    try:
        while True:
            data = await _request("GET", "/api/pos")

            if data and isinstance(data, dict):
                pt = Point("arm_joints").time(time.time_ns())
                has_valid_field = False

                actual_payload = data.get("data", {}) if isinstance(data.get("data"), dict) else data

                for k, v in actual_payload.items():
                    try:
                        if k in ["success", "timestamp"]:
                            continue

                        pt.field(str(k), float(v))
                        has_valid_field = True
                    except (ValueError, TypeError):
                        pass
                if has_valid_field:
                    write_api.write(bucket=INFLUXDB_BUCKET, org=org, record=pt)

            # 将原始数据实时推送到前端展示
            await websocket.send_json(data)

            # 设置采样频率（约 20Hz）
            await asyncio.sleep(0.2)

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


@router.get("/reset")
async def reset() -> SuccessResponse:
    try:
        await _request("get", "/api/reset")
        return SuccessResponse(success=True)
    except Exception as e:
        raise HTTPException(status_code=503, detail=str(e))
