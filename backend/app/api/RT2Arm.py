import httpx
from fastapi import APIRouter
from app.schemas import TrackScriptRequest, SysVarRequest
from app.robotic_arm.bridge import _request

router = APIRouter()


@router.post("/connect")
async def connect_robot():
    return await _request("POST", "/api/connect")


@router.get("/get-actual-joint-pos-degree")
async def get_actual_joint_pos_degree():
    return await _request("GET", "/api/get-actual-joint-pos-degree")


@router.post("/start-jog")
async def start_jog(payload: dict):
    """
    payload 示例:
    {
      "ref": 4,
      "axis": "x",  # x, y, z, Rx, Ry, Rz
      "direction": "positive",  # positive or negative
      "vel": 20.0,
      "acc": 20.0,
      "max_dist": 30.0
    }
    """
    axis_mapping = {"x": 1, "y": 2, "z": 3, "Rx": 4, "Ry": 5, "Rz": 6}
    direction_mapping = {"positive": 1, "negative": 0}

    if payload["axis"] not in axis_mapping or payload["direction"] not in direction_mapping:
        return {"error": "Invalid axis or direction"}

    payload["axis_id"] = axis_mapping[payload.pop("axis")]
    payload["direction"] = direction_mapping[payload["direction"]]
    return await _request("POST", "/api/start-jog", json=payload)


@router.post("/reset")
async def reset():
    return await _request("POST", "/api/reset")
