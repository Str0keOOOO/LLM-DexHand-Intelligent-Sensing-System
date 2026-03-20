from typing import Any, Optional, Dict

import httpx
from fastapi import HTTPException

ROBOT_BACKEND_BASE_URL = "http://127.0.0.1:8001"
REQUEST_TIMEOUT_S = 10.0


async def _request(method: str, path: str, json: Optional[dict] = None) -> Any:
    url = f"{ROBOT_BACKEND_BASE_URL}{path}"
    try:
        async with httpx.AsyncClient(timeout=REQUEST_TIMEOUT_S) as client:
            resp = await client.request(method, url, json=json)
    except httpx.RequestError as e:
        raise HTTPException(status_code=503, detail=f"Upstream not reachable: {url}") from e

    # 透传上游的 HTTP 错误（带上 body 便于排障）
    if resp.status_code >= 400:
        detail: Any
        try:
            detail = resp.json()
        except Exception:
            detail = resp.text
        raise HTTPException(status_code=resp.status_code, detail={"upstream": url, "detail": detail})

    # 成功则尽量返回 json，否则返回 text
    try:
        return resp.json()
    except Exception:
        return resp.text


# 1) 对应上游：POST /connect
async def connect_robot() -> Dict[str, Any]:
    return await _request("POST", "/api/connect")


# 2) 对应上游：GET /get-actual-joint-pos-degree
async def get_actual_joint_pos_degree() -> Any:
    return await _request("GET", "/api/get-actual-joint-pos-degree")


# 3) 对应上游：POST /start-jog
# payload 结构和你上游 JogRequest 一致
async def start_jog(payload: dict) -> Dict[str, Any]:
    """
    payload 示例:
    {
      "ref": 4,
      "axis_id": 1,
      "direction": 1,
      "vel": 20.0,
      "acc": 20.0,
      "max_dist": 30.0
    }
    """
    return await _request("POST", "/api/start-jog", json=payload)

async def reset() -> Dict[str, Any]:
    return await _request("POST", "/api/reset")