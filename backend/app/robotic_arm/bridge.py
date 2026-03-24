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
