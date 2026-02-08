from typing import Any, Optional

import httpx
from fastapi import HTTPException

# 解决用docker会出问题的可能
ROBOT_BACKEND_BASE_URL = "http://127.0.0.1:8001"
REQUEST_TIMEOUT_S = 10.0


async def _request(
    method: str,
    path: str,
    *,
    json: Optional[dict] = None,
    params: Optional[dict] = None,
) -> Any:
    url = f"{ROBOT_BACKEND_BASE_URL}{path}"
    try:
        async with httpx.AsyncClient(timeout=REQUEST_TIMEOUT_S) as client:
            resp = await client.request(method, url, json=json, params=params)
    except httpx.ConnectError:
        raise HTTPException(status_code=503, detail=f"Upstream not reachable: {url}")
    except httpx.TimeoutException:
        raise HTTPException(status_code=504, detail=f"Upstream timeout: {url}")
    except httpx.HTTPError as e:
        raise HTTPException(status_code=502, detail=f"Upstream HTTP error: {e!s}")

    if resp.status_code < 200 or resp.status_code >= 300:
        detail: Any
        try:
            detail = resp.json()
        except Exception:
            detail = resp.text
        raise HTTPException(status_code=resp.status_code, detail=detail)

    try:
        return resp.json()
    except Exception:
        return resp.text
