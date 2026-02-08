import httpx
from fastapi import APIRouter
from app.schemas import TrackScriptRequest, SysVarRequest
from app.robotic_arm.bridge import _request

router = APIRouter()


@router.get("/connect")
async def api_connect():
    return await _request("GET", "/api/connect")


@router.post("/call_track_script")
async def api_call_track_script(req: TrackScriptRequest):
    return await _request("POST", "/api/call_track_script", json=req.model_dump())


@router.post("/init_robot")
async def api_init_robot():
    return await _request("POST", "/api/init_robot")


@router.get("/check_program_state")
async def api_check_program_state():
    return await _request("GET", "/api/check_program_state")


@router.get("/get_current_line")
async def api_get_current_line():
    return await _request("GET", "/api/get_current_line")


@router.post("/open_gripper")
async def api_open_gripper():
    return await _request("POST", "/api/open_gripper")


@router.post("/close_gripper")
async def api_close_gripper():
    return await _request("POST", "/api/close_gripper")


@router.get("/get_actual_joint_pos_degree")
async def api_get_actual_joint_pos_degree():
    return await _request("GET", "/api/get_actual_joint_pos_degree")


@router.post("/get_sys_var_value")
async def api_get_sys_var_value(req: SysVarRequest):
    return await _request("POST", "/api/get_sys_var_value", json=req.model_dump())
