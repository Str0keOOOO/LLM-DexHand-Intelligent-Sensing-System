from fastapi import FastAPI, HTTPException, APIRouter
from pydantic import BaseModel, Field

from robotic_arm import RobotRPC

app = FastAPI(title="Robotic Arm API")

# TODO 用上config
# TODO env文件需不需要
robot = RobotRPC("192.168.58.2")


def _require(result, msg: str = "Robot not connected"):
    if result is None or result is False:
        raise HTTPException(status_code=503, detail=msg)
    return result


class TrackScriptRequest(BaseModel):
    n: int = Field(..., ge=0, description="第n段轨迹编号")


class SysVarRequest(BaseModel):
    idx: int = Field(..., ge=0, description="系统变量索引")


# 统一 API 前缀
router = APIRouter(prefix="/api", tags=["robotic-arm"])


@router.get("/connect")
async def api_connect():
    client = robot.connect()
    _require(client)
    return {"status": "connected", "robot_address": robot.robot_address}


@router.post("/call_track_script")
async def api_call_track_script(req: TrackScriptRequest):
    code = robot.call_track_script(req.n)
    _require(code, "call_track_script failed (not connected?)")
    return {"status": "ok", "code": code, "n": req.n}


@router.post("/init_robot")
async def api_init_robot():
    ok = robot.init_robot()
    _require(ok, "init_robot failed (not connected?)")
    return {"status": "ok"}


@router.get("/check_program_state")
async def api_check_program_state():
    state = robot.check_program_state()
    _require(state, "check_program_state failed (not connected?)")
    return {"status": "ok", "state": state}


@router.get("/get_current_line")
async def api_get_current_line():
    line = robot.get_current_line()
    _require(line, "get_current_line failed (not connected?)")
    return {"status": "ok", "line": line}


@router.post("/open_gripper")
async def api_open_gripper():
    ok = robot.open_gripper()
    _require(ok, "open_gripper failed (not connected?)")
    return {"status": "ok"}


@router.post("/close_gripper")
async def api_close_gripper():
    ok = robot.close_gripper()
    _require(ok, "close_gripper failed (not connected?)")
    return {"status": "ok"}


@router.get("/get_actual_joint_pos_degree")
async def api_get_actual_joint_pos_degree():
    res = robot.get_actual_joint_pos_degree()

    if isinstance(res, (tuple, list)) and len(res) >= 2:
        error, pos = res[0], res[1]
    else:
        error, pos = 0, res  # 认为无错误，res就是位置

    return {"error": error, "pos": pos}


@router.post("/get_sys_var_value")
async def api_get_sys_var_value(req: SysVarRequest):
    res = robot.get_sys_var_value(req.idx)
    _require(res, "get_sys_var_value failed (not connected?)")
    if isinstance(res, (list, tuple)) and len(res) >= 2:
        return {"status": "ok", "error": res[0], "value": res[1], "raw": res}
    return {"status": "ok", "raw": res}


app.include_router(router)
