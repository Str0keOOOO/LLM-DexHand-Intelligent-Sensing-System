from fastapi import FastAPI, HTTPException, APIRouter
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
import Robot

clientRobot = None

app = FastAPI(title="Robotic Arm API")

# 统一 API 前缀
router = APIRouter()


@router.post("/connect")
def connectRobot():
    """
    连接机器人
    """
    global clientRobot
    if clientRobot is not None:
        return {"ok": True, "msg": "already connected"}

    try:
        host = "192.168.58.2"
        clientRobot = Robot.RPC(host)
        return {"ok": True, "msg": "connected", "host": host}
    except Exception as e:
        clientRobot = None
        raise HTTPException(status_code=500, detail=f"连接失败: {e}")


@router.post("/call-track-script/{n}")
def callTrackScript(n: int):
    """
    调用机器人第n段轨迹
    """
    global clientRobot
    path = f"/usr/local/etc/controller/lua/UAV0{n}.lua"
    if clientRobot is None:
        connectRobot()
    clientRobot.SetSysVarValue(4, 0)
    clientRobot.Mode(0)  # 机器人切入自动运行模式
    clientRobot.ProgramLoad(path)  # 加载要执行的机器人程序
    code = clientRobot.ProgramRun()
    return {"ok": True, "code": code}


@router.post("/init-robot")
def initRobot():
    """
    初始化机器人
    """
    global clientRobot
    path = "/usr/local/etc/controller/lua/Init.lua"
    if clientRobot is None:
        connectRobot()

    clientRobot.Mode(0)  # 机器人切入自动运行模式
    clientRobot.ProgramLoad(path)  # 加载要执行的机器人程序，testPTP.lua
    clientRobot.ProgramRun()
    return {"ok": True, "msg": "Robot initialized"}


@router.get("/check-program-state")
def checkProgramState():
    """
    检测机器人程序运行状态
    """
    global clientRobot
    if clientRobot is None:
        connectRobot()
    state = clientRobot.GetProgramState()
    state_msg = {1: "程序停止或无程序运行", 2: "机器人程序运行中", 3: "机器人程序已暂停"}.get(state, "机器人程序状态未知")
    return {"ok": True, "state": state, "message": state_msg}


@router.get("/get-current-line")
def getCurrentLine():
    """
    获取机器人当前程序运行行号
    """
    global clientRobot
    if clientRobot is None:
        connectRobot()
    res = clientRobot.GetCurrentLine()
    return {"ok": True, "current_line": res[1]}

# TODO 如果后期发现比较多，就可以根据SDK的目录进行分类
@router.get("/get-actual-joint-pos-degree")
def getActualJointPosDegree():
    """
    获取机器人角度
    """
    global clientRobot
    if clientRobot is None:
        connectRobot()
    res = clientRobot.GetActualJointPosDegree()
    return {"ok": True, "joint_positions": res}


app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(router, prefix="/api", tags=["robotic-arm"])
