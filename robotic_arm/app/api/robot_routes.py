from fastapi import APIRouter, HTTPException
import math
from pydantic import BaseModel
from app.services.robot_manager import robot_manager
import time

router = APIRouter()


class SuccessResponse(BaseModel):
    success: bool


class JogRequest(BaseModel):
    ref: int = 4
    nb: int
    dir: int = 0
    vel: float = 20.0
    acc: float = 20.0
    max_dis: float = 30.0


class JogData(BaseModel):
    x: float = 0
    y: float = 0
    z: float = 0
    rx: float = 0
    ry: float = 0
    rz: float = 0
    timestamp: int = 0


class JogResponse(BaseModel):
    success: bool = True
    data: JogData


@router.get("/pos")
def get_actual_joint_pos_degree():
    """
    获取机器人角度与末端位姿 (正解)
    """
    robot = robot_manager.get_robot()
    if robot is None:
        robot_manager.connect()
        robot = robot_manager.get_robot()

    try:
        res = robot.GetActualJointPosRadian(1)
        radians = res[1]
        degrees = [math.degrees(rad) for rad in radians]
        success, res_tcp = robot.GetForwardKin(degrees)
        if success == 0:
            x, y, z, rx, ry, rz = res_tcp
            return JogResponse(
                success=True,
                data=JogData(x=x, y=y, z=z, rx=rx, ry=ry, rz=rz, timestamp=int(time.time())),
            )
        else:
            raise HTTPException(status_code=500, detail="正解计算失败")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"获取机器人角度失败: {e}")


# TODO max_dis根本不会影响到移动距离不知道为什么
@router.post("/move")
def start_jog(payload: JogRequest):
    """
    开始JOG运动（带自愈机制）
    """
    robot = robot_manager.get_robot()
    if robot is None:
        raise HTTPException(status_code=503, detail="机器人未连接")

    try:
        robot.ResetAllError()
        robot.RobotEnable(1)
        robot.ImmStopJOG()
        ret = robot.StartJOG(payload.ref, payload.nb, payload.dir, payload.vel, payload.acc, payload.max_dis)
        
        if ret == 0:
            return SuccessResponse(success=True)
        else:
            error_msg = f"机器人拒绝移动，错误码: {ret}。"
            if ret == 14:
                error_msg += " 存在不可复位的物理干预，请登录机械臂 Web 界面查看顶部红色报警提示（如：急停被按下、轴超限、严重碰撞等）。"
            raise HTTPException(status_code=500, detail=error_msg)

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"启动JOG异常: {e}")


# TODO 搞一下到底应该怎么重置
@router.get("/reset")
def reset_robot():
    """
    重置机器人：切换到手动模式 -> 开启拖动(10s) -> 关闭拖动 -> 回到自动模式并使能
    """
    robot = robot_manager.get_robot()
    if robot is None:
        robot_manager.connect()
        robot = robot_manager.get_robot()

    try:
        robot.Mode(1)
        time.sleep(0.5)

        ret = robot.DragTeachSwitch(1)
        if ret != 0:
            raise HTTPException(status_code=500, detail=f"开启拖动失败，错误码: {ret}")

        print("机器人已进入拖动模式，限时10秒...")
        time.sleep(10)

        robot.DragTeachSwitch(0)
        time.sleep(0.5)

        robot.Mode(0)

        robot.RobotEnable(1)
        time.sleep(1)

        return SuccessResponse(success=True)
    except Exception as e:
        robot.ResetAllError()
        raise HTTPException(status_code=500, detail=f"重置失败: {e}")


@router.get("/check")
def check_robot():
    """
    检查机器人连接状态
    """
    robot = robot_manager.get_robot()
    if robot is None:
        robot_manager.connect()
        robot = robot_manager.get_robot()

    if robot is not None:
        return SuccessResponse(success=True)
    else:
        raise HTTPException(status_code=500, detail="机器人连接失败")
