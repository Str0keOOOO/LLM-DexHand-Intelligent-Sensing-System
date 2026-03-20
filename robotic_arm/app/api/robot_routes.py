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
    开始JOG运动
    """
    robot = robot_manager.get_robot()
    if robot is None:
        robot_manager.connect()
        robot = robot_manager.get_robot()

    try:
        robot.ImmStopJOG()
        robot.StartJOG(payload.ref, payload.nb, payload.dir, payload.vel, payload.acc, payload.max_dis)
        time.sleep(1)
        return SuccessResponse(success=True)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"启动JOG失败: {e}")


# TODO 搞一下到底应该怎么重置
@router.get("/reset")
def reset_robot():
    """s
    重置机器人连接
    """
    robot = robot_manager.get_robot()
    if robot is None:
        robot_manager.connect()
        robot = robot_manager.get_robot()

    try:
        robot.DragTeachSwitch(1)
        time.sleep(10)
        robot.DragTeachSwitch(0)

        return SuccessResponse(success=True)
    except Exception as e:
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
