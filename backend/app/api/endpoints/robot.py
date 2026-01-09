from fastapi import APIRouter
from app.schemas import RobotStatus
from datetime import datetime
import random

router = APIRouter()

@router.get("/status", response_model=RobotStatus)
async def get_robot_status():
    # --- 未来这里通过 ros_bridge 从 ROS2 话题或 InfluxDB 读取真实数据 ---
    
    # 模拟生成三个手指的力传感器数据 (0-10N 波动)
    fake_load = [
        round(random.uniform(5.0, 8.0), 2), # 食指
        round(random.uniform(3.0, 6.0), 2), # 中指
        round(random.uniform(0.5, 2.0), 2)  # 拇指
    ]
    
    return RobotStatus(
        timestamp=datetime.now().strftime("%H:%M:%S"),
        fingers=fake_load
    )