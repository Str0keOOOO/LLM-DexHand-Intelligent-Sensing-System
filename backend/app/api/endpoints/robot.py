from fastapi import APIRouter
from app.schemas import RobotStatus
from datetime import datetime
from app.core.ros_client import latest_robot_state # 导入全局状态

router = APIRouter()

@router.get("/status", response_model=RobotStatus)
async def get_robot_status():
    # 直接从内存中读取 ROS 线程更新的最新数据
    real_data = latest_robot_state["fingers"]
    
    return RobotStatus(
        timestamp=datetime.now().strftime("%H:%M:%S"),
        fingers=real_data
    )