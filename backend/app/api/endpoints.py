from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session
from app.schemas import ChatRequest
import time
import random
import math  # 引入数学库用于生成波形

# 导入模块
from app.llm.client import ask_ai
from app.ros.bridge import ros_node
from app.database.mysql import get_db
from app.database.models import ChatLog

router = APIRouter()

# --- 1. LLM 对话接口 ---
@router.post("/chat/send")
async def chat(req: ChatRequest, db: Session = Depends(get_db)):
    # 1. [DB] 暂时注释掉
    # db.add(ChatLog(role="user", content=req.message))
    # db.commit()

    # 2. [LLM] 思考
    context_str = ""
    # 尝试获取一点传感器数据作为上下文
    if ros_node and ros_node.latest_state:
        fingers = ros_node.latest_state.get("fingers", [])
        context_str = f"(Current Sensor Data: {fingers})"
    
    reply = ask_ai(req.message + context_str, "You are a helpful robot assistant.")

    # 3. [ROS] 执行控制指令
    if ros_node:
        if "grasp" in reply or "抓" in reply:
            ros_node.send_action({"action": "grasp", "force": 5.0})
        elif "open" in reply or "松" in reply:
            ros_node.send_action({"action": "open"})
        elif "reset" in reply:
            ros_node.send_action({"action": "reset"})

    # 4. [DB] 暂时注释掉
    # db.add(ChatLog(role="ai", content=reply))
    # db.commit()

    return {"reply": reply}

# --- 2. 机器人状态接口 (带模拟数据回退) ---
@router.get("/robot/status")
async def get_robot_status():
    """
    获取机器人状态。
    逻辑：如果 ROS 连接正常且有数据，返回真实数据；否则返回模拟波形数据。
    """
    
    # [方案 A] 优先返回真实数据 (原来那个数据代码保留在这里)
    # 判断条件：ros_node 存在，且最新数据的时间戳不为 0 (说明收到过数据)
    if ros_node and ros_node.latest_state and ros_node.latest_state.get("timestamp", 0) > 0:
        return ros_node.latest_state

    # [方案 B] 生成模拟数据 (当真实数据不可用时)
    # 使用正弦波生成呼吸效果，让前端图表动起来
    t = time.time()
    
    # 模拟 4 个手指的压力/位置数据 (0~10 之间波动)
    f1 = 5.0 + 3.0 * math.sin(t * 2.0)          # 食指：大幅波动
    f2 = 4.0 + 2.0 * math.sin(t * 2.5 + 1.0)    # 中指：不同频率
    f3 = 2.0 + 1.0 * math.cos(t * 1.5)          # 无名指
    f4 = 1.0 + random.uniform(0, 0.5)           # 拇指：随机噪点

    simulated_data = {
        "fingers": [abs(f1), abs(f2), abs(f3), abs(f4)], # 取绝对值保证为正
        "timestamp": t,
        "status": "simulated", # 标记这是模拟数据
        "error": "ROS not connected (Displaying Simulated Data)" # 提示信息
    }
    
    return simulated_data