import time
import math
import random
from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session

# --- 数据库与 ROS ---
from app.ros.bridge import ros_node
from app.database.mysql import get_db

# --- Schema 定义 ---
from app.schemas import ChatRequest, ChatResponse, CheckModelRequest, CheckModelResponse, ModelListResponse, ModelOption

# --- 关键修改：导入函数而不是类 ---
# 对应现在的 client.py (函数式版本)
from app.llm.client import ask_ai, validate_model, AVAILABLE_MODELS

router = APIRouter()


# --- 1. LLM 对话接口 ---
@router.post("/chat/send", response_model=ChatResponse)
async def chat(req: ChatRequest, db: Session = Depends(get_db)):
    """
    Chat 接口：调用函数式的 ask_ai
    """
    print(f"收到聊天请求，消息: {req.message}，模型: {req.model}")

    # 1. 准备上下文 (ROS 手部状态)
    context_str = ""
    if ros_node and ros_node.latest_state:
        fingers = ros_node.latest_state.get("fingers", [])
        context_str = f"\n(Current Sensor Data: {fingers})"

    # 2. [关键修改] 调用 ask_ai 函数
    # ask_ai 返回两个值：(回复内容, 实际使用的模型名)
    # 注意：目前的 ask_ai 是同步函数，直接调用即可
    try:
        reply_text, used_model = ask_ai(
            text=req.message + context_str, system_prompt="You are a helpful robot assistant. You can control the robotic hand via specific commands.", model_name=req.model
        )
    except Exception as e:
        reply_text = f"Error calling AI: {str(e)}"
        used_model = "error"

    # 3. [ROS] 解析并执行控制指令 (保持原有逻辑)
    action_code = None
    if ros_node:
        if "grasp" in reply_text.lower() or "抓" in reply_text:
            ros_node.send_action({"action": "grasp", "force": 5.0})
            action_code = "grasp"
        elif "open" in reply_text.lower() or "松" in reply_text:
            ros_node.send_action({"action": "open"})
            action_code = "open"
        elif "reset" in reply_text.lower():
            ros_node.send_action({"action": "reset"})
            action_code = "reset"

    # 4. 返回结果
    return ChatResponse(
        reply=reply_text,
        model_name=used_model,
        action_code=action_code,
    )


# --- 2. 机器人状态接口 (保持不变) ---
@router.get("/robot/status")
async def get_robot_status():
    if ros_node and ros_node.latest_state and ros_node.latest_state.get("timestamp", 0) > 0:
        return ros_node.latest_state

    # 模拟数据生成
    t = time.time()
    f1 = 5.0 + 3.0 * math.sin(t * 2.0)
    f2 = 4.0 + 2.0 * math.sin(t * 2.5 + 1.0)
    f3 = 2.0 + 1.0 * math.cos(t * 1.5)
    f4 = 1.0 + random.uniform(0, 0.5)

    return {"fingers": [abs(f1), abs(f2), abs(f3), abs(f4)], "timestamp": t, "status": "simulated", "error": "ROS not connected (Displaying Simulated Data)"}


# --- 3. 模型连接检查接口 ---
@router.post("/chat/check", response_model=CheckModelResponse)
async def check_model_connection(req: CheckModelRequest):
    """
    检查模型是否连通。
    调用 client.py 中的异步函数 validate_model
    """
    print(f"正在检测模型连接: {req.model}")

    # [关键修改] 直接调用异步验证函数
    success, msg = await validate_model(req.model)

    return CheckModelResponse(success=success, message=msg)


# --- 4. 获取模型列表接口 ---
@router.get("/chat/models", response_model=ModelListResponse)
async def get_available_models():
    """
    返回后端支持的所有模型列表
    直接使用从 app.llm.client 导入的列表常量
    """
    # 将字典列表转换为 Pydantic 对象列表
    model_list = [ModelOption(**m) for m in AVAILABLE_MODELS]
    return ModelListResponse(models=model_list)
