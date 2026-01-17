from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session
from app.schemas import ChatRequest, ChatResponse
import time
import math
import random

# 导入模块
from app.llm.client import ask_ai
from app.ros.bridge import ros_node
from app.database.mysql import get_db
from app.schemas import ChatRequest, ChatResponse, CheckModelRequest, CheckModelResponse
from app.llm.client import ask_ai, validate_model
from app.schemas import ModelListResponse, ModelOption  # 记得导入新 Schema
from app.config import settings
from app.schemas import ChatRequest, ChatResponse, ModelListResponse, ModelOption, CheckModelRequest, CheckModelResponse
# from app.database.models import ChatLog # 暂时注释

router = APIRouter()


# --- 1. LLM 对话接口 ---
@router.post("/chat/send", response_model=ChatResponse)
async def chat(req: ChatRequest, db: Session = Depends(get_db)):
    """
    Chat 接口现在支持选择模型。
    前端 JSON 示例: { "message": "你好", "model": "deepseek-chat" }
    """
    print(f"收到聊天请求，消息: {req.message}，模型: {req.model}")
    # 1. 准备上下文
    context_str = ""
    if ros_node and ros_node.latest_state:
        fingers = ros_node.latest_state.get("fingers", [])
        context_str = f"\n(Current Sensor Data: {fingers})"

    # 2. [关键修改] 调用 ask_ai，传入前端选择的 model
    # ask_ai 现在返回两个值：回复内容 和 实际使用的模型名
    reply_text, used_model = ask_ai(
        text=req.message + context_str,
        system_prompt="You are a helpful robot assistant. You can control the robotic hand via specific commands.",
        model_name=req.model,  # <--- 这里传入前端选的模型
    )

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

    # 4. 返回结果，包含 model_name
    return ChatResponse(
        reply=reply_text,
        model_name=used_model,  # <--- 告诉前端用的哪个模型
        action_code=action_code,
    )


# --- 2. 机器人状态接口 (保持不变) ---
@router.get("/robot/status")
async def get_robot_status():
    # ... (保持你原本的代码不变) ...
    if ros_node and ros_node.latest_state and ros_node.latest_state.get("timestamp", 0) > 0:
        return ros_node.latest_state

    t = time.time()
    f1 = 5.0 + 3.0 * math.sin(t * 2.0)
    f2 = 4.0 + 2.0 * math.sin(t * 2.5 + 1.0)
    f3 = 2.0 + 1.0 * math.cos(t * 1.5)
    f4 = 1.0 + random.uniform(0, 0.5)

    return {"fingers": [abs(f1), abs(f2), abs(f3), abs(f4)], "timestamp": t, "status": "simulated", "error": "ROS not connected (Displaying Simulated Data)"}


@router.post("/chat/check", response_model=CheckModelResponse)
async def check_model_connection(req: CheckModelRequest):
    """
    恢复真实的连接检查 (必须配合 client.py 的异步实现)
    """
    print(f"正在检测模型连接: {req.model}")
    
    # [关键恢复] 调用异步验证函数
    # 注意：这里必须有 await，且 client.py 里必须是 async def
    success, msg = await validate_model(req.model)
    
    return CheckModelResponse(success=success, message=msg)


@router.get("/chat/models", response_model=ModelListResponse)
async def get_available_models():
    """
    返回后端支持的所有模型列表，供前端下拉框使用
    """
    # 将 config 中的字典列表转换为 Pydantic 对象列表
    model_list = [ModelOption(**m) for m in settings.AVAILABLE_MODELS]
    return ModelListResponse(models=model_list)
