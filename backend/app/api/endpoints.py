import time
import math
import random
import json
import asyncio

from fastapi import APIRouter, WebSocket, WebSocketDisconnect, Depends
from sqlalchemy.orm import Session

# --- 数据库与 ROS ---
# [关键修改] 统一导入 bridge_node，不再使用旧的 ros_node
from app.ros.bridge import bridge_node
from app.database.mysql import get_db

# --- Schema 定义 ---
from app.schemas import ChatRequest, ChatResponse, CheckModelRequest, CheckModelResponse, ModelListResponse, ModelOption

# --- LLM Client ---
from app.llm.client import ask_ai, validate_model, AVAILABLE_MODELS

router = APIRouter()

# ==========================================
# 1. 系统基础与 WebSocket 接口 (新增部分)
# ==========================================


@router.get("/health")
async def health_check():
    """
    前端轮询这个接口来判断后端是否连接 (通信状态)
    同时可以用来检查 ROS Bridge 是否已初始化
    """
    status_data = {"status": "ok", "ros_bridge": "connected" if bridge_node else "initializing"}
    return status_data


@router.websocket("/ws/robot-data")
async def websocket_endpoint(websocket: WebSocket):
    """
    前端连接这个 WS 来获取机器人实时数据
    数据源：backend/app/ros/bridge.py 中的 bridge_node.latest_state
    """
    await websocket.accept()
    try:
        while True:
            if bridge_node:
                # 1. 从 Bridge 获取最新数据 (包含 mode 和 payload)
                data = bridge_node.latest_state
                # 2. 发送给前端
                await websocket.send_text(json.dumps(data))
            else:
                # Bridge 还没启动时的等待状态
                await websocket.send_text(json.dumps({"mode": "BACKEND_INIT", "payload": None, "timestamp": time.time()}))

            # 控制发送频率，例如 10Hz
            await asyncio.sleep(0.1)

    except WebSocketDisconnect:
        print("WS Client disconnected")
    except Exception as e:
        print(f"WS Error: {e}")


# ==========================================
# 2. LLM 对话与控制接口 (合并修改部分)
# ==========================================


@router.post("/chat/send", response_model=ChatResponse)
async def chat(req: ChatRequest, db: Session = Depends(get_db)):
    """
    Chat 接口：调用函数式的 ask_ai，并根据回复执行 ROS 指令
    """
    print(f"收到聊天请求，消息: {req.message}，模型: {req.model}")

    # 1. 准备上下文 (使用 bridge_node 中的数据)
    context_str = ""
    if bridge_node and bridge_node.latest_state:
        # 这里的结构取决于 bridge.py 中存储的 latest_state 结构
        # 假设结构为 { "payload": { "joints": ..., "touch": ... }, "mode": ... }
        payload = bridge_node.latest_state.get("payload")
        if payload:
            context_str = f"\n(Current Robot State: {payload})"

    # 2. 调用 LLM
    try:
        reply_text, used_model = ask_ai(
            text=req.message + context_str, system_prompt="You are a helpful robot assistant. You can control the robotic hand via specific commands.", model_name=req.model
        )
    except Exception as e:
        reply_text = f"Error calling AI: {str(e)}"
        used_model = "error"

    # 3. [ROS] 解析并执行控制指令
    # 注意：如果你需要在后端 Bridge 发送指令，你需要在 BridgeNode 类中实现 publish 功能
    # 这里暂时保留逻辑，假设 bridge_node 未来会支持 send_action 或类似方法
    action_code = None

    # 简单的指令关键词匹配 (仅示例，实际可能需要更复杂的意图识别)
    if bridge_node:
        # TODO: 确保你的 bridge_node 实现了指令发布功能，否则这里仅做日志记录
        if "grasp" in reply_text.lower() or "抓" in reply_text:
            # bridge_node.send_action({"action": "grasp", "force": 5.0})
            action_code = "grasp"
        elif "open" in reply_text.lower() or "松" in reply_text:
            # bridge_node.send_action({"action": "open"})
            action_code = "open"
        elif "reset" in reply_text.lower():
            # bridge_node.send_action({"action": "reset"})
            action_code = "reset"

    # 4. 返回结果
    return ChatResponse(
        reply=reply_text,
        model_name=used_model,
        action_code=action_code,
    )


@router.post("/chat/check", response_model=CheckModelResponse)
async def check_model_connection(req: CheckModelRequest):
    """
    检查模型是否连通
    """
    print(f"正在检测模型连接: {req.model}")
    success, msg = await validate_model(req.model)
    return CheckModelResponse(success=success, message=msg)


@router.get("/chat/models", response_model=ModelListResponse)
async def get_available_models():
    """
    返回后端支持的所有模型列表
    """
    model_list = [ModelOption(**m) for m in AVAILABLE_MODELS]
    return ModelListResponse(models=model_list)
