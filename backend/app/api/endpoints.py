import time
import math
import random
import json
import asyncio

from fastapi import APIRouter, WebSocket, WebSocketDisconnect, Depends
from sqlalchemy.orm import Session

# [修复点 1] 改变导入方式，不要直接导入变量，而是导入模块
# 这样可以确保我们在运行时获取到最新的 bridge_node 单例
import app.ros.bridge as ros_bridge

from app.database.mysql import get_db
from app.schemas import ChatRequest, ChatResponse, CheckModelRequest, CheckModelResponse, ModelListResponse, ModelOption
from app.llm.client import ask_ai, validate_model, AVAILABLE_MODELS

router = APIRouter()

# ==========================================
# 1. 系统基础与 WebSocket 接口
# ==========================================

@router.get("/health")
async def health_check():
    # 动态检查模块中的变量
    is_connected = ros_bridge.bridge_node is not None
    status_data = {"status": "ok", "ros_bridge": "connected" if is_connected else "initializing"}
    return status_data

@router.websocket("/ws/robot-data")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    start_time = time.time()
    
    try:
        while True:
            # [修复点 2] 动态获取当前的 bridge_node
            current_bridge = ros_bridge.bridge_node
            
            # 准备要发送的数据
            final_data = None

            # 情况 A: ROS Bridge 已启动
            if current_bridge:
                # 获取 ROS 传来的最新数据
                data = current_bridge.latest_state
                
                # 如果 ROS 还没发来有效载荷 (payload 为 None)，后端自己生成模拟数据兜底
                if data.get("payload") is None:
                    t = time.time() - start_time
                    mock_payload = {
                        "joints": {
                            "th_rot": 30 + 10 * math.sin(t), 
                            "ff_mcp": 45 + 15 * math.cos(t), 
                            "lf_mcp": 10 + 5 * math.sin(t * 2)
                        },
                        "touch": {
                            "th": abs(2 * math.sin(t)), 
                            "ff": abs(2 * math.cos(t))
                        }
                    }
                    final_data = {
                        "mode": "BACKEND_MOCK", # 标记这是后端生成的模拟数据
                        "timestamp": time.time(),
                        "payload": mock_payload
                    }
                else:
                    # ROS 有数据，直接转发
                    final_data = data
            
            # 情况 B: ROS Bridge 甚至还没初始化 (Main 还没跑完 start_ros_bridge)
            else:
                # 同样生成模拟数据，保证前端不白屏
                t = time.time() - start_time
                mock_payload = {
                    "joints": {"th_rot": 0, "ff_mcp": 0, "lf_mcp": 0}, # 归零或动起来都可以
                    "touch": {"th": 0, "ff": 0}
                }
                final_data = {
                    "mode": "BACKEND_INIT_MOCK",
                    "timestamp": time.time(),
                    "payload": mock_payload
                }

            # 发送数据
            await websocket.send_text(json.dumps(final_data))
            
            # 10Hz 刷新率
            await asyncio.sleep(0.1)

    except WebSocketDisconnect:
        print("WS Client disconnected")
    except Exception as e:
        print(f"WS Error: {e}")

# ==========================================
# 2. LLM 对话与控制接口
# ==========================================

@router.post("/chat/send", response_model=ChatResponse)
async def chat(req: ChatRequest, db: Session = Depends(get_db)):
    print(f"收到聊天请求，消息: {req.message}，模型: {req.model}")

    context_str = ""
    # 同样使用动态访问
    if ros_bridge.bridge_node and ros_bridge.bridge_node.latest_state:
        payload = ros_bridge.bridge_node.latest_state.get("payload")
        if payload:
            context_str = f"\n(Current Robot State: {payload})"

    try:
        reply_text, used_model = ask_ai(
            text=req.message + context_str, 
            system_prompt="You are a helpful robot assistant. You can control the robotic hand via specific commands.", 
            model_name=req.model
        )
    except Exception as e:
        reply_text = f"Error calling AI: {str(e)}"
        used_model = "error"

    action_code = None
    if "grasp" in reply_text.lower() or "抓" in reply_text:
        action_code = "grasp"
    elif "open" in reply_text.lower() or "松" in reply_text:
        action_code = "open"
    elif "reset" in reply_text.lower():
        action_code = "reset"

    return ChatResponse(
        reply=reply_text,
        model_name=used_model,
        action_code=action_code,
    )

@router.post("/chat/check", response_model=CheckModelResponse)
async def check_model_connection(req: CheckModelRequest):
    print(f"正在检测模型连接: {req.model}")
    success, msg = await validate_model(req.model)
    return CheckModelResponse(success=success, message=msg)

@router.get("/chat/models", response_model=ModelListResponse)
async def get_available_models():
    model_list = [ModelOption(**m) for m in AVAILABLE_MODELS]
    return ModelListResponse(models=model_list)