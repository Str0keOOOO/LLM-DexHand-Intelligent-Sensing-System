from fastapi import APIRouter
from app.schemas import ChatRequest, ChatResponse
import time

router = APIRouter()

@router.post("/send", response_model=ChatResponse)
async def chat_with_llm(request: ChatRequest):
    user_msg = request.message
    
    # --- 这里是未来接入真实大模型 (GPT/DeepSeek) 的地方 ---
    # 目前使用逻辑模拟
    
    print(f"[LLM Core] 收到指令: {user_msg}")
    
    # 模拟处理延迟
    # time.sleep(1) 
    
    if "抓" in user_msg or "拿" in user_msg:
        return ChatResponse(
            reply=f"收到，正在规划抓取动作。已识别目标物体，即将执行力位混合控制。",
            action_code="ROS2_Action: DexHand_Grasp(force=5.0N, target='cube')"
        )
    elif "停止" in user_msg:
        return ChatResponse(
            reply="紧急指令已确认！系统急停中...",
            action_code="ROS2_Service: /emergency_stop"
        )
    else:
        return ChatResponse(
            reply=f"LLM 已收到您的消息: '{user_msg}'。请下达具体的采集或控制指令。",
            action_code="Status: IDLE"
        )