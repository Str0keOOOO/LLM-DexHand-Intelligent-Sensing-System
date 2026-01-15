from fastapi import APIRouter
from ...schemas import ChatRequest, ChatResponse
import time

router = APIRouter()

@router.post("/send", response_model=ChatResponse)
async def chat_with_llm(request: ChatRequest):
    user_msg = request.message
    
    # --- 这里是未来接入真实大模型 (GPT/DeepSeek) 的地方 ---
    # 目前使用逻辑模拟
    
    print(f"[LLM Core] 收到指令: {user_msg}")
    return ChatResponse(
            reply=f"我现在很笨但我知道你说的是“{user_msg}”。",
            action_code="ROS2_Action: DexHand_Grasp(force=5.0N, target='cube')"
        )