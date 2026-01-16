from pydantic import BaseModel
from typing import List, Optional


# --- 聊天相关模型 ---
class ChatMessage(BaseModel):
    role: str  # 'user' 或 'system'
    content: str


class ChatRequest(BaseModel):
    message: str  # 前端发来的最新指令


class ChatResponse(BaseModel):
    reply: str  # LLM 给用户的自然语言回复
    action_code: Optional[str] = None  # 解析出的 ROS 动作指令 (用于调试显示)


# --- 机器人状态模型 ---
class RobotStatus(BaseModel):
    timestamp: str
    fingers: List[float]  # [食指载荷, 中指载荷, 拇指载荷]
