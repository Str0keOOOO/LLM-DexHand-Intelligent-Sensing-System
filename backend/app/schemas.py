from pydantic import BaseModel
from typing import List, Optional, Dict, Any


# --- 1. 聊天相关 ---
class ChatMessage(BaseModel):
    role: str
    content: str


class ChatRequest(BaseModel):
    message: str
    model: Optional[str] = None  # 允许为空，后端会有默认值


class ChatResponse(BaseModel):
    reply: str
    model_name: str
    action_code: Optional[str] = None


# --- 2. 模型列表与检测相关 (最关键的部分) ---
class ModelOption(BaseModel):
    label: str
    value: str


class ModelListResponse(BaseModel):
    models: List[ModelOption]


class CheckModelRequest(BaseModel):
    model: str


class CheckModelResponse(BaseModel):
    success: bool
    message: str


# --- 3. 机器人状态 ---
class RobotStatus(BaseModel):
    timestamp: float
    fingers: List[float]
    status: Optional[str] = None
    error: Optional[str] = None


class ControlCommand(BaseModel):
    hand: str
    joints: Dict[str, float]
