from pydantic import BaseModel
from typing import List, Optional, Dict

# --- 1. 基础控制指令 (提前定义以供引用) ---
class ControlCommand(BaseModel):
    hand: str
    joints: Dict[str, float]

# --- 2. 聊天相关 ---
class ChatMessage(BaseModel):
    role: str
    content: str

class ChatRequest(BaseModel):
    message: str
    model: Optional[str] = None

class ChatResponse(BaseModel):
    reply: str
    model_name: str
    control_command: Optional[ControlCommand] = None

# --- 3. 模型列表与检测相关 ---
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

# --- 4. 机器人状态 ---
class RobotStatus(BaseModel):
    timestamp: float
    fingers: List[float]
    status: Optional[str] = None
    error: Optional[str] = None