from pydantic import BaseModel, Field
from typing import List, Optional, Dict


# --- 1. 基础 ---
class SuccessResponse(BaseModel):
    success: bool


# --- 2. 灵巧手 ---
class HandJointsCommand(BaseModel):
    th_dip: float
    th_mcp: float
    th_rot: float
    ff_spr: float
    ff_dip: float
    ff_mcp: float
    mf_dip: float
    mf_mcp: float
    rf_dip: float
    rf_mcp: float
    lf_dip: float
    lf_mcp: float


# --- 3. 机械臂 ---
class ArmJointsCommand(BaseModel):
    ref: int = 4
    nb: int
    dir: int
    vel: float
    acc: float
    max_dis: float


# --- 4. LLM ---
class CheckModelRequest(BaseModel):
    model: str


class ModelOption(BaseModel):
    label: str
    value: str


class ModelListResponse(BaseModel):
    models: List[ModelOption]


class ChatRequest(BaseModel):
    message: str
    model: Optional[str] = None


class ChatResponse(BaseModel):
    reply: str
    model_name: str
    
