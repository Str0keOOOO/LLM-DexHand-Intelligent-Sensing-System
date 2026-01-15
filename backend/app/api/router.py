from fastapi import APIRouter
from ..api.endpoints import chat, robot

api_router = APIRouter()

# 注册子路由
api_router.include_router(chat.router, prefix="/chat", tags=["LLM Interaction"])
api_router.include_router(robot.router, prefix="/robot", tags=["Robot Data"])