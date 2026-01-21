from fastapi import APIRouter
from app.api.RT2LLM import router as llm_router
from app.api.RT2Robot import router as robot_router

api_router = APIRouter()

api_router.include_router(llm_router, prefix="/chat", tags=["chat"])
api_router.include_router(robot_router, prefix="/ros_ws", tags=["ros_ws"])
