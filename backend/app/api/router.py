from fastapi import APIRouter
from app.api.RT2LLM import router as llm_router
from app.api.RT2Robot import router as robot_router
from app.api.RT2DB import router as db_router
from app.api.RT2Arm import router as arm_router

api_router = APIRouter()

api_router.include_router(llm_router, prefix="/chat", tags=["chat"])
api_router.include_router(robot_router, prefix="/ros_ws", tags=["ros_ws"])
api_router.include_router(db_router, prefix="/data_base", tags=["data_base"])
api_router.include_router(arm_router, prefix="/robotic_arm", tags=["robotic_arm"])
