from fastapi import APIRouter
from app.api.RT2LLM import router as llm_router
from app.api.RT2Hand import router as hand_router
from app.api.RT2DB import router as db_router
from app.api.RT2Arm import router as arm_router

api_router = APIRouter()

api_router.include_router(llm_router, prefix="/llm", tags=["chat"])
api_router.include_router(hand_router, prefix="/hand", tags=["hand"])
api_router.include_router(db_router, prefix="/db", tags=["data_base"])
api_router.include_router(arm_router, prefix="/arm", tags=["robotic_arm"])
