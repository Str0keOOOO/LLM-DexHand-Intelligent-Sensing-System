from fastapi import APIRouter
from app.api.endpoints import router as endpoints_router

api_router = APIRouter()

# ✅ 正确写法（直接包含，由 endpoints.py 自己定义完整路径）：
api_router.include_router(endpoints_router)