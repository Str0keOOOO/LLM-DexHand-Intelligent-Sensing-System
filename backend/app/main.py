from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.api import router
from contextlib import asynccontextmanager
from app.core.ros_client import start_ros_node # 导入我们刚写的模块

# 使用 lifespan 管理应用生命周期 (FastAPI 推荐方式)
@asynccontextmanager
async def lifespan(app: FastAPI):
    # 启动时：运行 ROS 节点
    print(">>> 正在启动 ROS2 桥接节点...")
    start_ros_node()
    yield
    # 关闭时：可以在这里处理清理工作 (rclpy.shutdown 会由 daemon 线程自动处理)
    print(">>> 系统关闭")

app = FastAPI(
    title="LDISS Backend System",
    description="Based on LLM & DexHand Intelligent Sensing System",
    version="1.0.0",
    lifespan=lifespan # 挂载生命周期
)

# 修改 backend/app/main.py
origins = [
    "*",  # 为了在 WSL 环境下调试方便，允许所有来源
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(router.api_router, prefix="/api")

@app.get("/")
async def root():
    return {"message": "LDISS System Backend is Running with ROS2 Bridge..."}

# uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
# python -m uvicorn app.main:app --reload --host 0.0.0.0 --port 8000