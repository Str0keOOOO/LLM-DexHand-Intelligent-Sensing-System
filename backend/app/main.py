import os
from dotenv import load_dotenv

# 1. 必须在所有其他 import 之前加载环境变量
load_dotenv()

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
from app.api.router import api_router
from app.ros.bridge import start_ros
from app.database.mysql import engine, Base

@asynccontextmanager
async def lifespan(app: FastAPI):
    # --- [修改] 注释掉数据库建表操作 ---
    # 暂时跳过数据库连接，防止因 MySQL 未启动导致报错
    # Base.metadata.create_all(bind=engine)
    # print(">>> 数据库表结构已同步")
    # ----------------------------------

    # 2. 启动 ROS
    start_ros()
    
    yield

app = FastAPI(title=os.getenv("PROJECT_NAME", "LDISS Backend"))

app = FastAPI(lifespan=lifespan)

# CORS 配置保持不变
origins = [
    "http://localhost",
    "http://localhost:5173",
    "http://localhost:8080",
    "*"
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(api_router, prefix="/api")

@app.get("/")
def root():
    return {"message": "LLM-DexHand Intelligent Sensing System Backend is Running"}

# conda activate ldiss
# python -m uvicorn app.main:app --reload --host 0.0.0.0 --port 8000