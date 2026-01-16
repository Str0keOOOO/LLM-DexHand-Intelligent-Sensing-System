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