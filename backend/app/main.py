from fastapi import FastAPI
from contextlib import asynccontextmanager
from app.api.router import api_router
from app.ros.bridge import start_ros
from app.database.mysql import engine, Base

@asynccontextmanager
async def lifespan(app: FastAPI):
    # 1. 建表 (如果表不存在)
    Base.metadata.create_all(bind=engine)
    print(">>> 数据库表结构已同步")

    # 2. 启动 ROS
    start_ros()
    
    yield

app = FastAPI(lifespan=lifespan)
app.include_router(api_router, prefix="/api")