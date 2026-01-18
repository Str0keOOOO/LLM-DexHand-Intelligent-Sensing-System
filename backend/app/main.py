from dotenv import load_dotenv
load_dotenv()

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.api.router import api_router

# [修复点 1] 导入新的函数名 start_ros_bridge
from app.ros.bridge import start_ros_bridge 

app = FastAPI(title="LLM DexHand System")

# CORS 设置 (允许前端访问)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(api_router, prefix="/api")

# [修复点 2] 在启动事件中调用正确的函数
@app.on_event("startup")
async def startup_event():
    print("正在启动 ROS Bridge...")
    start_ros_bridge()

@app.get("/")
def read_root():
    return {"message": "Welcome to LLM DexHand System Backend"}