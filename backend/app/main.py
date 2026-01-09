from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.api import router

app = FastAPI(
    title="LDISS Backend System",
    description="Based on LLM & DexHand Intelligent Sensing System",
    version="1.0.0"
)

# --- 关键：配置跨域，允许前端 Vue 访问 ---
origins = [
    "http://localhost:5173",  # 前端开发地址
    "http://127.0.0.1:5173",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 挂载路由模块
app.include_router(router.api_router, prefix="/api")

@app.get("/")
async def root():
    return {"message": "LDISS System Backend is Running..."}