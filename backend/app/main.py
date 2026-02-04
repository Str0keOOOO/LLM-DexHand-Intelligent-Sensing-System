from dotenv import load_dotenv
from contextlib import asynccontextmanager

load_dotenv()

import asyncio
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

# 导入新增的维护函数
from app.database.mysql import init_db_with_retry, engine, Base, ChatLog, cleanup_old_chat_logs
from app.database.influx import setup_influx_retention
from app.ros.bridge import ROSBridgeManager
from app.api.router import api_router


@asynccontextmanager
async def lifespan(app: FastAPI):
    mgr = ROSBridgeManager()
    mgr.start()
    app.state.ros_bridge = mgr

    setup_influx_retention(days=7)

    async def init_and_cleanup():
        success = await init_db_with_retry()
        if success:
            loop = asyncio.get_running_loop()
            await loop.run_in_executor(None, cleanup_old_chat_logs, 30)

    asyncio.create_task(init_and_cleanup())

    yield
    print("🛑 Shutting down...")


app = FastAPI(title="LLM DexHand System", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(api_router, prefix="/api")


@app.get("/")
async def root():
    return {"message": "恭喜你，被我恭喜到了!"}
