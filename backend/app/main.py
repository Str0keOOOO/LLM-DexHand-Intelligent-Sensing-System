from dotenv import load_dotenv
from contextlib import asynccontextmanager

load_dotenv()


from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from app.api.router import api_router
from app.ros.bridge import ROSBridgeManager
from app.database.mysql import engine, Base, ChatLog


@asynccontextmanager
async def lifespan(app: FastAPI):
    mgr = ROSBridgeManager()
    mgr.start()

    app.state.ros_bridge = mgr
    print("✅ ROS Bridge Manager started and attached to app.state")

    yield

    print("🛑 Shutting down...")


app = FastAPI(title="LLM DexHand System", lifespan=lifespan)

Base.metadata.create_all(bind=engine)

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
