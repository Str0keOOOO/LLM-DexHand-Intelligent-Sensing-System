from fastapi import FastAPI, APIRouter
from fastapi.middleware.cors import CORSMiddleware
from app.api.robot_routes import router as robot_router


clientRobot = None

app = FastAPI(title="Robotic Arm API")

router = APIRouter()


app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(robot_router, prefix="/api", tags=["robotic-arm"])
