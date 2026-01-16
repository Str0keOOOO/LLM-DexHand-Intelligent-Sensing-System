from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session
from app.schemas import ChatRequest

# 导入四大模块
from app.llm.client import ask_ai  # Brain
from app.ros.bridge import ros_node  # Hand
from app.database.mysql import get_db  # Memory (SQL)
from app.database.models import ChatLog  # Memory (Model)
# InfluxDB 通常在 ROS 回调里自动存，或者在这里手动存

router = APIRouter()


@router.post("/chat")
async def chat(req: ChatRequest, db: Session = Depends(get_db)):
    # 1. [DB] 先把用户的存下来
    db.add(ChatLog(role="user", content=req.message))
    db.commit()

    # 2. [LLM] 思考
    reply = ask_ai(req.message, "你是一个机器人...")

    # 3. [ROS] 执行 (如果需要)
    if "grasp" in reply and ros_node:
        ros_node.send_action({"action": "grasp"})

    # 4. [DB] 把 AI 的回复也存下来
    db.add(ChatLog(role="ai", content=reply))
    db.commit()

    return {"reply": reply}
