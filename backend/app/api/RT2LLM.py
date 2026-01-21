from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session
import app.ros.bridge as ros_bridge
from app.schemas import ChatRequest, ChatResponse, CheckModelRequest, CheckModelResponse, ModelListResponse, ModelOption
from app.llm.client import ask_ai, validate_model, AVAILABLE_MODELS

router = APIRouter()


@router.post("/send", response_model=ChatResponse)
async def chat(req: ChatRequest):
    print(f"收到聊天请求，消息: {req.message}，模型: {req.model}")

    context_str = ""
    # if ros_bridge.bridge_node and ros_bridge.bridge_node.latest_state:
    #     payload = ros_bridge.bridge_node.latest_state.get("payload")
    #     if payload:
    #         context_str = f"\n(Current Robot State: {payload})"

    try:
        reply_text, used_model = ask_ai(
            text=req.message + context_str, system_prompt="You are a helpful robot assistant. You can control the robotic hand via specific commands.", model_name=req.model
        )
    except Exception as e:
        reply_text = f"Error calling AI: {str(e)}"
        used_model = "error"

    action_code = None

    # if "grasp" in reply_text.lower() or "抓" in reply_text:
    #     action_code = "grasp"
    # elif "open" in reply_text.lower() or "松" in reply_text:
    #     action_code = "open"
    # elif "reset" in reply_text.lower():
    #     action_code = "reset"
 
    return ChatResponse(
        reply=reply_text,
        model_name=used_model,
        action_code=action_code,
    )


@router.post("/check", response_model=CheckModelResponse)
async def check_model_connection(req: CheckModelRequest):
    success, msg = await validate_model(req.model)
    return CheckModelResponse(success=success, message=msg)


@router.get("/models", response_model=ModelListResponse)
async def get_available_models():
    model_list = [ModelOption(**m) for m in AVAILABLE_MODELS]
    return ModelListResponse(models=model_list)
