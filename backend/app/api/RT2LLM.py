import json
import re
from fastapi import APIRouter, Request, Depends
from sqlalchemy.orm import Session
from app.schemas import ChatRequest, ChatResponse, CheckModelRequest, CheckModelResponse, ModelListResponse, ModelOption, ControlCommand
from app.llm.client import ask_ai, validate_model, AVAILABLE_MODELS
from app.ros.bridge import ROSBridgeManager
from app.database.mysql import get_db, ChatLog

router = APIRouter()


def extract_json(text: str):
    """尝试从 LLM 回复中提取 JSON 块"""
    try:
        match = re.search(r"\{.*\}", text, re.DOTALL)
        if match:
            return json.loads(match.group())
        return None
    except (TypeError, json.JSONDecodeError, re.error):
        return None


@router.post("/send", response_model=ChatResponse)
async def chat(req: ChatRequest, request: Request, db: Session = Depends(get_db)):
    bridge: ROSBridgeManager | None = getattr(request.app.state, "ros_bridge", None)
    context_str = ""
    if bridge:
        state = bridge.get_latest_state()
        simple_state = {k: v.get("joints", {}) for k, v in state.items() if k in ["left", "right"]}
        context_str = f"\n[Current Robot Joint State]: {json.dumps(simple_state)}"

    system_prompt = (
        "You are an intelligent robot assistant for DexHand. "
        "You can control the robot by outputting a JSON command."
        "\n\nRULES:"
        "\n1. If the user asks for a physical action (e.g., 'open hand', 'grasp'), output a JSON object with 'reply' and 'command' fields."
        "\n2. The 'command' field must have: 'hand' ('left' or 'right') and 'joints' (a dictionary of joint_name: angle)."
        "\n3. If no action is needed, 'command' should be null."
        '\n4. Valid format example: {{"reply": "Opening the hand now.", "command": {{"hand": "right", "joints": {{"thumb_flexion": 0.0, "index_flexion": 0.0}}}}}}'
        "\n5. Always ensure the JSON is valid."
    )

    print(f"Chat Request: {req.message}")

    try:
        raw_response, used_model = await ask_ai(text=req.message + context_str, system_prompt=system_prompt, model_name=req.model)

        json_data = extract_json(raw_response)

        control_cmd = None
        reply_text = raw_response

        if json_data and isinstance(json_data, dict):
            reply_text = json_data.get("reply", raw_response)
            cmd_data = json_data.get("command")

            if cmd_data:
                try:
                    control_cmd = ControlCommand(**cmd_data)
                except Exception as e:
                    reply_text += f"\n(Internal Error: Invalid command format generated): {str(e)}"

        try:
            user_log = ChatLog(role="user", content=req.message, model=used_model)
            db.add(user_log)

            full_content = reply_text
            if control_cmd:
                cmd_dict = control_cmd.model_dump()
                cmd_str = json.dumps(cmd_dict, ensure_ascii=False, indent=2)
                full_content += f"\n\n[Generated Command]:\n{cmd_str}"

            assistant_log = ChatLog(role="assistant", content=full_content, model=used_model)
            db.add(assistant_log)

            db.commit()
        except Exception as db_e:
            print(f"Database Error: {db_e}")
            db.rollback()

    except Exception as e:
        reply_text = f"Error calling AI: {str(e)}"
        used_model = "error"
        control_cmd = None

    return ChatResponse(
        reply=reply_text,
        model_name=used_model,
        control_command=control_cmd,
    )


@router.post("/check", response_model=CheckModelResponse)
async def check_model_connection(req: CheckModelRequest):
    success, msg = await validate_model(req.model)
    return CheckModelResponse(success=success, message=msg)


@router.get("/models", response_model=ModelListResponse)
async def get_available_models():
    model_list = [ModelOption(**m) for m in AVAILABLE_MODELS]
    return ModelListResponse(models=model_list)
