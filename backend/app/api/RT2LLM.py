import json
import time
import math
from fastapi import APIRouter, Request, Depends, HTTPException
from sqlalchemy.orm import Session
from app.schemas import ChatRequest, ChatResponse, CheckModelRequest, ModelListResponse, ModelOption, SuccessResponse, HandJointsCommand, ArmJointsCommand
from app.llm.client import ask_ai, validate_model, AVAILABLE_MODELS, extract_json
from app.ros.bridge import ROSBridgeManager
from app.robotic_arm.bridge import _request as arm_request
from app.database.mysql import get_db, ChatLog

router = APIRouter()


@router.post("/send")
async def chat(request: Request, req: ChatRequest, db: Session = Depends(get_db)) -> dict:
    bridge: ROSBridgeManager | None = getattr(request.app.state, "ros_bridge", None)

    # 1. 获取灵巧手的状态
    hand_state_str = "{}"
    if bridge:
        state = bridge.get_latest_state()
        simple_state = {k: v.get("joints", {}) for k, v in state.items() if k in ["left", "right"]}
        hand_state_str = json.dumps(simple_state)

    # 2. 获取机械臂的状态
    arm_state_str = "{}"
    try:
        arm_state = await arm_request("GET", "/api/pos")
        arm_state_str = json.dumps(arm_state)
    except Exception as e:
        arm_state_str = f"Unknown (Error: {str(e)})"

    context_str = f"\n[Current Hand Joint State]: {hand_state_str}\n[Current Arm State]: {arm_state_str}"

    system_prompt = (
        "You are an intelligent robot assistant for DexHand and a Robotic Arm. "
        "You can control the robots by outputting a JSON command."
        "\n\nRULES:"
        "\n1. If the user asks for a physical action, output a JSON object with 'reply', and optionally 'hand_command' and/or 'arm_command' fields."
        "\n2. The 'hand_command' field must have the following keys (floats in degrees): th_dip, th_mcp, th_rot, ff_spr, ff_dip, ff_mcp, mf_dip, mf_mcp, rf_dip, rf_mcp, lf_dip, lf_mcp."
        "\n3. The 'arm_command' field must have: 'nb' (axis: 'x', 'y', 'z', 'Rx', 'Ry', 'Rz'), 'dir' ('positive' or 'negative'), 'vel' (float), 'acc' (float), 'max_dis' (float)."
        "\n4. If no action is needed, omit 'hand_command' and 'arm_command'."
        '\n5. Valid format example: {"reply": "Moving arm and hand.", "hand_command": {"th_dip": 0.0, "th_mcp": 0.0, "th_rot": 0.0, "ff_spr": 0.0, "ff_dip": 0.0, "ff_mcp": 0.0, "mf_dip": 0.0, "mf_mcp": 0.0, "rf_dip": 0.0, "rf_mcp": 0.0, "lf_dip": 0.0, "lf_mcp": 0.0}, "arm_command": {"nb": "x", "dir": "positive", "vel": 20.0, "acc": 20.0, "max_dis": 30.0}}'
        "\n6. Always ensure the JSON is valid."
    )

    print(f"Chat Request: {req.message}")

    # 保存用户的提问到 MySQL
    user_log = ChatLog(
        session_id="default_session",
        role="user",
        content=req.message,
        model=req.model or "default",
    )
    db.add(user_log)
    db.commit()

    used_model = req.model or "default"
    reply_text = ""
    try:
        raw_response, used_model = await ask_ai(
            text=req.message + context_str,
            system_prompt=system_prompt,
            model_name=req.model,
        )

        json_data = extract_json(raw_response)

        reply_text = raw_response
        executed_commands = {}

        if json_data and isinstance(json_data, dict):
            reply_text = json_data.get("reply", raw_response)
            hand_cmd_data = json_data.get("hand_command")
            arm_cmd_data = json_data.get("arm_command")

            # --- 解析和处理灵巧手指令 ---
            if hand_cmd_data:
                try:
                    hand_cmd = HandJointsCommand(**hand_cmd_data)
                    executed_commands["hand"] = hand_cmd.model_dump()

                    if bridge and bridge.is_started():
                        joints_deg = hand_cmd.model_dump()
                        joints_rad = {k: (float(v) * math.pi / 180.0) for k, v in joints_deg.items()}
                        bridge.send_command(joints_rad)
                    else:
                        reply_text += "\n(Warning: Hand ROS Bridge not initialized)"
                except Exception as e:
                    reply_text += f"\n(Hand Command Error: {str(e)})"

            # --- 解析和处理机械臂指令 ---
            if arm_cmd_data:
                try:
                    nb_mapping = {"x": 1, "y": 2, "z": 3, "Rx": 4, "Ry": 5, "Rz": 6}
                    dir_mapping = {"positive": 1, "negative": 0}

                    arm_cmd_data["nb"] = nb_mapping.get(arm_cmd_data.get("nb"), 1)
                    arm_cmd_data["dir"] = dir_mapping.get(arm_cmd_data.get("dir"), 1)

                    arm_cmd = ArmJointsCommand(**arm_cmd_data)
                    executed_commands["arm"] = arm_cmd.model_dump()

                    await arm_request("POST", "/api/move", json=arm_cmd.model_dump())
                except Exception as e:
                    reply_text += f"\n(Arm Command Error: {str(e)})"

            if executed_commands:
                print("[Control Commands]", json.dumps(executed_commands, ensure_ascii=False))

    except Exception as e:
        reply_text = f"Error calling AI: {str(e)}"
        used_model = "error"

    # 保存 AI 的回复到 MySQL
    ai_log = ChatLog(session_id="default_session", role="assistant", content=reply_text, model=used_model)
    db.add(ai_log)
    db.commit()

    return {
        "success": True,
        "data": {
            "reply": reply_text,
            "model_name": used_model,
            "timestamp": int(time.time()),
        },
    }


@router.post("/check")
async def check_model_connection(req: CheckModelRequest) -> SuccessResponse:
    try:
        success, _ = await validate_model(req.model)
        if success:
            return SuccessResponse(success=True)
        else:
            raise HTTPException(status_code=400, detail=f"Model '{req.model}' validation failed.")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error validating model '{req.model}': {str(e)}") from e


@router.get("/list")
async def get_available_models() -> dict:
    model_list = [ModelOption(**m) for m in AVAILABLE_MODELS]
    return {
        "success": True,
        "data": {
            "model": [m.model_dump() for m in model_list],
            "timestamp": int(time.time()),
        },
    }
