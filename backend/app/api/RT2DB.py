from fastapi import APIRouter, Depends, Query
from sqlalchemy.orm import Session
from app.database.influx import query_api, INFLUXDB_BUCKET, org
from app.database.mysql import get_db, ChatLog
from collections import defaultdict
import re

router = APIRouter()


@router.get("/chat_history")
def get_chat_history(limit: int = 20, skip: int = 0, db: Session = Depends(get_db)):
    """获取聊天记录"""
    logs = db.query(ChatLog).order_by(ChatLog.created_at.desc()).offset(skip).limit(limit).all()
    return {"data": [{"id": log.id, "role": log.role, "content": log.content, "model": log.model, "created_at": log.created_at} for log in logs]}

# TODO 这个数据库很丑，需要重构一下
@router.get("/sensor_history")
async def get_sensor_history(
    measurement: str = Query("dexhand_joints", description="数据类型: dexhand_joints, dexhand_touch, dexhand_motor"),
    minutes: int = Query(1, description="时间范围"),
    hand: str = Query("right", pattern="^right$"),
):
    """获取传感器历史，支持结构化还原"""
    query = f"""
    from(bucket: "{INFLUXDB_BUCKET}")
      |> range(start: -{minutes}m)
      |> filter(fn: (r) => r["_measurement"] == "{measurement}")
      |> filter(fn: (r) => r["hand"] == "{hand}")
      |> aggregateWindow(every: 500ms, fn: mean, createEmpty: false)
      |> yield(name: "mean")
    """
    tables = query_api.query(query=query, org=org)

    # 按时间戳聚合数据
    grouped_data = defaultdict(dict)
    for table in tables:
        for record in table.records:
            t = record.get_time().isoformat()
            grouped_data[t][record.get_field()] = record.get_value()

    results = []
    for timestamp, fields in grouped_data.items():
        entry = {"time": timestamp, "measurement": measurement}

        # 根据不同测量类型还原数据结构
        if measurement in ["dexhand_position", "dexhand_velocity"]:
            entry["data"] = fields  # 语义关节直接是字典
        elif measurement == "dexhand_touch":
            entry["data"] = reconstruct_array(fields, 5, ["normal_force", "normal_force_delta", "tangential_force", "tangential_force_delta", "direction", "proximity", "temperature"])
        elif measurement == "dexhand_motor":
            entry["data"] = reconstruct_array(fields, 12, ["angle", "encoder_position", "current", "velocity", "error_code", "impedance"])

        results.append(entry)

    results.sort(key=lambda x: x["time"], reverse=True)
    return {"data": results}


def reconstruct_array(fields, count, keys):
    """辅助函数：将平铺的 key_0, key_1 还原为数组"""
    res = {k: [0.0] * count for k in keys}
    for f_name, val in fields.items():
        match = re.match(r"(.+)_(\d+)$", f_name)
        if match:
            k, idx = match.groups()
            if k in res and int(idx) < count:
                res[k][int(idx)] = val
    return res
