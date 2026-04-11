from fastapi import APIRouter, Depends, Query
from sqlalchemy.orm import Session
from app.database.influx import query_api, INFLUXDB_BUCKET, org
from app.database.mysql import get_db, ChatLog
from collections import defaultdict

router = APIRouter()


@router.get("/chat_history")
def get_chat_history(limit: int = 20, skip: int = 0, db: Session = Depends(get_db)):
    """获取 MySQL 中存储的聊天记录"""
    logs = db.query(ChatLog).order_by(ChatLog.created_at.desc()).offset(skip).limit(limit).all()

    return {
        "success": True,
        "data": [
            {
                "id": log.id,
                "role": log.role,
                "content": log.content,
                "model": log.model,
                "timestamp": int(log.created_at.timestamp()) if log.created_at else None,
            }
            for log in logs
        ],
    }


@router.get("/sensor_history")
async def get_sensor_history(
    target: str = Query("hand", description="查询目标: hand 或 arm"),
    minutes: int = Query(1, description="时间范围(分钟)"),
):
    """从 InfluxDB 获取历史数据，并强制执行机械臂字段顺序"""
    measurement = "dexhand_joints" if target == "hand" else "arm_joints"
    
    query = f"""
    from(bucket: "{INFLUXDB_BUCKET}")
      |> range(start: -{minutes}m)
      |> filter(fn: (r) => r["_measurement"] == "{measurement}")
      |> aggregateWindow(every: 500ms, fn: mean, createEmpty: false)
      |> yield(name: "mean")
    """

    tables = query_api.query(query=query, org=org)
    grouped_data = defaultdict(dict)
    
    for table in tables:
        for record in table.records:
            t = record.get_time()
            ts = int(t.timestamp()) if t else None
            if ts is None: continue
            grouped_data[ts][record.get_field()] = record.get_value()

    results = []
    # 核心修复：定义机械臂期望的显示顺序（支持大小写）
    arm_key_order = ["x", "y", "z", "rx", "ry", "rz", "Rx", "Ry", "Rz"]

    for ts, fields in grouped_data.items():
        ordered_fields = {}
        if target == "arm":
            # 1. 按照预设顺序提取已知物理字段
            for k in arm_key_order:
                if k in fields:
                    ordered_fields[k] = fields[k]
            # 2. 提取其余字段并过滤掉冗余字段
            for k, v in fields.items():
                if k not in arm_key_order and k not in ["timestamp", "success"]:
                    ordered_fields[k] = v
        else:
            # 灵巧手保持默认排序
            ordered_fields = dict(sorted(fields.items()))

        results.append({
            "target": target,
            "data": ordered_fields,
            "timestamp": ts,
        })

    # 按时间戳从新到旧排序
    results.sort(key=lambda x: x["timestamp"], reverse=True)
    return {"success": True, "data": results}