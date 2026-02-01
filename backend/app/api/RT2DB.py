from fastapi import APIRouter, Depends, Query  # 新增依赖
from sqlalchemy.orm import Session  # 新增
from typing import List, Optional
from app.database.influx import query_api, INFLUXDB_BUCKET, org
from app.database.mysql import get_db, ChatLog  # 新增


router = APIRouter()

# --- 新增接口：获取聊天历史 ---
@router.get("/chat_history")
def get_chat_history(
    limit: int = 20, 
    skip: int = 0, 
    db: Session = Depends(get_db)
):
    """
    分页获取聊天记录，按时间倒序排列
    """
    logs = db.query(ChatLog).order_by(ChatLog.created_at.desc()).offset(skip).limit(limit).all()
    
    # 将 ORM 对象转换为简单的字典列表返回
    results = []
    for log in logs:
        results.append({
            "id": log.id,
            "role": log.role,
            "content": log.content,
            "model": log.model,
            "created_at": log.created_at
        })
    return {"data": results}

# --- 原有的 InfluxDB 传感器查询接口 (保持不变) ---
@router.get("/sensor_history")
async def get_sensor_history(
    minutes: int = Query(1, description="Time range in minutes"), 
    hand: str = Query("right", regex="^(left|right)$")
):
    """
    获取指定手部的触觉传感器历史数据
    """
    query = f"""
    from(bucket: "{INFLUXDB_BUCKET}")
      |> range(start: -{minutes}m)
      |> filter(fn: (r) => r["_measurement"] == "dexhand_touch")
      |> filter(fn: (r) => r["hand"] == "{hand}")
      |> aggregateWindow(every: 500ms, fn: mean, createEmpty: false)
      |> yield(name: "mean")
    """

    tables = query_api.query(query=query, org=org)

    results = []
    for table in tables:
        for record in table.records:
            results.append(
                {
                    "time": record.get_time(),
                    "field": record.get_field(),
                    "value": record.get_value(),
                }
            )

    return {"data": results}