from sqlalchemy import Column, Integer, String, Text, DateTime
from sqlalchemy.sql import func
from app.database.mysql import Base


class ChatLog(Base):
    __tablename__ = "chat_logs"

    id = Column(Integer, primary_key=True, index=True)
    role = Column(String(50))  # 'user' 或 'ai'
    content = Column(Text)  # 聊天内容
    action = Column(String(255))  # 如果有动作指令，存这里
    created_at = Column(DateTime(timezone=True), server_default=func.now())
