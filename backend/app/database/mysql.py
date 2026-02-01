import os
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy import Column, Integer, String, Text, DateTime, func


# 读取配置
SQLALCHEMY_DATABASE_URL = os.getenv("MYSQL_DATABASE_URL")

# pool_recycle 防止连接因为超时被 MySQL 断开
# pool_size 控制并发数
engine = create_engine(SQLALCHEMY_DATABASE_URL, pool_recycle=3600, pool_size=20, max_overflow=0)

SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base = declarative_base()


def get_db():
    db = SessionLocal()
    try:
        yield db    
    finally:
        db.close()


class ChatLog(Base):
    __tablename__ = "chat_logs"

    id = Column(Integer, primary_key=True, index=True)
    # 会话ID，方便未来把一次聊天的多轮对话串起来
    session_id = Column(String(50), index=True, nullable=True)
    # 角色：'user' (你) 或 'assistant' (机器人)
    role = Column(String(20), nullable=False)
    # 聊天内容
    content = Column(Text, nullable=False)
    # 模型名称 (比如 Qwen-7B)
    model = Column(String(50), nullable=True)
    # 自动记录时间
    created_at = Column(DateTime(timezone=True), server_default=func.now())
