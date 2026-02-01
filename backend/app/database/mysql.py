import os
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy import Column, Integer, String, Text, DateTime, func


# 读取配置
SQLALCHEMY_DATABASE_URL = os.getenv("MYSQL_DATABASE_URL")

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
    session_id = Column(String(50), index=True, nullable=True)
    role = Column(String(20), nullable=False)
    content = Column(Text, nullable=False)
    model = Column(String(50), nullable=True)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
