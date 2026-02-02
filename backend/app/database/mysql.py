import os
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy import Column, Integer, String, Text, DateTime, func


def _build_mysql_url() -> str:
    # 获取环境变量，提供本地开发默认值
    user = os.getenv("DB_USER", "root")
    password = os.getenv("DB_PASSWORD", "rootpassword")
    host = os.getenv("DB_HOST", "localhost")
    port = os.getenv("DB_PORT", "3306")
    dbname = os.getenv("DB_NAME", "dexhand_db")

    return f"mysql+pymysql://{user}:{password}@{host}:{port}/{dbname}?charset=utf8mb4"


SQLALCHEMY_DATABASE_URL = _build_mysql_url()
engine = create_engine(SQLALCHEMY_DATABASE_URL, pool_recycle=3600)
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
    session_id = Column(String(50), index=True)
    role = Column(String(20), nullable=False)
    content = Column(Text, nullable=False)
    model = Column(String(50))
    created_at = Column(DateTime(timezone=True), server_default=func.now())
