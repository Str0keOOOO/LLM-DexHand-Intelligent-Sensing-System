import os
import asyncio
import logging
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy import Column, Integer, String, Text, DateTime, func
from sqlalchemy.exc import OperationalError
from datetime import datetime, timedelta

logger = logging.getLogger(__name__)


def _build_mysql_url() -> str:
    user = os.getenv("DB_USER", "root")
    password = os.getenv("DB_PASSWORD", "rootpassword")
    host = os.getenv("DB_HOST", "localhost")
    port = os.getenv("DB_PORT", "3306")
    dbname = os.getenv("DB_NAME", "dexhand_db")

    return f"mysql+pymysql://{user}:{password}@{host}:{port}/{dbname}?charset=utf8mb4"


SQLALCHEMY_DATABASE_URL = _build_mysql_url()
engine = create_engine(SQLALCHEMY_DATABASE_URL, pool_recycle=3600, pool_pre_ping=True, connect_args={"connect_timeout": 5})
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


async def init_db_with_retry():
    """尝试初始化数据库，失败则等待并重试"""
    max_retries = 100
    retry_interval = 5

    for attempt in range(1, max_retries + 1):
        try:
            loop = asyncio.get_running_loop()
            await loop.run_in_executor(None, lambda: Base.metadata.create_all(bind=engine))
            logger.info("✅ Database tables created successfully.")
            return True
        except OperationalError as e:
            logger.error(f"❌ Database connection failed (Attempt {attempt}/{max_retries}): {e}")
            if attempt < max_retries:
                await asyncio.sleep(retry_interval)
            else:
                logger.critical("🔥 Could not connect to database after maximum retries.")
    return False


def cleanup_old_chat_logs(days_to_keep: int = 30):
    """同步清理函数，由 lifespan 或 background task 调用"""
    db = SessionLocal()
    try:
        threshold_date = datetime.now() - timedelta(days=days_to_keep)
        deleted_count = db.query(ChatLog).filter(ChatLog.created_at < threshold_date).delete()
        db.commit()
        if deleted_count > 0:
            logger.info(f"🧹 MySQL Cleanup: Deleted {deleted_count} old chat logs.")
    except Exception as e:
        db.rollback()
        logger.error(f"❌ MySQL Cleanup Failed: {e}")
    finally:
        db.close()
