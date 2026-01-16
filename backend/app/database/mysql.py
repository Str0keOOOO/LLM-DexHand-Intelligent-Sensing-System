from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker, declarative_base
from app.config import settings

# 1. 创建引擎
engine = create_engine(
    settings.MYSQL_DATABASE_URL,
    pool_pre_ping=True,  # 自动重连
    pool_size=10,
)

# 2. 创建会话工厂
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# 3. 模型基类
Base = declarative_base()


# 4. 依赖注入函数 (给 API 用)
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()
