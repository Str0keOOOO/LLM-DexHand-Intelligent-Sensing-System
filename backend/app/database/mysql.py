import os
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.ext.declarative import declarative_base

# 直接读取
SQLALCHEMY_DATABASE_URL = os.getenv(
    "MYSQL_DATABASE_URL", 
    "mysql+pymysql://root:password@localhost:3306/dexhand_db"
)

engine = create_engine(SQLALCHEMY_DATABASE_URL)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base = declarative_base()

def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()