from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    PROJECT_NAME: str = "LDISS Backend"
    
    # ROS Topics
    ROS_TOPIC_SENSORS: str = "/dexhand/sensors"
    ROS_TOPIC_COMMAND: str = "/dexhand/command"

    # Database URLs (对应 docker-compose 的配置)
    # 格式: mysql+pymysql://user:pass@host:port/db
    MYSQL_DATABASE_URL: str = "mysql+pymysql://root:password@localhost:3306/dexhand_db"

    INFLUXDB_URL: str = "http://localhost:8086"
    INFLUXDB_TOKEN: str = "my-super-secret-token"
    INFLUXDB_ORG: str = "dexhand_org"
    INFLUXDB_BUCKET: str = "sensor_data"

    LLM_API_KEY: str = "your-llm-api-key"
    LLM_BASE_URL: str = "https://api.openai.com/v1"
    
    class Config:
        env_file = ".env"


settings = Settings()
