from pydantic_settings import BaseSettings
from typing import Optional
from typing import List, Dict

class Settings(BaseSettings):
    PROJECT_NAME: str = "LDISS Backend"
    
    # ROS Topics
    ROS_TOPIC_SENSORS: str = "/dexhand/sensors"
    ROS_TOPIC_COMMAND: str = "/dexhand/command"

    # Database
    MYSQL_DATABASE_URL: str = "mysql+pymysql://root:password@localhost:3306/dexhand_db"

    # InfluxDB
    INFLUXDB_URL: str = "http://localhost:8086"
    INFLUXDB_TOKEN: str = "my-super-secret-token"
    INFLUXDB_ORG: str = "dexhand_org"
    INFLUXDB_BUCKET: str = "sensor_data"

    # --- LLM 配置 (扩展版) ---
    # 默认模型
    DEFAULT_LLM_MODEL: str = "gpt-3.5-turbo"

    # 1. OpenAI 官方
    OPENAI_API_KEY: Optional[str] = None

    # 2. DeepSeek / 其他 OpenAI 兼容模型 (如 Moonshot, Yi)
    DEEPSEEK_API_KEY: Optional[str] = "sk-xxxxxx"
    DEEPSEEK_BASE_URL: str = "https://api.deepseek.com"

    SILICONFLOW_API_KEY: str = "sk-ypyoicjnatynrezdbqeuwhvbbtlxgjanrcwxitnuveevqavg" 
    SILICONFLOW_BASE_URL: str = "https://api.siliconflow.cn/v1"
    
    # 3. Ollama (本地)
    OLLAMA_BASE_URL: str = "http://localhost:11434"

    # 为了兼容你之前的旧代码，保留这两个字段，但主要逻辑会用上面的
    LLM_API_KEY: str = "your-default-key"
    LLM_BASE_URL: str = "https://api.openai.com/v1"
    
    AVAILABLE_MODELS: List[Dict[str, str]] = [
        {"label": "Qwen3-8B", "value": "Qwen/Qwen3-8B"},
        # {"label": "DeepSeek (官方)", "value": "deepseek-chat"},
        # {"label": "Llama3 (本地Ollama)", "value": "llama3"},
        # {"label": "GPT-4o", "value": "gpt-4o"},
    ]
    
    class Config:
        env_file = ".env"

settings = Settings()