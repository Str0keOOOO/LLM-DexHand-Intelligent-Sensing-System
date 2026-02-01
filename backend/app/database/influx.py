import os
from influxdb_client import InfluxDBClient
from influxdb_client.client.write_api import WriteOptions

# 读取 .env 配置
url = os.getenv("INFLUXDB_URL", "http://localhost:8086")
token = os.getenv("INFLUXDB_TOKEN", "my-super-secret-token")
org = os.getenv("INFLUXDB_ORG", "dexhand_org")
INFLUXDB_BUCKET = os.getenv("INFLUXDB_BUCKET", "sensor_data")

client = InfluxDBClient(url=url, token=token, org=org)

# --- 核心修改：启用异步批处理 ---
# 这一步至关重要！否则高频写入会卡死 ROS 回调
_write_options = WriteOptions(
    batch_size=500,       # 攒够 500 条数据才发一次网络请求
    flush_interval=1000,  # 或者每隔 1000ms 发一次
    jitter_interval=200,  # 随机抖动，避免瞬间网络拥堵
    retry_interval=5000   # 失败重试间隔
)

# 获取“非阻塞”的写入 API
write_api = client.write_api(write_options=_write_options)
query_api = client.query_api()