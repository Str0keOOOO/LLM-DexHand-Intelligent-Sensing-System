import os
from influxdb_client import InfluxDBClient
from influxdb_client.client.write_api import WriteOptions

# 读取 .env 配置
url = os.getenv("INFLUXDB_URL")
token = os.getenv("INFLUXDB_TOKEN")
org = os.getenv("INFLUXDB_ORG")
INFLUXDB_BUCKET = os.getenv("INFLUXDB_BUCKET")

client = InfluxDBClient(url=url, token=token, org=org)

_write_options = WriteOptions(batch_size=500, flush_interval=1000, jitter_interval=200, retry_interval=5000)

write_api = client.write_api(write_options=_write_options)
query_api = client.query_api()
