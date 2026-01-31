# import os
# from influxdb_client import InfluxDBClient
# from influxdb_client.client.write_api import SYNCHRONOUS

# # 直接读取
# url = os.getenv("INFLUXDB_URL", "http://localhost:8086")
# token = os.getenv("INFLUXDB_TOKEN", "my-super-secret-token")
# org = os.getenv("INFLUXDB_ORG", "dexhand_org")

# client = InfluxDBClient(url=url, token=token, org=org)
# write_api = client.write_api(write_options=SYNCHRONOUS)
# query_api = client.query_api()

# # 导出这个变量供其他文件使用
# INFLUXDB_BUCKET = os.getenv("INFLUXDB_BUCKET", "sensor_data")