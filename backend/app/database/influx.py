from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS
from app.config import settings


class InfluxWrapper:
    def __init__(self):
        try:
            self.client = InfluxDBClient(url=settings.INFLUXDB_URL, token=settings.INFLUXDB_TOKEN, org=settings.INFLUXDB_ORG)
            self.write_api = self.client.write_api(write_options=SYNCHRONOUS)
        except Exception as e:
            print(f"InfluxDB 连接失败: {e}")
            self.client = None

    def save_sensors(self, fingers: list):
        if not self.client:
            return
        point = Point("dexhand_sensors").field("f1", fingers[0]).field("f2", fingers[1]).field("f3", fingers[2])
        try:
            self.write_api.write(bucket=settings.INFLUXDB_BUCKET, record=point)
        except Exception:
            pass


# 单例
influx_db = InfluxWrapper()
