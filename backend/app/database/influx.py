import os
from influxdb_client import InfluxDBClient
from influxdb_client.client.write_api import WriteOptions
from influxdb_client.domain.bucket_retention_rules import BucketRetentionRules


def _build_influx_url() -> str:
    host = os.getenv("INFLUX_HOST", "localhost")
    port = os.getenv("INFLUX_PORT", "8086")
    return f"http://{host}:{port}"


url = _build_influx_url()
token = os.getenv("INFLUX_TOKEN", "my-super-secret-token")
org = os.getenv("INFLUX_ORG", "dexhand_org")
INFLUXDB_BUCKET = os.getenv("INFLUXDB_BUCKET", "dexhand_bucket")

client = InfluxDBClient(url=url, token=token, org=org)
write_api = client.write_api(write_options=WriteOptions(batch_size=500))
query_api = client.query_api()


def setup_influx_retention(days: int = 7):
    """配置 InfluxDB Bucket 的数据保留策略"""
    try:
        buckets_api = client.buckets_api()
        bucket = buckets_api.find_bucket_by_name(INFLUXDB_BUCKET)
        if bucket:
            retention_seconds = days * 24 * 3600
            rule = BucketRetentionRules(type="expire", every_seconds=retention_seconds)
            bucket.retention_rules = [rule]
            buckets_api.update_bucket(bucket)
            print(f"✅ InfluxDB bucket '{INFLUXDB_BUCKET}' retention set to {days} days.")
    except Exception as e:
        print(f"❌ InfluxDB retention setup failed: {e}")
