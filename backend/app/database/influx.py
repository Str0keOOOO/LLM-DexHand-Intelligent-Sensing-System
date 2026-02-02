import os
from influxdb_client import InfluxDBClient
from influxdb_client.client.write_api import WriteOptions


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
