import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from influxdb_client import Point
from app.database.influx import write_api, INFLUXDB_BUCKET, org


class BackendBridgeNode(Node):
    def __init__(self):
        super().__init__("backend_bridge_node")

        self.hands = ["left", "right"]
        self.global_state = {"left": {"joints": {}, "touch": [], "motor": []}, "right": {"joints": {}, "touch": [], "motor": []}, "timestamp": 0}

        self.publishers_dict = {}
        for hand in self.hands:
            topic = f"/{hand}_hand/joint_commands"
            self.publishers_dict[hand] = self.create_publisher(JointState, topic, 10)
            self.get_logger().info(f"Publisher created: {topic}")

        for hand in self.hands:
            self.create_subscription(JointState, f"/{hand}_hand/joint_states", lambda msg, h=hand: self.callback_joint(msg, h), 10)
            self.create_subscription(Float64MultiArray, f"/{hand}_hand/touch_sensors", lambda msg, h=hand: self.callback_touch(msg, h), 10)
            self.create_subscription(Float64MultiArray, f"/{hand}_hand/motor_feedback", lambda msg, h=hand: self.callback_motor(msg, h), 10)

    def callback_joint(self, msg: JointState, hand: str):
        joint_data = {}
        for name, pos in zip(msg.name, msg.position):
            joint_data[name] = pos
        self.global_state[hand]["joints"] = joint_data
        self.global_state["timestamp"] = time.time()

        try:
            point = Point("dexhand_joints").tag("hand", hand).time(time.time_ns())

            for name, pos in joint_data.items():
                point.field(name, float(pos))

            write_api.write(bucket=INFLUXDB_BUCKET, org=org, record=point)
        except Exception:
            pass

    def callback_touch(self, msg: Float64MultiArray, hand: str):
        data_list = msg.data.tolist()
        self.global_state[hand]["touch"] = data_list

        try:
            point = Point("dexhand_touch").tag("hand", hand).time(time.time_ns())

            for i, val in enumerate(data_list):
                point.field(f"sensor_{i}", float(val))

            write_api.write(bucket=INFLUXDB_BUCKET, org=org, record=point)
        except Exception:
            pass

    def callback_motor(self, msg: Float64MultiArray, hand: str):
        data_list = msg.data.tolist()
        self.global_state[hand]["motor"] = data_list

        try:
            point = Point("dexhand_motor").tag("hand", hand).time(time.time_ns())

            for i, val in enumerate(data_list):
                point.field(f"motor_{i}", float(val))

            write_api.write(bucket=INFLUXDB_BUCKET, org=org, record=point)
        except Exception:
            pass

    def call_reset_service(self):
        client = self.create_client(Trigger, "/reset_hands")
        if not client.wait_for_service(timeout_sec=1.0):
            return False
        req = Trigger.Request()
        client.call_async(req)
        return True

    def publish_command(self, hand: str, joint_map: dict):
        if hand not in self.publishers_dict:
            self.get_logger().error(f"Unknown hand: {hand}")
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(joint_map.keys())
        msg.position = [float(v) for v in joint_map.values()]

        # 发布消息
        self.publishers_dict[hand].publish(msg)

        try:
            point = Point("dexhand_commands").tag("hand", hand).time(time.time_ns())
            for name, pos in joint_map.items():
                point.field(name, float(pos))
            write_api.write(bucket=INFLUXDB_BUCKET, org=org, record=point)
        except Exception:
            pass


class ROSBridgeManager:
    def __init__(self):
        self._node: BackendBridgeNode | None = None
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()

    def start(self) -> BackendBridgeNode:
        with self._lock:
            if self._node is not None:
                return self._node

            if not rclpy.ok():
                rclpy.init()

            self._node = BackendBridgeNode()
            self._thread = threading.Thread(target=rclpy.spin, args=(self._node,), daemon=True)
            self._thread.start()
            return self._node

    def is_started(self) -> bool:
        return self._node is not None

    def get_latest_state(self) -> dict:
        node = self._node
        return node.global_state if node else {}

    def send_command(self, hand: str, command: dict):
        if self._node:
            self._node.publish_command(hand, command)
