import threading
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState

# --- 新增：引入数据库写入工具 ---
from influxdb_client import Point
from app.database.influx import write_api, INFLUXDB_BUCKET, org


class BackendBridgeNode(Node):
    def __init__(self):
        super().__init__("backend_bridge_node")

        self.hands = ["left", "right"]
        # 原有的全局状态保持不变
        self.global_state = {"left": {"joints": {}, "touch": [], "motor": []}, "right": {"joints": {}, "touch": [], "motor": []}, "timestamp": 0}

        # --- 1. 创建发布者 ---
        self.publishers_dict = {}
        for hand in self.hands:
            topic = f"/{hand}_hand/joint_commands"
            self.publishers_dict[hand] = self.create_publisher(JointState, topic, 10)
            self.get_logger().info(f"Publisher created: {topic}")

        # --- 2. 创建订阅者 ---
        for hand in self.hands:
            self.create_subscription(JointState, f"/{hand}_hand/joint_states", lambda msg, h=hand: self.callback_joint(msg, h), 10)
            self.create_subscription(Float64MultiArray, f"/{hand}_hand/touch_sensors", lambda msg, h=hand: self.callback_touch(msg, h), 10)
            self.create_subscription(Float64MultiArray, f"/{hand}_hand/motor_feedback", lambda msg, h=hand: self.callback_motor(msg, h), 10)

    def callback_joint(self, msg: JointState, hand: str):
        # --- 原有逻辑：更新内存状态 ---
        joint_data = {}
        for name, pos in zip(msg.name, msg.position):
            joint_data[name] = pos
        self.global_state[hand]["joints"] = joint_data
        self.global_state["timestamp"] = time.time()

        # --- 新增逻辑：写入数据库 (异步非阻塞) ---
        try:
            # 创建一个时间点数据
            point = Point("dexhand_joints").tag("hand", hand).time(time.time_ns())  # 使用纳秒级时间戳

            # 将所有关节角度存入字段
            for name, pos in joint_data.items():
                point.field(name, float(pos))

            # 这一步是瞬间完成的，因为我们在 influx.py 里开了 batching
            write_api.write(bucket=INFLUXDB_BUCKET, org=org, record=point)
        except Exception:
            pass  # 忽略写入错误，绝不影响机器人控制

    def callback_touch(self, msg: Float64MultiArray, hand: str):
        # --- 原有逻辑 ---
        data_list = msg.data.tolist()
        self.global_state[hand]["touch"] = data_list

        # --- 新增逻辑 ---
        try:
            point = Point("dexhand_touch").tag("hand", hand).time(time.time_ns())

            for i, val in enumerate(data_list):
                point.field(f"sensor_{i}", float(val))

            write_api.write(bucket=INFLUXDB_BUCKET, org=org, record=point)
        except Exception:
            pass

    def callback_motor(self, msg: Float64MultiArray, hand: str):
        # --- 原有逻辑 ---
        data_list = msg.data.tolist()
        self.global_state[hand]["motor"] = data_list

        # --- 新增逻辑 ---
        try:
            point = Point("dexhand_motor").tag("hand", hand).time(time.time_ns())

            for i, val in enumerate(data_list):
                point.field(f"motor_{i}", float(val))

            write_api.write(bucket=INFLUXDB_BUCKET, org=org, record=point)
        except Exception:
            pass

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

        # --- 新增：记录下发的指令，用于后续“指令vs实际”的对比分析 ---
        try:
            point = Point("dexhand_commands").tag("hand", hand).time(time.time_ns())
            for name, pos in joint_map.items():
                point.field(name, float(pos))
            write_api.write(bucket=INFLUXDB_BUCKET, org=org, record=point)
        except Exception:
            pass


# --- ROSBridgeManager 类保持完全一致 ---
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
            # 这里的 daemon=True 很重要，保证后端关闭时 ROS 线程也自动退出
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
