import threading
import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from influxdb_client import Point
from app.database.influx import write_api, INFLUXDB_BUCKET, org


class BackendBridgeNode(Node):
    def __init__(self):
        super().__init__("backend_bridge_node")

        self.global_state = {"right": {"joints": {}, "touch": [], "motor": []}, "timestamp": 0}

        self.semantic_to_ros = {
            "th_dip": ["r_f_joint1_3", "r_f_joint1_4"],
            "th_mcp": ["r_f_joint1_2"],
            "th_rot": ["r_f_joint1_1"],
            "ff_spr": ["r_f_joint2_1", "r_f_joint4_1", "r_f_joint5_1"],
            "ff_dip": ["r_f_joint2_3", "r_f_joint2_4"],
            "ff_mcp": ["r_f_joint2_2"],
            "mf_dip": ["r_f_joint3_3", "r_f_joint3_4"],
            "mf_mcp": ["r_f_joint3_2"],
            "rf_dip": ["r_f_joint4_3", "r_f_joint4_4"],
            "rf_mcp": ["r_f_joint4_2"],
            "lf_dip": ["r_f_joint5_3", "r_f_joint5_4"],
            "lf_mcp": ["r_f_joint5_2"],
        }

        self.ros_to_semantic = {}
        for semantic, ros_list in self.semantic_to_ros.items():
            for ros_name in ros_list:
                self.ros_to_semantic[ros_name] = semantic

        # 4. 初始化 ROS 发布者和订阅者 (固定为 right_hand)
        self.publisher = self.create_publisher(JointState, "/right_hand/joint_commands", 10)
        self.get_logger().info("Publisher created: /right_hand/joint_commands")

        self.create_subscription(JointState, "/right_hand/joint_states", self.callback_joint, 10)
        self.create_subscription(Float64MultiArray, "/right_hand/touch_sensors", self.callback_touch, 10)
        self.create_subscription(Float64MultiArray, "/right_hand/motor_feedback", self.callback_motor, 10)

    def publish_command(self, hand: str, joint_map: dict):
        """发送指令：前端发来 th_dip，这里翻译成 r_f_joint1_3 和 1_4 发给 ROS"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        hardware_names = []
        hardware_positions = []

        for semantic_name, pos in joint_map.items():
            ros_names = self.semantic_to_ros.get(semantic_name, [])
            for ros_name in ros_names:
                hardware_names.append(ros_name)
                hardware_positions.append(float(pos))

        msg.name = hardware_names
        msg.position = hardware_positions
        self.publisher.publish(msg)

        # InfluxDB: 按前端发来的纯粹名字存入数据库
        try:
            point = Point("dexhand_commands").tag("hand", "right").time(time.time_ns())
            for name, pos in joint_map.items():
                point.field(name, float(pos))
            write_api.write(bucket=INFLUXDB_BUCKET, org=org, record=point)
        except Exception:
            pass

    def callback_joint(self, msg: JointState):
        """接收反馈：ROS 发来 r_f_joint1_3，这里翻译回 th_dip 还给前端"""
        joint_data = {}
        for name, pos_rad in zip(msg.name, msg.position):
            if name in self.ros_to_semantic:
                semantic_name = self.ros_to_semantic[name]
                pos_deg = float(pos_rad) * 180.0 / math.pi
                joint_data[semantic_name] = pos_deg

        self.global_state["right"]["joints"] = joint_data
        self.global_state["timestamp"] = time.time()

        # InfluxDB: 按翻译后的纯粹名字存入数据库
        try:
            point = Point("dexhand_joints").tag("hand", "right").time(time.time_ns())
            for name, pos in joint_data.items():
                point.field(name, float(pos))
            write_api.write(bucket=INFLUXDB_BUCKET, org=org, record=point)
        except Exception:
            pass

    def callback_touch(self, msg: Float64MultiArray):
        self.global_state["right"]["touch"] = msg.data.tolist()
        try:
            point = Point("dexhand_touch").tag("hand", "right").time(time.time_ns())
            for i, val in enumerate(msg.data.tolist()):
                point.field(f"sensor_{i}", float(val))
            write_api.write(bucket=INFLUXDB_BUCKET, org=org, record=point)
        except Exception:
            pass

    def callback_motor(self, msg: Float64MultiArray):
        self.global_state["right"]["motor"] = msg.data.tolist()
        try:
            point = Point("dexhand_motor").tag("hand", "right").time(time.time_ns())
            for i, val in enumerate(msg.data.tolist()):
                point.field(f"motor_{i}", float(val))
            write_api.write(bucket=INFLUXDB_BUCKET, org=org, record=point)
        except Exception:
            pass

    def call_reset_service(self):
        client = self.create_client(Trigger, "/reset_hands")
        if not client.wait_for_service(timeout_sec=1.0):
            return False
        client.call_async(Trigger.Request())
        return True


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
