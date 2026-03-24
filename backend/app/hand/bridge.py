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

        self.global_state = {
            "joint": {
                "position": {},
                "velocity": {},
            },
            "touch": {
                "normal_force": [],
                "normal_force_delta": [],
                "tangential_force": [],
                "tangential_force_delta": [],
                "direction": [],
                "proximity": [],
                "temperature": [],
            },
            "motor": {
                "angle": [],
                "encoder_position": [],
                "current": [],
                "velocity": [],
                "error_code": [],
                "impedance": [],
            },
            "timestamp": 0,
        }

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

        self.publisher = self.create_publisher(JointState, "/right_hand/joint_commands", 10)
        self.create_subscription(JointState, "/right_hand/joint_states", self.callback_joint, 10)
        self.create_subscription(Float64MultiArray, "/right_hand/touch_sensors", self.callback_touch, 10)
        self.create_subscription(Float64MultiArray, "/right_hand/motor_feedback", self.callback_motor, 10)

    def publish_command(self, joint_map: dict):
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

    def callback_joint(self, msg: JointState):
        """合成语义关节的 position 和 velocity"""

        def process_metric(names, values, convert_to_deg=False):
            if not values or len(values) == 0:
                return {}
            temp_sums, temp_counts = {}, {}
            for name, val in zip(names, values):
                if name in self.ros_to_semantic:
                    safe_val = 0.0 if math.isnan(val) else val
                    semantic_name = self.ros_to_semantic[name]
                    processed_val = float(safe_val) * 180.0 / math.pi if convert_to_deg else float(safe_val)
                    temp_sums[semantic_name] = temp_sums.get(semantic_name, 0.0) + processed_val
                    temp_counts[semantic_name] = temp_counts.get(semantic_name, 0) + 1
            return {n: temp_sums[n] / temp_counts[n] for n in temp_sums}

        # 更新语义关节位置和速度
        self.global_state["joint"]["position"] = process_metric(msg.name, msg.position, convert_to_deg=True)
        self.global_state["joint"]["velocity"] = process_metric(msg.name, msg.velocity, convert_to_deg=True)
        self.global_state["timestamp"] = time.time()

    def callback_touch(self, msg: Float64MultiArray):
        data = msg.data.tolist()
        if len(data) < 40:
            return

        touch_dict = {"normal_force": [], "normal_force_delta": [], "tangential_force": [], "tangential_force_delta": [], "direction": [], "proximity": [], "temperature": []}
        for i in range(5):
            base = i * 8
            touch_dict["normal_force"].append(data[base + 1])
            touch_dict["normal_force_delta"].append(data[base + 2])
            touch_dict["tangential_force"].append(data[base + 3])
            touch_dict["tangential_force_delta"].append(data[base + 4])
            touch_dict["direction"].append(data[base + 5])
            touch_dict["proximity"].append(data[base + 6])
            touch_dict["temperature"].append(data[base + 7])

        self.global_state["touch"] = touch_dict

    def callback_motor(self, msg: Float64MultiArray):
        data = msg.data.tolist()
        if len(data) < 84:
            return

        motor_dict = {"angle": [], "encoder_position": [], "current": [], "velocity": [], "error_code": [], "impedance": []}
        for i in range(12):
            base = i * 7
            motor_dict["angle"].append(data[base + 1])
            motor_dict["encoder_position"].append(data[base + 2])
            curr_val = abs(data[base + 3])  # 电流取绝对值
            motor_dict["current"].append(curr_val)
            motor_dict["velocity"].append(data[base + 4])
            motor_dict["error_code"].append(data[base + 5])
            motor_dict["impedance"].append(data[base + 6])

        self.global_state["motor"] = motor_dict

    def call_reset_service(self) -> dict:
        resp = None
        ok = False
        message = ""

        try:
            client = self.create_client(Trigger, "/reset_hands")
            if not client.wait_for_service(timeout_sec=1.0):
                message = "Service /reset_hands not available"
            else:
                req = Trigger.Request()
                future = client.call_async(req)

                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

                if not future.done():
                    message = "Service call timeout"
                else:
                    resp = future.result()
                    if resp is None:
                        message = "Service call failed with no response"
                    else:
                        ok = bool(resp.success)
                        message = str(resp.message)
        except Exception as e:
            message = str(e)

        return {"success": ok, "message": message}


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

    def send_command(self, command: dict):
        if self._node:
            self._node.publish_command(command)
