import threading
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState


class BackendBridgeNode(Node):
    def __init__(self):
        super().__init__("backend_bridge_node")

        # 定义需要监听和控制的手
        self.hands = ["left", "right"]

        # 内部状态存储，用于聚合所有 Topic 的数据
        self.global_state = {"left": {"joints": {}, "touch": [], "motor": []}, "right": {"joints": {}, "touch": [], "motor": []}, "timestamp": 0}

        # --- 1. 创建发布者 (Publishers) ---
        self.publishers_dict = {}
        for hand in self.hands:
            topic = f"/{hand}_hand/joint_commands"
            self.publishers_dict[hand] = self.create_publisher(JointState, topic, 10)
            self.get_logger().info(f"Publisher created: {topic}")

        # --- 2. 创建订阅者 (Subscribers) ---
        for hand in self.hands:
            # 2.1 关节反馈 (Joint States)
            self.create_subscription(JointState, f"/{hand}_hand/joint_states", lambda msg, h=hand: self.callback_joint(msg, h), 10)
            # 2.2 触觉传感器 (Touch Sensors)
            self.create_subscription(Float64MultiArray, f"/{hand}_hand/touch_sensors", lambda msg, h=hand: self.callback_touch(msg, h), 10)
            # 2.3 电机反馈 (Motor Feedback)
            self.create_subscription(Float64MultiArray, f"/{hand}_hand/motor_feedback", lambda msg, h=hand: self.callback_motor(msg, h), 10)

    # --- 回调函数：处理接收到的 ROS 数据并更新 global_state ---
    def callback_joint(self, msg: JointState, hand: str):
        # 将 JointState 转换为字典: {"joint_name": angle, ...}
        joint_data = {}
        for name, pos in zip(msg.name, msg.position):
            joint_data[name] = pos
        self.global_state[hand]["joints"] = joint_data
        self.global_state["timestamp"] = time.time()

    def callback_touch(self, msg: Float64MultiArray, hand: str):
        # 触觉数据是数组，直接转 list
        self.global_state[hand]["touch"] = msg.data.tolist()

    def callback_motor(self, msg: Float64MultiArray, hand: str):
        # 电机数据是数组，直接转 list
        self.global_state[hand]["motor"] = msg.data.tolist()

    # --- 控制函数：发送指令到 ROS ---
    def publish_command(self, hand: str, joint_map: dict):
        """
        发送关节控制指令
        :param hand: 'left' or 'right'
        :param joint_map: {"joint_name": angle_value, ...}
        """
        if hand not in self.publishers_dict:
            self.get_logger().error(f"Unknown hand: {hand}")
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(joint_map.keys())
        msg.position = [float(v) for v in joint_map.values()]

        # 发布消息
        self.publishers_dict[hand].publish(msg)


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
        """获取当前聚合的完整状态"""
        node = self._node
        return node.global_state if node else {}

    def send_command(self, hand: str, command: dict):
        """对外暴露的控制接口"""
        if self._node:
            self._node.publish_command(hand, command)
