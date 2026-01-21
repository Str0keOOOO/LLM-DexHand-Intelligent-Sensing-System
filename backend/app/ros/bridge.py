import threading
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class BackendBridgeNode(Node):
    def __init__(self):
        super().__init__("backend_bridge_listener")
        self.status_sub = self.create_subscription(String, "/dexhand/status", self.on_status_received, 10)
        self.latest_state = {"mode": "DISCONNECTED", "payload": None, "timestamp": 0}

    def on_status_received(self, msg):
        try:
            self.latest_state = json.loads(msg.data)
        except json.JSONDecodeError:
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
        return (node.latest_state if node else {}) or {}
