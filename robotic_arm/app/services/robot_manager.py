import sdk.Robot as Robot

class RobotManager:
    def __init__(self):
        self.client_robot = None
        self.host = "192.168.58.2"

    def connect(self):
        if self.client_robot is not None:
            return True, "already connected"
        try:
            self.client_robot = Robot.RPC(self.host)
            return True, "connected"
        except Exception as e:
            self.client_robot = None
            raise e

    def get_robot(self):
        return self.client_robot

robot_manager = RobotManager()