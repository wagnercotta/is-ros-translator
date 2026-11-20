import pytest
from is_ros_translator_v2.config import Settings
from is_ros_translator_v2.gateway import Gateway


class FakeBridge:
    def __init__(self, version):
        self.version = version
        self.sent = []
        self.listeners = {}
        self.available = True

    def connected(self):
        return self.available

    async def close(self):
        self.available = False

    async def get_topics(self):
        return ["/chatter", "/odom"]

    async def get_services(self):
        return ["/map_server/map", "/dynamic_map"]

    async def get_topic_type(self, topic):
        return {"/chatter": "std_msgs/String" if self.version == 1 else "std_msgs/msg/String",
                "/odom": "nav_msgs/Odometry" if self.version == 1 else "nav_msgs/msg/Odometry"}.get(topic)

    async def get_service_type(self, endpoint):
        return "nav_msgs/GetMap" if self.version == 1 else "nav_msgs/srv/GetMap"

    async def get_action_type(self, endpoint):
        return "nav2_msgs/action/NavigateToPose" if endpoint == "/navigate_to_pose" else None

    async def action_details(self, ros_type):
        return {"typedefs": [{"type": ros_type, "fieldnames": [], "fieldtypes": [], "fieldarraylen": []}]}

    async def message_details(self, ros_type):
        if "String" in ros_type:
            return {"typedefs": [{"type": "std_msgs/String", "fieldnames": ["data"],
                                   "fieldtypes": ["string"], "fieldarraylen": [-1]}]}
        return {"typedefs": [{"type": ros_type, "fieldnames": [], "fieldtypes": [], "fieldarraylen": []}]}

    async def service_details(self, ros_type):
        return {"typedefs": [{"type": ros_type, "fieldnames": [], "fieldtypes": [], "fieldarraylen": []}]}

    async def publish(self, endpoint, ros_type, data):
        self.sent.append(("publish", endpoint, ros_type, data))
        return {"published": True}

    async def service(self, endpoint, ros_type, data, timeout):
        self.sent.append(("service", endpoint, ros_type, data))
        return {"map": {"data": [0, 0, 0, 0], "info": {"resolution": 0.2}}}

    async def action(self, endpoint, ros_type, data, timeout, feedback=None):
        self.sent.append(("action", endpoint, ros_type, data))
        if feedback:
            feedback({"distance_remaining": 0.0})
        return {"status": 4 if self.version == 2 else 3}

    async def subscribe(self, name, ros_type, callback):
        self.listeners[name] = callback
        if name == "/odom":
            callback({"header": {"frame_id": "map"}, "pose": {"pose": {"position": {"x": 4}}}, "twist": {}})
        return name

    async def unsubscribe(self, handle):
        self.listeners.pop(handle, None)


@pytest.fixture
def settings():
    return Settings.model_validate({
        "http": {"enabled": True},
        "security": {"require_api_key": False},
        "robots": {
            "r1": {"ros_version": 1, "host": "localhost", "navigation": {
                "action": "/move_base", "action_type": "move_base_msgs/MoveBaseAction"},
                "map_service": "/dynamic_map", "allowed": {"publish": ["/chatter"],
                    "subscribe": ["/odom", "/chatter"], "services": ["/dynamic_map"], "actions": ["/move_base"]}},
            "r2": {"ros_version": 2, "host": "localhost", "allowed": {
                "publish": ["/chatter"], "subscribe": ["/odom", "/chatter"],
                "services": ["/map_server/map"], "actions": ["/navigate_to_pose"]}},
        },
    })


@pytest.fixture
def gateway(settings):
    return Gateway(settings, bridges={"r1": FakeBridge(1), "r2": FakeBridge(2)})
