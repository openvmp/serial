import rclpy
from rclpy.node import Node as rclpyNode

from remote_serial.srv import InjectOutput
import std_msgs.msg


class SerialTesterNode(rclpyNode):
    test_context = {}

    def __init__(self, name="serial_tester_node"):
        super().__init__(name)

    def _save(self, id, direction, msg):
        print("Data received on com" + str(id) + " on " + direction)
        if not id in self.test_context:
            self.test_context[id] = {}
        if not direction in self.test_context[id]:
            self.test_context[id][direction] = ""
        self.test_context[id][direction] += msg.data

    def subscribe(
        self,
        id=1,
        direction="input",
    ):
        this_node = self
        subscription = self.create_subscription(
            std_msgs.msg.String,
            "/serial/com" + str(id) + "/inspect/" + direction,
            lambda msg: this_node._save(id, direction, msg),
            10,
        )
        return subscription

    def inject(
        self, id=1, direction="output", text=bytes("test", "latin1"), timeout=10.0
    ):
        client = self.create_client(
            InjectOutput, "/serial/com" + str(id) + "/inject/" + str(direction)
        )
        ready = client.wait_for_service(timeout_sec=timeout)
        if not ready:
            raise RuntimeError("Wait for service timed out")

        request = InjectOutput.Request()
        request.data = str(text, "latin_1", "ignore")
        future = client.call_async(request)
        return future
