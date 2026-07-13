#!/usr/bin/env python3

from greenwave_monitor_interfaces.srv import ManageTopic
import rclpy
from rclpy.node import Node

IGNORED_TOPICS = {'/rosout', '/parameter_events', '/diagnostics'}

IGNORED_TOPIC_SUFFIXES = ('/compressedDepth',)

DISCOVERY_PERIOD_SEC = 2.0
MANAGE_TOPIC_SERVICE = 'greenwave_monitor/manage_topic'


class GreenwaveAutoDiscovery(Node):

    def __init__(self):
        super().__init__('greenwave_auto_discovery')
        self._monitored = set()
        self._client = self.create_client(ManageTopic, MANAGE_TOPIC_SERVICE)
        self.create_timer(DISCOVERY_PERIOD_SEC, self._discover)

    def _discover(self):
        if not self._client.service_is_ready():
            return
        for topic_name, _ in self.get_topic_names_and_types():
            if topic_name in self._monitored or topic_name in IGNORED_TOPICS:
                continue
            if topic_name.endswith(IGNORED_TOPIC_SUFFIXES):
                continue
            request = ManageTopic.Request()
            request.topic_name = topic_name
            request.add_topic = True
            self._client.call_async(request)
            self._monitored.add(topic_name)


def main(args=None):
    rclpy.init(args=args)
    node = GreenwaveAutoDiscovery()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
