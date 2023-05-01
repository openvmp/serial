import sys

sys.path.append("test/lib")

from serial_test import SerialTesterNode

import os
import tempfile
from time import sleep

import pytest
import unittest

import rclpy
from launch import LaunchDescription
from launch.actions import (
    RegisterEventHandler,
    ExecuteProcess,
)
from launch_testing.actions import ReadyToTest
import launch_testing.markers
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from launch.substitutions import FindExecutable


TTY1 = "/tmp/ttyS21"
TTY2 = "/tmp/ttyS22"


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    socat = ExecuteProcess(
        name="socat",
        cmd=[
            [
                FindExecutable(name="socat"),
                " -s",
                " PTY,rawer,link=",
                TTY1,
                " PTY,rawer,link=",
                TTY2,
            ]
        ],
        shell=True,
    )
    node1 = Node(
        name="serial_com1",
        package="remote_serial",
        executable="remote_serial_standalone",
        # arguments=["--ros-args", "--log-level", "debug"],
        parameters=[
            {
                "serial_is_remote": False,
                "serial_prefix": "/serial/com1",
                "serial_dev_name": TTY1,
                # "serial_skip_init": True,
                "serial_baud_rate": 115200,
                "serial_data": 8,
                "serial_parity": False,
                "serial_stop": 1,
                "serial_flow_control": True,
            }
        ],
        output="screen",
    )
    node2 = Node(
        name="serial_com2",
        package="remote_serial",
        executable="remote_serial_standalone",
        # arguments=["--ros-args", "--log-level", "debug"],
        parameters=[
            {
                "serial_is_remote": False,
                "serial_prefix": "/serial/com2",
                "serial_dev_name": TTY2,
                # "serial_skip_init": True,
                "serial_baud_rate": 115200,
                "serial_data": 8,
                "serial_parity": False,
                "serial_stop": 1,
                "serial_flow_control": True,
            }
        ],
        output="screen",
    )

    return (
        LaunchDescription(
            [
                socat,
                RegisterEventHandler(
                    event_handler=OnProcessStart(
                        target_action=socat,
                        on_start=[
                            node1,
                            node2,
                        ],
                    )
                ),
                ReadyToTest(),
            ]
        ),
        {"socat": socat, "serial_com1": node1, "serial_com2": node2},
    )


class TestInjectOutput(unittest.TestCase):
    def test_basic(self, proc_output):
        rclpy.init()
        try:
            sleep(3)
            node = SerialTesterNode("test_node")
            node.subscribe(1)
            node.subscribe(2)
            node.subscribe(1, "output")
            node.subscribe(2, "output")
            sleep(1)

            future = node.inject(1)
            sleep(1)
            rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

            assert future.done(), "Client request timed out"
            _response = future.result()
            # assert response, "Could not inject!"
            sleep(3)

            assert node.test_context[1]["output"] == "test", (
                "com1 output mismatch: " + node.test_context[1]["output"]
            )
            assert node.test_context[2]["input"] == "test", (
                "com2 input mismatch: " + node.test_context[2]["input"]
            )
        finally:
            rclpy.shutdown()
            _ignore = 1
