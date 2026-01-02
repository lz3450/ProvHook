#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import csv


class SimpleJointStateLoggerNode(Node):
    def __init__(self):
        super().__init__("simple_joint_state_logger")

        self.joint_names_seen = set()
        self.joint_columns_written = False
        self.starting_time_sec = 0
        self.file = open("simple_joint_state_log.csv", "w", newline="")
        self.writer = csv.writer(self.file)

        self.subscription = self.create_subscription(
            JointState,  # Change to your topic's message type
            "/joint_commands",
            self.command_listener_callback,
            10,
        )
        self.subscription = self.create_subscription(
            JointState,  # Change to your topic's message type
            "/joint_states",
            self.states_listener_callback,
            10,
        )

    def log_joint_state(self, msg: JointState, topic: str):
        # On first message, define the header dynamically
        if not self.joint_columns_written:
            self.starting_time_sec = msg.header.stamp.sec
            self.joint_names_seen = msg.name
            headers = ["topic", "timestamp", "timestamp_ns"]
            for joint in msg.name:
                headers.extend([f"{joint}_position", f"{joint}_velocity", f"{joint}_effort"])

            self.writer.writerow(headers)
            self.joint_columns_written = True

        row = [topic, msg.header.stamp.sec, msg.header.stamp.nanosec]
        for i, joint in enumerate(self.joint_names_seen):
            pos = msg.position[i] if i < len(msg.position) else ""
            vel = msg.velocity[i] if i < len(msg.velocity) else ""
            eff = msg.effort[i] if i < len(msg.effort) else ""
            row.extend([pos, vel, eff])

        self.writer.writerow(row)

    def command_listener_callback(self, msg):
        self.log_joint_state(msg, "command")

    def states_listener_callback(self, msg):
        self.log_joint_state(msg, "states")

    def destroy_node(self):
        self.file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = SimpleJointStateLoggerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard interrupt, shutting down...")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
