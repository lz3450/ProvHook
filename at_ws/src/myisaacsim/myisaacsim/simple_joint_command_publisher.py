#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
import numpy as np
import time


class SimpleJointCommandPublisher(Node):
    def __init__(self):
        super().__init__("test_ros2bridge")

        # Create the publisher. This publisher will publish a JointState message to the /joint_command topic.
        self.clock_subscriber_ = self.create_subscription(Clock, "/clock", self.clock_callback, 10)
        self.publisher_ = self.create_publisher(JointState, "joint_commands", 10)

        # Create a JointState message
        self.joint_state = JointState()

        self.joint_state.name = [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
            "panda_finger_joint1",
            "panda_finger_joint2",
        ]

        num_joints = len(self.joint_state.name)

        # make sure kit's editor is playing for receiving messages
        self.joint_state.position = np.array([0.0] * num_joints, dtype=np.float64).tolist()
        self.default_joints = [0.0, -1.16, -0.0, -2.3, -0.0, 1.6, 1.1, 0.4, 0.4]

        # limiting the movements to a smaller range (this is not the range of the robot, just the range of the movement
        self.max_joints = np.array(self.default_joints) + 0.5
        self.min_joints = np.array(self.default_joints) - 0.5

        # position control the robot to wiggle around each joint
        self.ts_start_sec: int | None = None
        self.ts_sec: int
        self.ts_nanosec: int

        self.timer = self.create_timer(1, self.timer_callback)

    def clock_callback(self, msg: Clock):
        self.ts_sec = msg.clock.sec
        self.ts_nanosec = msg.clock.nanosec

    def timer_callback(self):
        if self.ts_start_sec is None:
            self.ts_start_sec = self.ts_sec
            return

        time_elapsed = self.ts_sec - self.ts_start_sec
        self.joint_state.header.stamp = Time(sec=self.ts_sec, nanosec=self.ts_nanosec)

        if (time_elapsed) % 2 == 0:
            joint_position = np.array(self.default_joints) + 0.25
        else:
            joint_position = np.array(self.default_joints) - 0.25

        self.joint_state.position = joint_position.tolist()

        # Publish the message to the topic
        self.publisher_.publish(self.joint_state)


def main():
    rclpy.init()

    publisher = SimpleJointCommandPublisher()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        print("Keyboard interrupt, shutting down...")

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
