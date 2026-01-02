#
# logger.py
#
# Logger for RoboGuard task0
#

import os
from datetime import datetime
import csv

###
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.msg import MotionPlanRequest

HOME_DIR = os.path.expanduser("~")
TIMESTAMP = datetime.now().strftime("%Y%m%d_%H%M%S")
DATA_ROOT_DIR = f"{HOME_DIR}/Projects/RoboGuard/rg_ws/data/task0"
JOINT_STATE_CSV_PATH = f"{DATA_ROOT_DIR}/{TIMESTAMP}.csv"
MOTION_PLAN_LOG_PATH = f"{DATA_ROOT_DIR}/motion_plan-{TIMESTAMP}.txt"


class Task0LoggerNode(Node):
    def __init__(self):
        super().__init__("task0_logger_node")

        self.joint_header_ready = False
        self.starting_time_sec = 0
        self.joint_state_csv = open(JOINT_STATE_CSV_PATH, "w", newline="")
        self.motion_plan_log = open(MOTION_PLAN_LOG_PATH, "w", newline="")
        self.writer = csv.writer(self.joint_state_csv)

        self.states_subscription = self.create_subscription(
            JointState,
            "/isaac_joint_states",
            self.states_listener_callback,
            10,
        )

        self.commands_subscription = self.create_subscription(
            JointState,
            "/isaac_joint_commands",
            self.command_listener_callback,
            10,
        )

        self.motion_plan_subscription = self.create_subscription(
            MotionPlanRequest,
            "/motion_plan_request",
            self.motion_plan_listener_callback,
            10,
        )

    def log_joint_state(self, msg: JointState, topic: str):
        row = [topic, msg.header.stamp.sec, msg.header.stamp.nanosec]
        row_pos = []
        row_vel = []
        row_eff = []
        for i in range(len(self.joint_names_seen)):
            pos = msg.position[i] if i < len(msg.position) else ""
            vel = msg.velocity[i] if i < len(msg.velocity) else ""
            eff = msg.effort[i] if i < len(msg.effort) else ""
            row_pos.append(pos)
            row_vel.append(vel)
            row_eff.append(eff)

        self.writer.writerow(row + row_pos + row_vel + row_eff)

    def command_listener_callback(self, msg: JointState):
        if self.joint_header_ready:
            self.log_joint_state(msg, "c")

    def states_listener_callback(self, msg: JointState):
        # On first message, define the header dynamically
        if not self.joint_header_ready:
            self.starting_time_sec = msg.header.stamp.sec
            self.joint_names_seen = tuple(msg.name)
            header = ["t", "sec", "nanosec"]
            for t in ("pos", "vel", "eff"):
                header.extend([f"{joint.replace('panda_', '')}_{t}" for joint in self.joint_names_seen])

            self.writer.writerow(header)
            self.joint_header_ready = True

        self.log_joint_state(msg, "s")

    def motion_plan_listener_callback(self, msg: MotionPlanRequest):
        print(msg, file=self.motion_plan_log)

    def destroy_node(self):
        self.joint_state_csv.close()
        self.motion_plan_log.close()
        super().destroy_node()


def main(args=None):
    os.makedirs(DATA_ROOT_DIR, exist_ok=True)
    latest_symlink = f"{DATA_ROOT_DIR}/../task0.csv"
    if os.path.islink(latest_symlink):
        os.remove(latest_symlink)
    os.symlink(
        os.path.relpath(JOINT_STATE_CSV_PATH, os.path.dirname(latest_symlink)),
        latest_symlink,
        target_is_directory=False,
    )

    rclpy.init(args=args)

    node = Task0LoggerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard interrupt, shutting down...")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
