import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import BatteryState, Imu
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from irobot_create_msgs.msg import DockStatus # <-- Re-added for dock status
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import message_filters
import time
import threading
import subprocess
import os
import pandas as pd
import numpy as np
import math
import joblib
from sklearn.preprocessing import StandardScaler

# --- 1. PATROL CONFIGURATION ---
TEST_DURATION_SECONDS = 24 * 60 * 60  # 24 Hours
BATTERY_LOW_THRESHOLD = 0.25          # 25%
BATTERY_HIGH_THRESHOLD = 0.95         # 95% (Charged)
DOCK_TIMEOUT_SECONDS = 90.0           # 1.5-minute dock attempt buffer

# --- 2. DOCK SCRIPT CONFIGURATION ---
# Use os.path.expanduser to resolve the '~' home directory
DOCK_SCRIPT_BASE_PATH = os.path.expanduser("~/Projects/RoboGuard/rg_ws/tb4/scripts")
DOCK_SCRIPT_NAME = "dock.sh"
UNDOCK_SCRIPT_NAME = "undock.sh"

# --- 3. WAYPOINT CONFIGURATION (UPDATE THESE FROM RVIZ) ---
# Use 'ros2 topic echo /goal_pose --once' to get these values
# Position A
POS_A = {'x': -2.14, 'y': -2.49, 'z': 0.0, 'orientation_z': 0.0, 'orientation_w': 1.0} 
# Position B
POS_B = {'x': -1.11, 'y': 0.52, 'z': 0.0, 'orientation_z': 1.0, 'orientation_w': 0.0}
# Docking Staging Area (Location to go to BEFORE running dock.sh)
POS_DOCK = {'x': -0.47, 'y': -0.34, 'z': 0.0, 'orientation_z': 0.0, 'orientation_w': 1.0}
# Docking Failure Retry Point (A point to back up to)
POS_RETRY = {'x': -1.22, 'y': 0.05, 'z': 0.0, 'orientation_z': 0.0, 'orientation_w': 1.0}

# --- 4. DATA LOGGING CONFIGURATION ---
LOG_CHUNK_SIZE = 10000 # Save file every 10,000 data points
DATA_LOG_PATH = "./patrol_data_logs" # Directory to save chunk files
REALTIME_COLUMN_HEADERS = [
    "timestamp",
    "pos_x", "pos_y", "yaw",
    "linear_vel_x", "angular_vel_z",
    "cmd_linear_x", "cmd_angular_z",
    "imu_orientation_x", "imu_orientation_y", "imu_orientation_z", "imu_orientation_w",
    "imu_angular_velocity_x", "imu_angular_velocity_y", "imu_angular_velocity_z",
    "imu_linear_acceleration_x", "imu_linear_acceleration_y", "imu_linear_acceleration_z",
]

# =================================Example=============================================
# HELPER NODE 1: BATTERY MONITOR
# =============================================================================
class BatteryMonitor(Node):
    """Subscribes to battery state and caches the latest percentage."""
    def __init__(self):
        super().__init__('battery_monitor')
        self.battery_percent = 1.0 
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(
            BatteryState, '/battery_state', self.battery_callback, qos
        )
    def battery_callback(self, msg):
        self.battery_percent = msg.percentage

# =============================================================================
# HELPER NODE 2: DOCK MONITOR (RE-ADDED)
# =============================================================================
class DockMonitor(Node):
    """Subscribes to dock status and caches the latest boolean."""
    def __init__(self):
        super().__init__('dock_monitor')
        self.is_docked = False
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # Dock status is important
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(
            DockStatus,
            '/dock_status', # Check your robot's topic
            self.dock_callback,
            qos
        )
    def dock_callback(self, msg):
        self.is_docked = msg.is_docked

# =============================================================================
# HELPER NODE 3: DATA LOGGER
# (This class is identical to the previous version and is omitted for brevity)
# (It handles IMU, Odom, cmd_vel, and saves in chunks)
# =============================================================================
class DataLogger(Node):
    """
    Runs in a background thread and logs all data in chunks.
    This is a merge of your data_collection.py.
    """
    def __init__(self):
        super().__init__('data_logger')
        self.data_buffer = []
        self.last_imu_msg = None
        self.buffer_lock = threading.Lock()
        self.chunk_index = 0
        
        # Create data log directory
        os.makedirs(DATA_LOG_PATH, exist_ok=True)
        self.get_logger().info(f"Logging data chunks to: {os.path.abspath(DATA_LOG_PATH)}")

        qos_profile_best_effort = QoSProfile(          
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.odom_sub = message_filters.Subscriber(
            self, Odometry, '/odom', qos_profile=qos_profile_best_effort
        )
        self.cmd_vel_sub = message_filters.Subscriber(
            self, Twist, '/cmd_vel', qos_profile=qos_profile_best_effort
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, qos_profile_best_effort
        )
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.odom_sub, self.cmd_vel_sub], 
            queue_size=30, 
            slop=0.1,
            allow_headerless=True
        )
        self.ts.registerCallback(self.synchronized_callback)
        self.save_timer = self.create_timer(5.0, self.check_buffer_and_save)

    def imu_callback(self, msg: Imu):
        self.last_imu_msg = msg

    def synchronized_callback(self, odom_msg: Odometry, cmd_vel_msg: Twist):
        if self.last_imu_msg is None:
            return 
        time_sec = odom_msg.header.stamp.sec
        time_nanosec = odom_msg.header.stamp.nanosec
        timestamp = time_sec + time_nanosec * 1e-9

        pos_x = odom_msg.pose.pose.position.x
        pos_y = odom_msg.pose.pose.position.y
        _, _, yaw = self.euler_from_quaternion(odom_msg.pose.pose.orientation)
        linear_vel_x = odom_msg.twist.twist.linear.x
        angular_vel_z = odom_msg.twist.twist.angular.z
        cmd_linear_x = cmd_vel_msg.linear.x
        cmd_angular_z = cmd_vel_msg.angular.z
        imu = self.last_imu_msg
        imu_orientation_x, imu_orientation_y, imu_orientation_z, imu_orientation_w = (
            imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w
        )
        imu_angular_velocity_x, imu_angular_velocity_y, imu_angular_velocity_z = (
            imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z
        )
        imu_linear_acceleration_x, imu_linear_acceleration_y, imu_linear_acceleration_z = (
            imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z
        )
        data_row = [
            timestamp,
            pos_x, pos_y, yaw,
            linear_vel_x, angular_vel_z,
            cmd_linear_x, cmd_angular_z,
            imu_orientation_x, imu_orientation_y, imu_orientation_z, imu_orientation_w,
            imu_angular_velocity_x, imu_angular_velocity_y, imu_angular_velocity_z,
            imu_linear_acceleration_x, imu_linear_acceleration_y, imu_linear_acceleration_z
        ]
        with self.buffer_lock:
            self.data_buffer.append(data_row)

    def check_buffer_and_save(self):
        with self.buffer_lock:
            if len(self.data_buffer) < LOG_CHUNK_SIZE:
                return
            data_to_save = list(self.data_buffer)
            self.data_buffer.clear()
            
        self.get_logger().info(f"Buffer full, saving chunk {self.chunk_index}...")
        try:
            filename = os.path.join(DATA_LOG_PATH, f"patrol_data_chunk_{self.chunk_index}.csv")
            df = pd.DataFrame(data_to_save, columns=REALTIME_COLUMN_HEADERS)
            df.to_csv(filename, index=False)
            self.get_logger().info(f"Successfully saved {len(data_to_save)} points to {filename}")
            self.chunk_index += 1
        except Exception as e:
            self.get_logger().error(f"Failed to save data chunk: {e}")

    def save_final_chunk(self):
        with self.buffer_lock:
            if not self.data_buffer:
                self.get_logger().info("No final data to save.")
                return
            data_to_save = list(self.data_buffer)
            self.data_buffer.clear()
        
        self.get_logger().info(f"Saving final chunk {self.chunk_index}...")
        try:
            filename = os.path.join(DATA_LOG_PATH, f"patrol_data_chunk_{self.chunk_index}.csv")
            df = pd.DataFrame(data_to_save, columns=REALTIME_COLUMN_HEADERS)
            df.to_csv(filename, index=False)
            self.get_logger().info(f"Successfully saved {len(data_to_save)} final points.")
        except Exception as e:
            self.get_logger().error(f"Failed to save final data chunk: {e}")

    def euler_from_quaternion(self, q):
        t3 = +2.0 * (q.w * q.z + q.x * q.y); t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_z = math.atan2(t3, t4)
        return 0.0, 0.0, yaw_z

# =============================================================================
# HELPER FUNCTIONS
# =============================================================================
def create_pose(navigator, location_dict):
    """Helper to create PoseStamped messages from our dict"""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = location_dict['x']
    pose.pose.position.y = location_dict['y']
    pose.pose.position.z = location_dict['z']
    pose.pose.orientation.z = location_dict['orientation_z']
    pose.pose.orientation.w = location_dict['orientation_w']
    return pose

def run_shell_script(script_name):
    """
    Runs a shell script from the configured path NON-BLOCKING.
    This "fires and forgets" the script.
    Returns True on success, False on failure.
    """
    full_path = os.path.join(DOCK_SCRIPT_BASE_PATH, script_name)
    print(f"--- Triggering: {full_path} ---")
    try:
        # Use Popen for non-blocking execution.
        # We don't wait for it, we just fire it.
        subprocess.Popen(
            [full_path], 
            shell=True, 
            stdout=subprocess.DEVNULL, # Discard output
            stderr=subprocess.DEVNULL
        )
        print(f"--- Script {script_name} triggered successfully. ---")
        return True
    except Exception as e:
        print(f"!!! Error trying to run script {script_name}: {e}")
        return False

def send_nav_goal(navigator, pose, state_name):
    """Sends a Nav2 goal and waits for completion."""
    print(f"[{state_name}] Navigating to: {pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}")
    navigator.goToPose(pose)
    while not navigator.isTaskComplete():
        time.sleep(0.5)
    
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print(f"[{state_name}] Reached goal successfully.")
        return True
    elif result == TaskResult.CANCELED:
        print(f"[{state_name}] Navigation was canceled.")
        return False
    elif result == TaskResult.FAILED:
        print(f"[{state_name}] Navigation failed!")
        return False
    return False

# =============================================================================
# MAIN CONTROL LOOP
# =============================================================================
def main():
    rclpy.init()

    # 1. Create all our nodes
    navigator = BasicNavigator()
    battery_monitor = BatteryMonitor()
    dock_monitor = DockMonitor() # <-- Re-added
    data_logger = DataLogger()

    # 2. Start all helper nodes in a MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(battery_monitor)
    executor.add_node(dock_monitor) # <-- Re-added
    executor.add_node(data_logger)
    
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # 3. Setup Navigation
    print("Waiting for Nav2 to activate...")
    navigator.waitUntilNav2Active()
    print("Nav2 is ready!")

    # 4. Initialize State Machine
    start_time = time.time()
    current_target = "A"
    state = "PATROLLING"
    attempt_start_time = None
    
    # Create pose objects once
    pose_a = create_pose(navigator, POS_A)
    pose_b = create_pose(navigator, POS_B)
    pose_dock = create_pose(navigator, POS_DOCK)
    pose_retry = create_pose(navigator, POS_RETRY)

    print("--- STARTING 24-HOUR ENDURANCE TEST & LOGGING (HYBRID DOCKING) ---")

    try:
        while rclpy.ok():
            elapsed = time.time() - start_time
            if elapsed > TEST_DURATION_SECONDS:
                print("--- 24 HOUR TEST COMPLETE ---")
                break
            
            battery = battery_monitor.battery_percent
            is_docked = dock_monitor.is_docked # Get "ground truth"
            
            # --- Main State Machine ---
            
            if state == "PATROLLING":
                print(f"[Patrolling] Battery: {battery*100:.1f}% | Next Target: {current_target}")
                
                if battery < BATTERY_LOW_THRESHOLD:
                    print("!!! BATTERY LOW. Transition to DOCKING.")
                    navigator.cancelTask()
                    state = "GOING_TO_DOCK"
                    continue 

                if navigator.isTaskComplete():
                    if current_target == "A":
                        send_nav_goal(navigator, pose_a, state)
                        current_target = "B"
                    else:
                        send_nav_goal(navigator, pose_b, state)
                        current_target = "A"
            
            elif state == "GOING_TO_DOCK":
                print("[Going to Dock] Navigating to dock staging area...")
                if send_nav_goal(navigator, pose_dock, state):
                    print("[Going to Dock] Reached staging area. Transition to SENDING_DOCK_COMMAND.")
                    state = "SENDING_DOCK_COMMAND"
                else:
                    print("[Going to Dock] FAILED to reach dock staging area. Retrying...")
                    state = "GOING_TO_DOCK" 
            
            elif state == "SENDING_DOCK_COMMAND":
                print("[Docking] Calling dock.sh to *initiate* docking...")
                if run_shell_script(DOCK_SCRIPT_NAME):
                    attempt_start_time = time.time()
                    state = "WAITING_FOR_DOCK_STATUS"
                else:
                    print("[Docking] Failed to *run* dock.sh. Retrying in 10s.")
                    time.sleep(10.0)
                    # State remains SENDING_DOCK_COMMAND

            elif state == "WAITING_FOR_DOCK_STATUS":
                # Check 1: Success
                if is_docked:
                    print("[Docking] Dock status is TRUE. Success!")
                    state = "CHARGING"
                    continue
                
                # Check 2: Failure (Timeout)
                if time.time() - attempt_start_time > DOCK_TIMEOUT_SECONDS:
                    print(f"[Docking] FAILED: No dock status after {DOCK_TIMEOUT_SECONDS}s.")
                    state = "HANDLING_DOCK_FAILURE"
                    continue
                
                print(f"[Docking] Waiting for /dock_status to be true... ({time.time() - attempt_start_time:.0f}s)")

            elif state == "HANDLING_DOCK_FAILURE":
                print("[Dock Failure] Navigating to RETRY point...")
                if send_nav_goal(navigator, pose_retry, state):
                    print("[Dock Failure] Reached RETRY point. Will now retry docking.")
                    state = "GOING_TO_DOCK" # Go back to staging area
                else:
                    print("[Dock Failure] FAILED to reach RETRY point. Retrying...")
                    state = "HANDLING_DOCK_FAILURE"

            elif state == "CHARGING":
                if battery >= BATTERY_HIGH_THRESHOLD:
                    print(f"[Charging] Battery is {battery*100:.1f}%. Transition to SENDING_UNDOCK_COMMAND.")
                    state = "SENDING_UNDOCK_COMMAND"
                else:
                    print(f"[Charging] Battery: {battery*100:.1f}%...")
            
            elif state == "SENDING_UNDOCK_COMMAND":
                print("[Undocking] Calling undock.sh to *initiate* undocking...")
                if run_shell_script(UNDOCK_SCRIPT_NAME):
                    attempt_start_time = time.time()
                    state = "WAITING_FOR_UNDOCK_STATUS"
                else:
                    print("[Undocking] Failed to *run* undock.sh. Retrying in 10s.")
                    time.sleep(10.0)

            elif state == "WAITING_FOR_UNDOCK_STATUS":
                # Check 1: Success
                if not is_docked:
                    print("[Undocking] Dock status is FALSE. Success!")
                    state = "PATROLLING"
                    current_target = "A" # Reset target
                    continue
                
                # Check 2: Failure (Timeout)
                if time.time() - attempt_start_time > DOCK_TIMEOUT_SECONDS:
                    print(f"[Undocking] FAILED: Still docked after {DOCK_TIMEOUT_SECONDS}s.")
                    state = "HANDLING_UNDOCK_FAILURE"
                    continue
                
                print(f"[Undocking] Waiting for /dock_status to be false... ({time.time() - attempt_start_time:.0f}s)")

            elif state == "HANDLING_UNDOCK_FAILURE":
                print("[Undock Failure] undock.sh failed. Retrying script...")
                time.sleep(5.0) # Wait a few seconds
                state = "SENDING_UNDOCK_COMMAND" # Re-run the script

            time.sleep(2.0) # Main loop rate

    except KeyboardInterrupt:
        print("--- User interrupted test. Shutting down. ---")
    
    print("Shutting down navigation...")
    navigator.lifecycleShutdown()
    
    print("Saving final data chunk...")
    data_logger.save_final_chunk()
    
    print("Shutting down ROS nodes...")
    executor.shutdown()
    rclpy.shutdown()
    
    print("Test complete.")

if __name__ == '__main__':
    main()