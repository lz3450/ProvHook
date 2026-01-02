import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import numpy as np
import pandas as pd
import message_filters
import math
import joblib
from sklearn.preprocessing import StandardScaler
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor # Import the MultiThreadedExecutor
import threading

# --- All columns that will be saved in the FINAL processed CSV ---
# This includes raw odom, raw imu, raw cmd_vel, and calculated derivatives
FINAL_COLUMN_HEADERS = [
    "pos_x", "pos_y", "yaw",
    "linear_vel_x", "angular_vel_z",
    "cmd_linear_x", "cmd_angular_z",
    "imu_orientation_x", "imu_orientation_y", "imu_orientation_z", "imu_orientation_w",
    "imu_angular_velocity_x", "imu_angular_velocity_y", "imu_angular_velocity_z",
    "imu_linear_acceleration_x", "imu_linear_acceleration_y", "imu_linear_acceleration_z",
    "linear_accel", "angular_accel",
    "linear_jerk", "angular_jerk"
]

# Columns to be collected in real-time (derivatives are calculated at the end)
REALTIME_COLUMN_HEADERS = [
    "pos_x", "pos_y", "yaw",
    "linear_vel_x", "angular_vel_z",
    "cmd_linear_x", "cmd_angular_z",
    "imu_orientation_x", "imu_orientation_y", "imu_orientation_z", "imu_orientation_w",
    "imu_angular_velocity_x", "imu_angular_velocity_y", "imu_angular_velocity_z",
    "imu_linear_acceleration_x", "imu_linear_acceleration_y", "imu_linear_acceleration_z",
]

# Configuration for the EMA filter
SMOOTHING_SPAN = 1

class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')
        self.get_logger().info("Data Logger Node with IMU Started")
        
        self.data_buffer = []
        self.last_imu_msg = None
        self.buffer_lock = threading.Lock() # Lock for thread-safe buffer access

        qos_profile_best_effort = QoSProfile(          
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- Subscriber for low-frequency topics (20Hz) ---
        self.odom_sub = message_filters.Subscriber(
            self, Odometry, '/odom', qos_profile=qos_profile_best_effort
        )
        self.cmd_vel_sub = message_filters.Subscriber(
            self, Twist, '/cmd_vel', qos_profile=qos_profile_best_effort
        )

        # --- Independent Subscriber for high-frequency IMU topic (30Hz+) ---
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu', # <-- Confirm this is your IMU topic name
            self.imu_callback,
            qos_profile_best_effort
        )
        
        # --- TimeSynchronizer for low-frequency topics ONLY ---
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.odom_sub, self.cmd_vel_sub], queue_size=30, slop=0.1, allow_headerless=True
        )
        self.ts.registerCallback(self.synchronized_callback)

        self.get_logger().info("Subscribing to /odom, /cmd_vel, and /imu...")
        self.get_logger().info("Waiting for first messages...")

    def imu_callback(self, msg: Imu):
        """High-frequency callback. Caches the latest IMU message."""
        self.last_imu_msg = msg

    def synchronized_callback(self, odom_msg: Odometry, cmd_vel_msg: Twist):
        """
        Low-frequency callback (20Hz).
        Triggered when /odom and /cmd_vel are aligned.
        It then grabs the latest cached IMU message.
        """
        
        # If we haven't received any IMU data yet, wait.
        if self.last_imu_msg is None:
            if len(self.data_buffer) == 0:
                self.get_logger().warn("Waiting for first IMU message...")
            return

        if len(self.data_buffer) == 0:
             self.get_logger().info("First aligned message trifecta received! Starting data collection.")

        # 1. Odom Data
        pos_x = odom_msg.pose.pose.position.x
        pos_y = odom_msg.pose.pose.position.y
        _, _, yaw = self.euler_from_quaternion(odom_msg.pose.pose.orientation)
        linear_vel_x = odom_msg.twist.twist.linear.x
        angular_vel_z = odom_msg.twist.twist.angular.z

        # 2. Cmd_vel Data
        cmd_linear_x = cmd_vel_msg.linear.x
        cmd_angular_z = cmd_vel_msg.angular.z
        
        # 3. IMU Data (from cache)
        imu = self.last_imu_msg
        imu_orientation_x = imu.orientation.x
        imu_orientation_y = imu.orientation.y
        imu_orientation_z = imu.orientation.z
        imu_orientation_w = imu.orientation.w
        imu_angular_velocity_x = imu.angular_velocity.x
        imu_angular_velocity_y = imu.angular_velocity.y
        imu_angular_velocity_z = imu.angular_velocity.z
        imu_linear_acceleration_x = imu.linear_acceleration.x
        imu_linear_acceleration_y = imu.linear_acceleration.y
        imu_linear_acceleration_z = imu.linear_acceleration.z

        data_row = [
            pos_x, pos_y, yaw,
            linear_vel_x, angular_vel_z,
            cmd_linear_x, cmd_angular_z,
            imu_orientation_x, imu_orientation_y, imu_orientation_z, imu_orientation_w,
            imu_angular_velocity_x, imu_angular_velocity_y, imu_angular_velocity_z,
            imu_linear_acceleration_x, imu_linear_acceleration_y, imu_linear_acceleration_z
        ]
        
        with self.buffer_lock:
            self.data_buffer.append(data_row)

        if len(self.data_buffer) % 100 == 0:
            self.get_logger().info(f"Collected {len(self.data_buffer)} data points.")

    def euler_from_quaternion(self, q):
        t3 = +2.0 * (q.w * q.z + q.x * q.y); t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_z = math.atan2(t3, t4)
        return 0.0, 0.0, yaw_z # We only care about yaw

    def save_data(self):
        """
        Saves the collected data by running the full processing pipeline:
        1. Save raw data
        2. Filter velocities and calculate derivatives
        3. Save processed data
        4. Normalize processed data and save scaler
        5. Save normalized data
        """
        with self.buffer_lock:
            if not self.data_buffer:
                self.get_logger().warn("No data collected, nothing to save.")
                return
            
            self.get_logger().info(f"Preparing to save {len(self.data_buffer)} data points...")
            
            # Create DataFrame from the realtime buffer
            df_raw = pd.DataFrame(self.data_buffer, columns=REALTIME_COLUMN_HEADERS)
            
            # --- 1. Save Raw Data ---
            raw_path = "carter_data_raw.csv"
            df_raw.to_csv(raw_path, index=False)
            self.get_logger().info(f"Raw data saved to '{raw_path}'")
            
            # --- 2. Create Processed DataFrame ---
            df_processed = df_raw.copy()
            
            # 2a. Apply EMA Filter (from filter_and_process_data.py)
            self.get_logger().info(f"Applying EMA filter (span={SMOOTHING_SPAN})...")
            df_processed['linear_vel_x'] = df_processed['linear_vel_x'].ewm(span=SMOOTHING_SPAN, adjust=False).mean()
            df_processed['angular_vel_z'] = df_processed['angular_vel_z'].ewm(span=SMOOTHING_SPAN, adjust=False).mean()

            # 2b. Calculate Derivatives from Filtered Data
            self.get_logger().info("Calculating derivatives from filtered velocity...")
            df_processed['linear_accel'] = df_processed['linear_vel_x'].diff()
            df_processed['angular_accel'] = df_processed['angular_vel_z'].diff()
            df_processed['linear_jerk'] = df_processed['linear_accel'].diff()
            df_processed['angular_jerk'] = df_processed['angular_accel'].diff()
            df_processed.fillna(0, inplace=True) # Fill NaNs from .diff()

            # Re-order columns to match the new FINAL_COLUMN_HEADERS
            df_processed = df_processed[FINAL_COLUMN_HEADERS]
            
            # --- 3. Save Processed Data ---
            processed_path = "carter_data_processed.csv"
            df_processed.to_csv(processed_path, index=False)
            self.get_logger().info(f"Processed (un-normalized) data saved to '{processed_path}'")

            # --- 4. Normalize and Save Scaler ---
            self.get_logger().info("Normalizing data with StandardScaler...")
            scaler = StandardScaler()
            df_normalized_values = scaler.fit_transform(df_processed)
            df_normalized = pd.DataFrame(df_normalized_values, columns=FINAL_COLUMN_HEADERS)
            
            scaler_path = "carter_scaler.gz"
            joblib.dump(scaler, scaler_path)
            self.get_logger().info(f"Scaler saved to '{scaler_path}'")

            # --- 5. Save Normalized Data ---
            normalized_path = "carter_data_normalized.csv"
            df_normalized.to_csv(normalized_path, index=False)
            self.get_logger().info(f"Normalized data saved to '{normalized_path}'")
            self.get_logger().info("All data processing complete.")

def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    
    # --- CHANGE: Use a MultiThreadedExecutor ---
    # This allows each subscriber to have its own thread,
    # preventing the high-frequency IMU from blocking the odom/cmd_vel sync.
    executor = MultiThreadedExecutor(num_threads=4) # 4 threads: 3 for subs, 1 for system
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, saving data...")
    finally:
        node.save_data()
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()