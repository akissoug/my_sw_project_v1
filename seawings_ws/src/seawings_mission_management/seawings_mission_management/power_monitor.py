#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import BatteryStatus, VehicleGlobalPosition
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
import math
from threading import Lock
from collections import deque
import time

class PowerMonitor(Node):
    def __init__(self):
        super().__init__('power_monitor')

        # Parameters
        self.declare_parameter('safety_margin', 0.3)  # 30% - increased for safety
        self.declare_parameter('average_return_speed', 8.0)  # m/s - conservative estimate
        self.declare_parameter('battery_check_interval', 3.0)  # seconds - more frequent
        self.declare_parameter('min_battery_voltage', 14.0)  # volts
        self.declare_parameter('rtl_triggered_threshold', 0.25)  # 25% battery
        self.declare_parameter('battery_capacity_mah', 5000.0)  # Fallback capacity
        self.declare_parameter('min_current_threshold', 0.1)  # Minimum current for calculations (A)
        self.declare_parameter('current_averaging_window', 10)  # Number of samples for averaging
        self.declare_parameter('home_position_timeout', 30.0)  # Max time to wait for home position

        self.safety_margin = self.get_parameter('safety_margin').value
        self.return_speed = self.get_parameter('average_return_speed').value
        self.check_interval = self.get_parameter('battery_check_interval').value
        self.rtl_threshold = self.get_parameter('rtl_triggered_threshold').value
        self.battery_capacity = self.get_parameter('battery_capacity_mah').value
        self.min_current = self.get_parameter('min_current_threshold').value
        self.current_window_size = self.get_parameter('current_averaging_window').value
        self.home_timeout = self.get_parameter('home_position_timeout').value

        # State
        self.battery_status = None
        self.global_position = None
        self.home_position = None  # (lat, lon, alt)
        self.mavros_state = None
        self.rtl_triggered = False
        self.lock = Lock()
        self.start_time = time.time()
        
        # Current averaging for more stable calculations
        self.current_history = deque(maxlen=self.current_window_size)

        # Define QoS profile for PX4 topics
        px4_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        # Subscribers with PX4 QoS compatibility
        self.create_subscription(BatteryStatus, '/fmu/out/battery_status', self.battery_callback, qos_profile=px4_qos_profile)
        self.create_subscription(VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.gps_callback, qos_profile=px4_qos_profile)
        self.create_subscription(State, '/mavros/state', self.state_callback, 10)

        # Service client
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Periodic check
        self.create_timer(self.check_interval, self.check_battery_status)

        self.get_logger().info('✅ PowerMonitor node initialized with PX4 QoS compatibility')

    def battery_callback(self, msg):
        with self.lock:
            self.battery_status = msg
            # Store current history for averaging
            if msg.current_a > 0:
                self.current_history.append(msg.current_a)

    def gps_callback(self, msg):
        with self.lock:
            self.global_position = msg
            # Better home position detection - only set if we have good GPS
            if (self.home_position is None and 
                not msg.dead_reckoning and 
                abs(msg.lat) > 0.001 and abs(msg.lon) > 0.001 and  # Not zero coordinates
                abs(msg.lat) <= 90.0 and abs(msg.lon) <= 180.0):  # Valid coordinate range
                
                self.home_position = (msg.lat, msg.lon, msg.alt)
                self.get_logger().info(f'🏠 Home position set: lat={msg.lat:.6f}, lon={msg.lon:.6f}, alt={msg.alt:.2f} m')

    def state_callback(self, msg):
        with self.lock:
            self.mavros_state = msg

        # This method of distance calculation is more accurate for flights mor than 5Kms compared to Eucledian distance
    def haversine_distance(self, lat1, lon1, lat2, lon2):
        R = 6371000  # Earth radius (m)
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        d_phi = math.radians(lat2 - lat1)
        d_lambda = math.radians(lon2 - lon1)

        a = math.sin(d_phi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return R * c

    def calculate_distance_to_home(self):
        if self.home_position is None or self.global_position is None:
            return None
        
        lat1, lon1, alt1 = self.home_position
        lat2, lon2, alt2 = self.global_position.lat, self.global_position.lon, self.global_position.alt

        # Calculate horizontal distance
        horizontal = self.haversine_distance(lat1, lon1, lat2, lon2)
        
        # For return time, use horizontal distance + altitude consideration
        # (drone will likely return at current altitude then descend)
        vertical = abs(alt2 - alt1)
        
        # Return conservative estimate - horizontal distance + some vertical component
        return horizontal + (vertical * 0.5)  # Assume 50% of vertical distance adds to travel

    def get_average_current(self):
        """Get averaged current draw for more stable calculations"""
        if not self.current_history:
            return None
        return sum(self.current_history) / len(self.current_history)

    def calculate_remaining_time(self):
        if self.battery_status is None:
            return None

        # Use averaged current for stability
        avg_current = self.get_average_current()
        if avg_current is None or avg_current < self.min_current:
            return None

        # Use actual battery capacity if available, otherwise use parameter
        if hasattr(self.battery_status, 'capacity') and self.battery_status.capacity > 0:
            capacity = self.battery_status.capacity
        else:
            capacity = self.battery_capacity
            

        percent = self.battery_status.remaining
        remaining_mah = (percent / 100.0) * capacity
        current_draw_ma = avg_current * 1000.0

        time_hours = remaining_mah / current_draw_ma
        return time_hours * 3600.0  # seconds

    def check_battery_status(self):
        with self.lock:
            if self.rtl_triggered:
                return

            # Check if we're still waiting for home position
            if self.home_position is None:
                elapsed = time.time() - self.start_time
                if elapsed > self.home_timeout:
                    self.get_logger().warn(f'⚠️ No home position after {self.home_timeout}s - using current position')
                    if self.global_position is not None:
                        self.home_position = (self.global_position.lat, self.global_position.lon, self.global_position.alt)
                return

            # Skip if not in appropriate flight mode
            if (self.mavros_state is None or 
                self.mavros_state.mode in ['AUTO.RTL', 'AUTO.LAND', 'MANUAL']):
                return

            if not all([self.battery_status, self.global_position]):
                return

            remaining_time = self.calculate_remaining_time()
            distance = self.calculate_distance_to_home()

            if remaining_time is None or distance is None:
                self.get_logger().warn('⚠️ Cannot calculate battery remaining time or distance')
                return

            # Calculate return time with safety margin
            base_return_time = distance / self.return_speed
            safety_return_time = base_return_time * (1 + self.safety_margin)

            # Add extra buffer for altitude changes and maneuvering
            safety_return_time += 60.0  # Extra 60 seconds buffer

            self.get_logger().info(
                f'🔋 Battery: {self.battery_status.remaining:.1f}%, '
                f'⏳ Remaining: {remaining_time:.1f}s, '
                f'🏠 Distance: {distance:.1f}m, '
                f'🔁 Needed: {safety_return_time:.1f}s, '
                f'⚡ Current: {self.get_average_current():.2f}A'
            )

            # Trigger RTL if either condition is met
            if (remaining_time < safety_return_time or 
                self.battery_status.remaining < self.rtl_threshold * 100):
                
                reason = "insufficient time" if remaining_time < safety_return_time else "low battery threshold"
                self.trigger_rtl(reason)

    def trigger_rtl(self, reason="battery low"):
        if self.rtl_triggered:
            return
        
        self.rtl_triggered = True
        self.get_logger().warn(f'⚠️ LOW BATTERY - Triggering Return-to-Launch: {reason}')
        
        if self.set_mode_client.wait_for_service(timeout_sec=5.0):
            request = SetMode.Request()
            request.custom_mode = 'AUTO.RTL'
            future = self.set_mode_client.call_async(request)
            future.add_done_callback(self.rtl_response_callback)
        else:
            self.get_logger().error('❌ SetMode service unavailable!')

    def rtl_response_callback(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info('✅ RTL mode successfully set')
            else:
                self.get_logger().error('❌ Failed to set RTL mode')
                self.rtl_triggered = False  # Allow retry
        except Exception as e:
            self.get_logger().error(f'❌ RTL service call failed: {e}')
            self.rtl_triggered = False  # Allow retry

def main(args=None):
    rclpy.init(args=args)
    node = PowerMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()