#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import EstimatorStatusFlags, SensorGps
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
import time
from threading import Lock

class FaultDetector(Node):
    def __init__(self):
        super().__init__('fault_detector')

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('gps_timeout', 10.0), # Timeout for GPS data
                ('min_satellites', 6),
                ('min_fix_type', 3),  # 3D fix
                ('check_interval', 2.0),
                ('estimator_timeout', 10.0),
            ]
        )

        # Get parameters
        self.gps_timeout = self.get_parameter('gps_timeout').value
        self.min_satellites = self.get_parameter('min_satellites').value
        self.min_fix_type = self.get_parameter('min_fix_type').value
        self.check_interval = self.get_parameter('check_interval').value
        self.estimator_timeout = self.get_parameter('estimator_timeout').value

        # Log parameters
        self.get_logger().info(f'Parameters: gps_timeout={self.gps_timeout}s, '
                               f'min_satellites={self.min_satellites}, '
                               f'min_fix_type={self.min_fix_type}, '
                               f'check_interval={self.check_interval}s, '
                               f'estimator_timeout={self.estimator_timeout}s')

        # State variables
        self.gps_status = None # Stores SensorGps message
        self.estimator_status = None
        self.last_gps_time = None
        self.last_estimator_time = None
        self.mavros_state = None
        self.emergency_triggered = False
        self.data_lock = Lock()

        # Define QoS profile for PX4 topics (after 1st fix)
        px4_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        # Subscribers with PX4 QoS compatibility (dokimi)
        # Subscribe to SensorGps via vehicle_gps_position topic
        self.gps_sub = self.create_subscription(
            SensorGps,
            '/fmu/out/vehicle_gps_position',
            self.gps_callback,
            qos_profile=px4_qos_profile
        )

        # Subscribe to EstimatorStatusFlags topic that actually exists (dokimastiko fix)
        self.estimator_sub = self.create_subscription(
            EstimatorStatusFlags,
            '/fmu/out/estimator_status_flags',
            self.estimator_callback,
            qos_profile=px4_qos_profile
        )

        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )

        # Service clients
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Timer for periodic health checks
        self.timer = self.create_timer(self.check_interval, self.check_system_health)

        self.get_logger().info('FaultDetector node initialized with PX4 QoS compatibility')

    def gps_callback(self, msg):
        with self.data_lock:
            self.gps_status = msg
            # SensorGps has fix_type field - update time if GPS fix is adequate
            if msg.fix_type >= self.min_fix_type:
                self.last_gps_time = time.time()

    def estimator_callback(self, msg):
        with self.data_lock:
            self.estimator_status = msg
            self.last_estimator_time = time.time()

    def state_callback(self, msg):
        with self.data_lock:
            self.mavros_state = msg

    def check_system_health(self):
        """Main system health monitoring logic"""
        if self.emergency_triggered:
            return

        with self.data_lock:
            current_time = time.time()

            # Check GPS health
            gps_healthy = self.check_gps_health(current_time)

            # Check estimator health
            estimator_healthy = self.check_estimator_health(current_time)

            # Log system status
            gps_status_str = "HEALTHY" if gps_healthy else "DEGRADED"
            est_status_str = "HEALTHY" if estimator_healthy else "DEGRADED"

            self.get_logger().info(f'System Health - GPS: {gps_status_str}, Estimator: {est_status_str}')

            # Trigger emergency landing if critical systems are compromised(not sure yet, check again)
            if not gps_healthy:
                if self.gps_status is None:
                    self.trigger_emergency_landing("No GPS data received")
                elif self.gps_status.fix_type < 2:
                    self.trigger_emergency_landing(f"GPS fix lost (fix_type={self.gps_status.fix_type})")
                elif self.gps_status.satellites_used < self.min_satellites:
                    self.trigger_emergency_landing(f"Low satellite count ({self.gps_status.satellites_used})")
                elif self.last_gps_time is None or (current_time - self.last_gps_time > self.gps_timeout):
                    self.trigger_emergency_landing("GPS data timeout")
                else:
                    self.get_logger().warn("GPS unhealthy but not critical for immediate landing.")

            if not estimator_healthy:
                if self.estimator_status is None:
                    self.trigger_emergency_landing("No Estimator Status data received")
                elif self.last_estimator_time is None or (current_time - self.last_estimator_time > self.estimator_timeout):
                    self.trigger_emergency_landing("Estimator Status data timeout")
                else:
                    self.trigger_emergency_landing("Estimator flags indicate degraded position estimate")

    def check_gps_health(self, current_time):
        """Check GPS health status using SensorGps"""
        if self.gps_status is None:
            return False

        # Check if GPS fix is available (I am not sure why this is needed, but it is in the original code)
        # Check fix type - need at least 2D fix (FIX_TYPE_2D = 2)
        if self.gps_status.fix_type < self.min_fix_type:
            return False

        # Check number of satellites used
        if self.gps_status.satellites_used < self.min_satellites:
            return False

        # Check GPS timeout
        if self.last_gps_time is None or (current_time - self.last_gps_time > self.gps_timeout):
            return False

        # Check for jamming or spoofing (optional additional safety)
        if hasattr(self.gps_status, 'jamming_state') and self.gps_status.jamming_state == 3:  # CRITICAL
            return False
        
        if hasattr(self.gps_status, 'spoofing_state') and self.gps_status.spoofing_state >= 2:  # INDICATED or MULTIPLE
            return False

        return True

    def check_estimator_health(self, current_time):
        """Check estimator health status using EstimatorStatusFlags"""
        if self.estimator_status is None:
            return False

        # Timeout check for estimator status
        if self.last_estimator_time is None or (current_time - self.last_estimator_time > self.estimator_timeout):
            return False

        # Check GPS-related flags from gps_check_fail_flags
        if hasattr(self.estimator_status, 'gps_check_fail_flags'):
            gps_flags = self.estimator_status.gps_check_fail_flags
            # Check if GPS fix is insufficient (bit 0)
            if gps_flags & (1 << 0):  # GPS_CHECK_FAIL_GPS_FIX
                return False
            # Check if satellite count is insufficient (bit 1)
            if gps_flags & (1 << 1):  # GPS_CHECK_FAIL_MIN_SAT_COUNT
                return False
            # Check position accuracy issues (bits 3,4)
            if gps_flags & (1 << 3) or gps_flags & (1 << 4):  # HORZ_ERR or VERT_ERR
                return False

        # Check control mode flags if available
        if hasattr(self.estimator_status, 'control_mode_flags'):
            control_flags = self.estimator_status.control_mode_flags
            # Check if GPS is being used (bit 2)
            gps_active = control_flags & (1 << 2)  # CS_GPS
            if not gps_active:
                return False

        return True

    def trigger_emergency_landing(self, reason):
        """Trigger emergency landing"""
        if self.emergency_triggered:
            return

        self.emergency_triggered = True
        self.get_logger().error(f'EMERGENCY LANDING TRIGGERED: {reason}')

        if self.set_mode_client.wait_for_service(timeout_sec=5.0):
            request = SetMode.Request()
            request.custom_mode = 'AUTO.LAND'

            future = self.set_mode_client.call_async(request)
            future.add_done_callback(self.emergency_response_callback)
        else:
            self.get_logger().error('Failed to connect to set_mode service')

    def emergency_response_callback(self, future):
        """Handle emergency landing service response"""
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info('Emergency landing mode successfully set')
            else:
                self.get_logger().error('Failed to set emergency landing mode')
                self.emergency_triggered = False  # Allow retry
        except Exception as e:
            self.get_logger().error(f'Emergency landing service call failed: {e}')
            self.emergency_triggered = False  # Allow retry

def main(args=None):
    rclpy.init(args=args)
    node = FaultDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down FaultDetector...')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()