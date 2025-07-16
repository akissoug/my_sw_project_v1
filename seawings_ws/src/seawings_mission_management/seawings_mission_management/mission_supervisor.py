#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, BatteryStatus
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import String
import time
from threading import Lock
from enum import Enum

class MissionState(Enum):
    IDLE = 0
    ARMED = 1
    TAKEOFF = 2
    MISSION = 3
    RTL = 4
    LANDING = 5
    EMERGENCY = 6
    DISARMED = 7

class MissionSupervisor(Node):
    def __init__(self):
        super().__init__('mission_supervisor')
        
        # Declare parameters
        self.declare_parameter('heartbeat_timeout', 30.0)  # seconds
        self.declare_parameter('telemetry_timeout', 10.0)  # seconds
        self.declare_parameter('mission_timeout', 3600.0)  # seconds (1 hour)
        self.declare_parameter('check_interval', 1.0)  # seconds
        
        # Get parameters
        self.heartbeat_timeout = self.get_parameter('heartbeat_timeout').value
        self.telemetry_timeout = self.get_parameter('telemetry_timeout').value
        self.mission_timeout = self.get_parameter('mission_timeout').value
        self.check_interval = self.get_parameter('check_interval').value
        
        # State variables
        self.mission_state = MissionState.IDLE
        self.vehicle_status = None
        self.mavros_state = None
        self.extended_state = None
        self.battery_status = None
        self.last_heartbeat = None
        self.last_telemetry = None
        self.mission_start_time = None
        self.emergency_actions_taken = []
        self.data_lock = Lock()
        
        # Define QoS profile for PX4 topics
        px4_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Subscribers with PX4 QoS compatibility
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile=px4_qos_profile
        )
        
        self.battery_sub = self.create_subscription(
            BatteryStatus,
            '/fmu/out/battery_status',
            self.battery_callback,
            qos_profile=px4_qos_profile
        )
        
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )
        
        self.extended_state_sub = self.create_subscription(
            ExtendedState,
            '/mavros/extended_state',
            self.extended_state_callback,
            10
        )
        
        # Publishers
        self.mission_status_pub = self.create_publisher(
            String,
            '/seawings/mission_status',
            10
        )
        
        # Service clients
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        
        # Timer for periodic mission supervision
        self.timer = self.create_timer(self.check_interval, self.supervise_mission)
        
        self.get_logger().info('MissionSupervisor node initialized with PX4 QoS compatibility')
        
    def vehicle_status_callback(self, msg):
        with self.data_lock:
            self.vehicle_status = msg
            self.last_telemetry = time.time()
            
    def battery_callback(self, msg):
        with self.data_lock:
            self.battery_status = msg
            
    def state_callback(self, msg):
        with self.data_lock:
            self.mavros_state = msg
            self.last_heartbeat = time.time()
            
    def extended_state_callback(self, msg):
        with self.data_lock:
            self.extended_state = msg
            
    def supervise_mission(self):
        """Main mission supervision logic"""
        with self.data_lock:
            current_time = time.time()
            
            # Update mission state based on vehicle status
            self.update_mission_state()
            
            # Check for communication timeouts
            self.check_communication_health(current_time)
            
            # Check mission duration
            self.check_mission_duration(current_time)
            
            # Check for emergency conditions
            self.check_emergency_conditions()
            
            # Publish mission status
            self.publish_mission_status()
            
            # Log mission state
            self.log_mission_state()
            
    def update_mission_state(self):
        """Update internal mission state based on vehicle status"""
        if self.vehicle_status is None or self.mavros_state is None:
            return
            
        old_state = self.mission_state
        
        # State machine logic
        if not self.mavros_state.connected:
            self.mission_state = MissionState.IDLE
        elif not self.mavros_state.armed:
            self.mission_state = MissionState.DISARMED
        elif self.mavros_state.mode == 'AUTO.TAKEOFF':
            self.mission_state = MissionState.TAKEOFF
        elif self.mavros_state.mode in ['AUTO.MISSION', 'AUTO.LOITER']:
            self.mission_state = MissionState.MISSION
            if self.mission_start_time is None:
                self.mission_start_time = time.time()
        elif self.mavros_state.mode == 'AUTO.RTL':
            self.mission_state = MissionState.RTL
        elif self.mavros_state.mode in ['AUTO.LAND', 'AUTO.PRECLAND']:
            self.mission_state = MissionState.LANDING
        elif self.mavros_state.mode in ['STABILIZED', 'ALTITUDE', 'POSITION']:
            self.mission_state = MissionState.EMERGENCY
        else:
            self.mission_state = MissionState.ARMED
            
        # Log state transitions
        if old_state != self.mission_state:
            self.get_logger().info(f'Mission state changed: {old_state.name} -> {self.mission_state.name}')
            
    def check_communication_health(self, current_time):
        """Check communication link health"""
        # Check MAVROS heartbeat
        if self.last_heartbeat is not None:
            heartbeat_age = current_time - self.last_heartbeat
            if heartbeat_age > self.heartbeat_timeout:
                self.handle_communication_loss("MAVROS heartbeat timeout")
                
        # Check telemetry
        if self.last_telemetry is not None:
            telemetry_age = current_time - self.last_telemetry
            if telemetry_age > self.telemetry_timeout:
                self.handle_communication_loss("Telemetry timeout")
                
    def check_mission_duration(self, current_time):
        """Check if mission has exceeded maximum duration"""
        if self.mission_start_time is not None:
            mission_duration = current_time - self.mission_start_time
            if mission_duration > self.mission_timeout:
                self.handle_mission_timeout()
                
    def check_emergency_conditions(self):
        """Check for emergency conditions requiring immediate action"""
        if self.battery_status is None:
            return
            
        # Check for critical battery level
        if self.battery_status.remaining < 5.0:  # 5% battery remaining
            self.handle_critical_battery()
            
        # Check for other emergency conditions
        if self.vehicle_status is not None:
            # Check for failsafe activation
            if self.vehicle_status.failsafe:
                self.handle_failsafe_activation()
                
    def handle_communication_loss(self, reason):
        """Handle communication loss scenarios"""
        if "comm_loss" not in self.emergency_actions_taken:
            self.emergency_actions_taken.append("comm_loss")
            self.get_logger().error(f'Communication loss detected: {reason}')
            
            # Trigger RTL if not already in emergency mode
            if self.mission_state not in [MissionState.RTL, MissionState.LANDING, MissionState.EMERGENCY]:
                self.trigger_mode_change('AUTO.RTL', 'Communication loss')
                
    def handle_mission_timeout(self):
        """Handle mission timeout"""
        if "mission_timeout" not in self.emergency_actions_taken:
            self.emergency_actions_taken.append("mission_timeout")
            self.get_logger().warn('Mission timeout - triggering RTL')
            self.trigger_mode_change('AUTO.RTL', 'Mission timeout')
            
    def handle_critical_battery(self):
        """Handle critical battery level"""
        if "critical_battery" not in self.emergency_actions_taken:
            self.emergency_actions_taken.append("critical_battery")
            self.get_logger().error('CRITICAL BATTERY - triggering emergency landing')
            self.trigger_mode_change('AUTO.LAND', 'Critical battery')
            
    def handle_failsafe_activation(self):
        """Handle PX4 failsafe activation"""
        if "failsafe" not in self.emergency_actions_taken:
            self.emergency_actions_taken.append("failsafe")
            self.get_logger().error('PX4 failsafe activated')
            
    def trigger_mode_change(self, mode, reason):
        """Trigger a mode change via MAVROS"""
        self.get_logger().info(f'Triggering mode change to {mode}: {reason}')
        
        if self.set_mode_client.wait_for_service(timeout_sec=5.0):
            request = SetMode.Request()
            request.custom_mode = mode
            
            future = self.set_mode_client.call_async(request)
            future.add_done_callback(lambda f: self.mode_change_callback(f, mode, reason))
        else:
            self.get_logger().error('Failed to connect to set_mode service')
            
    def mode_change_callback(self, future, mode, reason):
        """Handle mode change service response"""
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info(f'Mode change to {mode} successful: {reason}')
            else:
                self.get_logger().error(f'Failed to change mode to {mode}: {reason}')
        except Exception as e:
            self.get_logger().error(f'Mode change service call failed: {e}')
            
    def publish_mission_status(self):
        """Publish current mission status"""
        msg = String()
        msg.data = f'{self.mission_state.name}'
        self.mission_status_pub.publish(msg)
        
    def log_mission_state(self):
        """Log detailed mission state information"""
        if self.mission_state == MissionState.MISSION and self.mission_start_time is not None:
            mission_duration = time.time() - self.mission_start_time
            battery_pct = self.battery_status.remaining if self.battery_status else 0
            
            self.get_logger().info(
                f'Mission Status - State: {self.mission_state.name}, '
                f'Duration: {mission_duration:.1f}s, '
                f'Battery: {battery_pct:.1f}%, '
                f'Mode: {self.mavros_state.mode if self.mavros_state else "Unknown"}'
            )

def main(args=None):
    rclpy.init(args=args)
    node = MissionSupervisor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()