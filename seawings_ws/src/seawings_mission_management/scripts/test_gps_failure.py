#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleGlobalPosition
import time

class GpsFailureSimulator(Node):
    def __init__(self):
        super().__init__('gps_failure_simulator')
        
        # Publisher for GPS position
        self.gps_pub = self.create_publisher(
            VehicleGlobalPosition,
            '/fmu/out/vehicle_global_position',
            10
        )
        
        # Timer to simulate GPS failure
        self.timer = self.create_timer(1.0, self.simulate_gps_failure)
        
        # Simulation parameters
        self.simulation_time = 0
        self.failure_start_time = 30  # Start failure after 30 seconds
        self.failure_duration = 20    # Failure lasts 20 seconds
        
        self.get_logger().info('GPS failure simulator started')
        
    def simulate_gps_failure(self):
        """Simulate GPS failure after certain time"""
        self.simulation_time += 1
        
        # Create GPS message
        msg = VehicleGlobalPosition()
        msg.timestamp = int(time.time() * 1000000)
        
        # Simulate GPS failure window
        if (self.failure_start_time <= self.simulation_time <= 
            self.failure_start_time + self.failure_duration):
            # GPS failure - no fix
            msg.fix_type = 0  # No fix
            msg.satellites_used = 0
            msg.lat = 0
            msg.lon = 0
            msg.alt = 0
            status = "GPS FAILURE"
        else:
            # Normal GPS operation
            msg.fix_type = 3  # 3D fix
            msg.satellites_used = 8
            msg.lat = int(47.397742 * 1e7)  # Example coordinates
            msg.lon = int(8.545594 * 1e7)
            msg.alt = int(488.0 * 1e3)
            status = "GPS OK"
            
        self.gps_pub.publish(msg)
        
        self.get_logger().info(f'Time: {self.simulation_time}s, Status: {status}, '
                              f'Satellites: {msg.satellites_used}, Fix: {msg.fix_type}')

def main(args=None):
    rclpy.init(args=args)
    node = GpsFailureSimulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
