#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

class BNO055IMUGravityNode(Node):
    def __init__(self):
        super().__init__('bno055_imu_gravity_node')
        
        # Store the latest gravity vector
        self.latest_gravity = None
        
        # Create subscribers
        self.imu_sub = self.create_subscription(
            Imu, 
            '/bno055/imu', 
            self.imu_callback, 
            10)
            
        self.grav_sub = self.create_subscription(
            Vector3, 
            '/bno055/grav', 
            self.grav_callback, 
            10)
        
        # Publisher for the combined IMU message
        self.imu_grav_pub = self.create_publisher(
            Imu, 
            '/bno055/imu_grav', 
            10)
        
        self.get_logger().info('BNO055 IMU Gravity Node started')

    def grav_callback(self, grav_msg):
        # Store the latest gravity vector
        self.latest_gravity = grav_msg
        # self.get_logger().debug(f'Received gravity Z: {grav_msg.z}')

    def imu_callback(self, imu_msg):
        # Only process if we have received gravity data
        if self.latest_gravity is None:
            self.get_logger().warn('No gravity data received yet')
            return
            
        # Create a new IMU message
        imu_grav_msg = Imu()
        
        # Copy all data from the original IMU message
        imu_grav_msg.header = imu_msg.header
        imu_grav_msg.orientation = imu_msg.orientation
        imu_grav_msg.orientation_covariance = imu_msg.orientation_covariance
        imu_grav_msg.angular_velocity = imu_msg.angular_velocity
        imu_grav_msg.angular_velocity_covariance = imu_msg.angular_velocity_covariance
        
        # Copy X and Y acceleration from IMU, but use Z from gravity
        imu_grav_msg.linear_acceleration.x = imu_msg.linear_acceleration.x
        imu_grav_msg.linear_acceleration.y = imu_msg.linear_acceleration.y
        imu_grav_msg.linear_acceleration.z = self.latest_gravity.z
        imu_grav_msg.linear_acceleration_covariance = imu_msg.linear_acceleration_covariance
        
        # Publish the combined message
        self.imu_grav_pub.publish(imu_grav_msg)
        
        # self.get_logger().debug(f'Published IMU with gravity Z: {self.latest_gravity.z}')

def main(args=None):
    rclpy.init(args=args)
    node = BNO055IMUGravityNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()