#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, TwistStamped
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import time
import numpy as np

class JoystickWatchdog(Node):
    def __init__(self):
        super().__init__('joystick_watchdog')
        
        # Declare parameters
        self.declare_parameter('timeout', 0.5,
                           ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                               description='Maximum time (seconds) to wait for joystick message before considering it inactive'))
        self.declare_parameter('use_stamped_twist', True,
                           ParameterDescriptor(type=ParameterType.PARAMETER_BOOL,
                                               description='Whether to publish TwistStamped or Twist messages'))
        self.declare_parameter('output_topic', '/bicycle_steering_controller/reference',
                           ParameterDescriptor(type=ParameterType.PARAMETER_STRING,
                                               description='Topic to publish control messages'))
        self.declare_parameter('input_topic', '/teleop_twist_joy/cmd_vel',
                           ParameterDescriptor(type=ParameterType.PARAMETER_STRING,
                                               description='Topic to subscribe for input twist messages'))
        self.declare_parameter('max_identical_msgs', 3,
                           ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER,
                                               description='Maximum number of identical joy messages before considering the joystick halted'))

        self.declare_parameter('debug', True,
                           ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER,
                                               description='Show debug'))
                           
        # Get parameter values
        self.timeout = self.get_parameter('timeout').value
        self.use_stamped_twist = self.get_parameter('use_stamped_twist').value
        self.output_topic = self.get_parameter('output_topic').value
        self.input_topic = self.get_parameter('input_topic').value
        self.max_identical_msgs = self.get_parameter('max_identical_msgs').value
        self.show_debug = self.get_parameter('debug').value
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Initialize variables to track joystick status
        self.last_joy_time = time.time()
        self.last_joy_msgs = []  
        self.joy_identical_count = 0
        
        # Initialize command variables
        self.current_twist = None
        
        # Create subscribers
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            qos_profile
        )
        
        # Determine input message type based on use_stamped_twist parameter
        if self.use_stamped_twist:
            self.twist_sub = self.create_subscription(
                TwistStamped,
                self.input_topic,
                self.twist_stamped_callback,
                qos_profile
            )
            self.twist_pub = self.create_publisher(
                TwistStamped,
                self.output_topic,
                qos_profile
            )
        else:
            self.twist_sub = self.create_subscription(
                Twist,
                self.input_topic,
                self.twist_callback,
                qos_profile
            )
            self.twist_pub = self.create_publisher(
                Twist,
                self.output_topic,
                qos_profile
            )
        
        # Create watchdog timer
        self.timer = self.create_timer(0.01, self.watchdog_callback)
        
        self.get_logger().info('Joystick Watchdog Node initialized')
        if self.show_debug:
            self.get_logger().info(f'Output topic: {self.output_topic}')
            self.get_logger().info(f'Using {"TwistStamped" if self.use_stamped_twist else "Twist"} messages')
            self.get_logger().info(f'Joystick timeout: {self.timeout} seconds')
            self.get_logger().info(f'Max identical messages: {self.max_identical_msgs}')
    
    def joy_arrays_equal(self, joy1, joy2):
        """Check if two joy messages have identical axes and buttons values"""
        if joy1 is None or joy2 is None:
            return False
            
        # Compare axes and buttons arrays
        axes_equal = np.array_equal(joy1.axes, joy2.axes)
        buttons_equal = np.array_equal(joy1.buttons, joy2.buttons)
        
        return axes_equal and buttons_equal
    
    def joy_callback(self, msg):
        self.last_joy_time = time.time()
        
        # Check if this message is identical to the previous one
        if len(self.last_joy_msgs) > 0 and self.joy_arrays_equal(msg, self.last_joy_msgs[-1]):
            self.joy_identical_count += 1
            if self.show_debug:
                self.get_logger().warn(f'Identical joy message detected: {self.joy_identical_count}/{self.max_identical_msgs}')
        else:
            # Reset counter if we get a different message
            self.joy_identical_count = 0
        
        # Keep track of recent messages
        self.last_joy_msgs.append(msg)
        if len(self.last_joy_msgs) > self.max_identical_msgs:
            self.last_joy_msgs.pop(0)  
        
    def twist_callback(self, msg):
        self.current_twist = msg
        
    def twist_stamped_callback(self, msg):
        self.current_twist = msg
        
    def is_joystick_active(self):
        # Check time-based activity
        current_time = time.time()
        time_active = (current_time - self.last_joy_time) < self.timeout
        
        # Check for repeated identical messages
        repeat_active = self.joy_identical_count < self.max_identical_msgs
        
        # Joystick is considered active only if both conditions are met
        return time_active and repeat_active
        
    def watchdog_callback(self):
        # Check if joystick is active
        joystick_active = self.is_joystick_active()
        
        if not joystick_active:
            # Joystick inactive - publish zero velocity
            if self.use_stamped_twist:
                zero_msg = TwistStamped()
                zero_msg.header.stamp = self.get_clock().now().to_msg()
                zero_msg.header.frame_id = 'base_link'  # Use appropriate frame
            else:
                zero_msg = Twist()
                
            self.twist_pub.publish(zero_msg)
            
            if self.joy_identical_count >= self.max_identical_msgs:
                if self.show_debug:
                    self.get_logger().warn(f'Joystick sending identical messages: {self.joy_identical_count}/{self.max_identical_msgs}')
            elif (time.time() - self.last_joy_time) >= self.timeout:
                if self.show_debug:
                    self.get_logger().warn(f'Joystick inactive for {self.timeout} seconds')
            
        elif self.current_twist is not None:
            self.twist_pub.publish(self.current_twist)
            
        else:
            # Joystick active but no twist command received yet
            self.get_logger().debug('Waiting for first twist command')

def main(args=None):
    rclpy.init(args=args)
    node = JoystickWatchdog()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()