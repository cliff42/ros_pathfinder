#!/usr/bin/env python3

import time
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.action import ActionServer
from action_interfaces.action import MotorControl
from std_msgs.msg import Float64, Float64MultiArray

class MotorControlServer(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.left_motor_publisher = self.create_publisher(Float64, 'left_motor', 10)
        self.right_motor_publisher = self.create_publisher(Float64, 'right_motor', 10)
        self.imu_subscription = self.create_subscription(Float64MultiArray, 'imu_topic', 10)
        self._action_server = ActionServer(
            self,
            MotorControl,
            'motor_control',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing goal... {goal_handle.request.plan}')
        self.publish_to_motors(goal_handle.request.plan[0], goal_handle.request.plan[1])
        goal_handle.succeed()
        result = MotorControl.Result()
        result.success = True
        return result
    
    def publish_to_motors(self, left_speed, right_speed):
        left_msg = Float64()
        left_msg.data = left_speed
        right_msg = Float64()
        right_msg.data = right_speed
        self.left_motor_publisher.publish(left_msg)
        self.right_motor_publisher.publish(right_msg)

class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")
        self.flag = True
        #TODO
        pass

def main(args=None):
    try:
        with rclpy.init(args=args):
            planner_action_server = MotorControlServer()

            rclpy.spin(planner_action_server)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == "__main__":
    main()