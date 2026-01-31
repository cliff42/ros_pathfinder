#!/usr/bin/env python3

import time
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from action_interfaces.action import MotorControl
from std_msgs.msg import Float64, Float64MultiArray
import math
import threading

class MotorControlServer(Node):
    prev_angle_l = 0
    prev_angle_r = 0
    distance_l = 0
    distance_r = 0

    MOTOR_SPEED = 0.5 # TODO: placeholder for now
    DISTANCE_TOLERANCE = 0.01

    def __init__(self):
        super().__init__('controller_node')
        self.cb_group = ReentrantCallbackGroup() # allow for concurrent callbacks
        self.left_motor_publisher = self.create_publisher(Float64, 'left_motor', 10)
        self.right_motor_publisher = self.create_publisher(Float64, 'right_motor', 10)
        self.imu_subscription = self.create_subscription(Float64MultiArray, 'test_topic', self.imu_listener_callback, 10, callback_group=self.cb_group)
        self.data_lock = threading.Lock()
        self._action_server = ActionServer(
            self,
            MotorControl,
            'motor_control',
            self.execute_callback,
            callback_group=self.cb_group)


    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing goal... {goal_handle.request.plan}')
        starting_distance = self.distance_l # TODO: change to use combined distance
        direction = goal_handle.request.plan[0]
        distance_goal = goal_handle.request.plan[1]

        with self.data_lock:
            start_l = self.distance_l
            start_r = self.distance_r

        feedback_msg = MotorControl.Feedback()
        
        while rclpy.ok():
            # catch case for cancelled request
            if goal_handle.is_cancel_requested:
                self.publish_to_motors(0.0, 0.0)
                goal_handle.canceled()
                return MotorControl.Result(success=False)
            
            with self.data_lock:
                dl = self.distance_l - start_l
                dr = self.distance_r - start_r
            
            progress = 0.5 * (dl + dr) # TODO: avg for now but should make this more complicated later
            remaining = max(0.0, distance_goal - progress)

            if remaining <= self.DISTANCE_TOLERANCE:
                break
                
            self.publish_to_motors(self.MOTOR_SPEED, self.MOTOR_SPEED)
            feedback_msg.distance_remaining = remaining
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info('Feedback: {0}'.format(feedback_msg.distance_remaining))

            # time.sleep(0.05) # TODO: update/ remove this -> should be same hz as publisher
        
        self.get_logger().info(f'here2... {self.distance_l - starting_distance }')
        # stop motors after goal distance
        self.publish_to_motors(0.0, 0.0)
        goal_handle.succeed()
        result = MotorControl.Result()
        result.success = True
        return result
    
    def imu_listener_callback(self, msg):
        self.get_logger().info(f"here in callback: {msg}")
        with self.data_lock:
            angle_l = float(msg.data[0])
            angle_r = float(msg.data[1])
            self.distance_l, self.prev_angle_l = self.get_distance(angle_l,self.prev_angle_l,self.distance_l)
            self.distance_r, self.prev_angle_r = self.get_distance(angle_r,self.prev_angle_r,self.distance_r)
        self.get_logger().info(f"distance_l: {self.distance_l}")


    def get_distance(self,angle,prev_angle, distance):
        delta_angle = 0
        if prev_angle > 270 and angle < 90:
            delta_angle = angle + 360 - prev_angle
        elif prev_angle < 90 and angle > 270:
            delta_angle = 360 - angle + prev_angle
        else:
            delta_angle = angle - prev_angle
        distance += (delta_angle/7) * (math.pi/180)*4*0.0254
        prev_angle = angle
        return distance, prev_angle

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
            planner_action_server = MotorControlServer() # allow for running multiple callbacks in parallel
            executor = MultiThreadedExecutor()
            executor.add_node(planner_action_server)
            executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == "__main__":
    main()