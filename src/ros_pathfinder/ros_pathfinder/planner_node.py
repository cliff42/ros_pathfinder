import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from action_interfaces.action import MotorControl

class MotorControlClient(Node):

    def __init__(self):
        super().__init__('planner_node')
        self._action_client = ActionClient(self, MotorControl, 'motor_control')

    def send_goal(self, plan):
        goal_msg = MotorControl.Goal()
        goal_msg.plan = plan

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.success))
        rclpy.shutdown()

def main(args=None):
    try:
        with rclpy.init(args=args):

            action_client = MotorControlClient()

            action_client.send_goal([1.0, 10.0])

            rclpy.spin(action_client)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()