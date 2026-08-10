import math
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Time

from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry

from ros_pathfinder.localization.wheel_imu_odometry import WheelIMUOdometry

class LocalOdometry(Node):

    ODOMETRY_TOPIC = "odom"
    IMU_TOPIC = "imu/data_raw"
    JOINT_STATES_TOPIC = "joint_states"

    ODOM_FRAME = "odom"
    BASE_FRAME = "base_link" # child frame b/c base_link moves with the robot

    def __init__(self):
        super().__init__("local_odometry")

        self._declare_parameters()
        self._get_parameters()

        self._estimator = WheelIMUOdometry(wheel_radius_m=self.wheel_radius_m, encoder_distance_scale=self.encoder_distance_scale)

        self.imu_subscription = self.create_subscription(
            Imu,
            self.IMU_TOPIC,
            self.imu_callback,
            qos_profile_sensor_data
        )

        self.joint_state_subscription = self.create_subscription(
            JointState,
            self.JOINT_STATES_TOPIC,
            self.joint_state_callback,
            qos_profile_sensor_data
        )

        self.odometry_publisher = self.create_publisher(
            Odometry,
            self.ODOMETRY_TOPIC,
            qos_profile_sensor_data
        )

    def imu_callback(self, msg: Imu) -> None:
        self._estimator.add_imu_sample(
            yaw_rate_rad_s=msg.angular_velocity.z, 
            timestamp_ns=self._timestamp_to_ns(msg.header.stamp)
        )

    def joint_state_callback(self, msg: JointState) -> None:
        left_idx = msg.name.index("left_wheel_joint")
        right_idx = msg.name.index("right_wheel_joint")

        odometry_state = self._estimator.add_encoder_sample(
            left_position_rad=msg.position[left_idx],
            right_position_rad=msg.position[right_idx],
            timestamp_ns=self._timestamp_to_ns(msg.header.stamp)
        )

        if odometry_state is not None:
            odometry_msg = Odometry()
            odometry_msg.header.stamp = msg.header.stamp
            odometry_msg.header.frame_id = self.ODOM_FRAME
            odometry_msg.child_frame_id = self.B

            odometry_msg.pose.pose.position.x = odometry_state.x_m
            odometry_msg.pose.pose.position.y = odometry_state.y_m
            odometry_msg.pose.pose.position.z = 0.0 # our robot operates in 2d space (assumes static height)

            # rotation is only around the z axis
            half_yaw = odometry_state.yaw_rad / 2.0
            odometry_msg.pose.pose.orientation.x = 0.0
            odometry_msg.pose.pose.orientation.y = 0.0
            odometry_msg.pose.pose.orientation.z = math.sin(half_yaw)
            odometry_msg.pose.pose.orientation.w = math.cos(half_yaw)

            odometry_msg.twist.twist.linear.x = odometry_state.linear_vel_m_s
            odometry_msg.twist.twist.angular.z = odometry_state.angular_vel_rad_s
        
            self.odometry_publisher.publish(odometry_msg)
    

    def destroy_node(self):
        return super().destroy_node()

    def _timestamp_to_ns(self, ts: Time) -> int:
        return(int(ts.sec) * 1000000000 + int(ts.nanosec))

    def _declare_parameters(self) -> None:
        self.declare_parameter("wheel_radius_m", 0.1016)
        self.declare_parameter("encoder_distance_scale", 1.10497)

    def _get_parameters(self) -> None:
        self.wheel_radius_m = self.get_parameter("wheel_radius_m").value
        self.encoder_distance_scale = self.get_parameter("encoder_distance_scale").value

def main(args=None) -> None:
    rclpy.init(args=args)
    node = None

    try:
        node = LocalOdometry()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()