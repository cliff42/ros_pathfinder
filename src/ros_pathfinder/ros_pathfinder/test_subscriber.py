from time import sleep
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node



import math

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import LaserScan

class MinimalSubscriber(Node):
    
    
    prev_angle_l = 0
    prev_angle_r = 0
    distance_l = 0
    distance_r = 0
    prev_distance_l = 0
    prev_distance_r = 0
    vel_l = 0
    vel_r = 0
    prev_vel_l = 0
    prev_vel_r = 0
    count = 0

    # vel_x, vel_y, vel_z = 0
    # dist_x, dist_y, dist_z = 0

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(Float64MultiArray,'test_topic',self.listener_callback,10)
        self.subscription2 = self.create_subscription(Float64MultiArray,'imu_topic',self.listener_callback2,10)
        self.subscription3 = self.create_subscription(LaserScan, 'scan', self.listener_callback3, 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        angle_l = float(msg.data[0])
        angle_r = float(msg.data[1])
        timer_period = float(msg.data[2])
        time_elapsed = timer_period * self.count
        # self.get_logger().info('angle (l): "%s"' % angle_l)
        # self.get_logger().info('angle (r): "%s"' % angle_r)
        # self.get_logger().info('timer period (s): "%s"' % timer_period)
        # self.get_logger().info('time elapsed (s): "%s"'% time_elapsed)
        if self.count == 0:
            self.prev_angle_l = angle_l
            self.prev_angle_r = angle_r
        
        self.distance_l,self.prev_angle_l = self.get_distance(angle_l,self.prev_angle_l,self.distance_l)
        self.distance_r,self.prev_angle_r = self.get_distance(angle_r,self.prev_angle_r,self.distance_r)
        
        self.vel_l,self.prev_distance_l = self.get_velocity(self.distance_l,self.prev_distance_l,timer_period)
        self.vel_r,self.prev_distance_r = self.get_velocity(self.distance_r,self.prev_distance_r,timer_period)

        # self.get_logger().info('distance (l) (m): "%s"' % self.distance_l)
        # self.get_logger().info('velocity (l) (m/s): "%s"' % self.vel_l)
        # self.get_logger().info('distance (r) (m): "%s"' % self.distance_r)
        # self.get_logger().info('velocity (r) (m/s): "%s"' % self.vel_r)
        

        self.count += 1
    def listener_callback2(self,msg):
        acc_x = float(msg.data[0])
        acc_y = float(msg.data[1])
        acc_z = float(msg.data[2])
        gyr_x = float(msg.data[3])
        gyr_y = float(msg.data[4])
        gyr_z = float(msg.data[5])
        timer_period = float(msg.data[6])
        # self.get_logger().info('acc_x : "%s" m/s^2 acc_y: "%s" m/s^2 acc_z: "%s" m/s^2' % (acc_x,acc_y,acc_z))
        # self.get_logger().info('gyr_x : "%s" rad/s gyr_y: "%s" rad/s gyr_z: "%s" rad/s' % (gyr_x,gyr_y,gyr_z))


        #find some way to get acc_x,acc_y,acc_z into actual acc_x,acc_y,acc_z
        # self.vel_x,self.dist_x = self.integrate(acc_x,self.vel_x,self.dist_x,timer_period)
        # self.vel_y,self.dist_y = self.integrate(acc_y,self.vel_y,self.dist_y,timer_period)
        # self.vel_z,self.dist_z = self.integrate(acc_z,self.vel_z,self.dist_z,timer_period)

    def listener_callback3(self,msg):
        sleep(5)
        self.get_logger().info('lidar msg: %s' % msg)


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
    def get_velocity(self,dist,prev_dist,timer_period):
        vel = (dist - prev_dist)/timer_period
        prev_dist = dist
        return vel, prev_dist
    def integrate(self,acc,vel,dist,timer_period):
        vel = vel + acc * timer_period
        dist = dist + vel * timer_period
        return vel, dist
        

        
            


def main(args=None):
    try:
        with rclpy.init(args=args):
            minimal_subscriber = MinimalSubscriber()

            rclpy.spin(minimal_subscriber)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()