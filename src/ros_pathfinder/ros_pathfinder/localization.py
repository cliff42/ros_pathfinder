import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

import numpy as np

import random
import math

class LandmarkPoint:
    def __init__(self, x, y, count):
        self.x = x
        self.y = y
        self.count = count


class LandmarkIdentification(Node):
    def __init__(self):
        super().__init__('localization')
        self.line_publisher = self.create_publisher(MarkerArray, 'ransac', 10) #change
        self.landmark_publisher = self.create_publisher(MarkerArray,'landmarks',10)
        self.scan_subscriber = self.create_subscription(LaserScan,'scan', self.scan_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.resolution = 0.05 # m per cell
        self.width = 400 # num cells
        self.height = 400 # num cells

        self.origin_x = -(self.width * self.resolution) / 2.0
        self.origin_y = -(self.height * self.resolution) / 2.0

        # 0 for unoccupied, 1 for occupied, -1 for unknown
        self.grid = [-1] * (self.width * self.height) # init grid to be all unknown

        self.landmark_db = []
        self.good_landmark_min_count = 5
        self.min_distance_same_landmark = 0.01 # TODO: validate

        self.ekf_initialized = False
        self.prev_odom_x = None
        self.prev_odom_y = None
        self.prev_odom_theta = None
        # TODO: these are just the robot parts for now, need to add the landmarks later
        # EFK system state (called X in SLAM doc)
        self.system_state_X = np.zeros((3, 1), dtype=float)
        # EFK covariance matrix (called P in SLAM doc)
        self.covariance_matrix_P = np.diag([
            0.05 ** 2,
            0.05 ** 2,
            (5.0 * math.pi / 180.0) ** 2
        ])

    def scan_callback(self, msg: LaserScan):
        try:
            odom_to_laser_tf = self.tf_buffer.lookup_transform(
                'odom',                  
                msg.header.frame_id, # laser
                rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn('could not look up odom->laser transform')
            return
        
        self.grid = [-1] * (self.width * self.height) # reset map
        laser_x = odom_to_laser_tf.transform.translation.x
        laser_y = odom_to_laser_tf.transform.translation.y
        qx = odom_to_laser_tf.transform.rotation.x
        qy = odom_to_laser_tf.transform.rotation.y
        qz = odom_to_laser_tf.transform.rotation.z
        qw = odom_to_laser_tf.transform.rotation.w
        laser_yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz * qz))

        # TODO: update ekf to be based on odom pose not laser
        if not self.ekf_initialized:
            self.system_state_X[0, 0] = laser_x
            self.system_state_X[1, 0] = laser_y
            self.system_state_X[2, 0] = laser_yaw

            self.prev_odom_x = laser_x
            self.prev_odom_y = laser_y
            self.prev_odom_theta = laser_yaw
            self.ekf_initialized = True
        else:
            delta_x = laser_x - self.prev_odom_x
            delta_y = laser_y - self.prev_odom_y
            delta_theta = self.wrap_angle(laser_yaw - self.prev_odom_theta)

            # TODO: only run EKF update when the robot is moving (odom frame updates) not just every time we get a new scan
            self.ekf_predict(delta_x, delta_y, delta_theta)

            self.prev_odom_x = laser_x
            self.prev_odom_y = laser_y
            self.prev_odom_theta = laser_yaw
        self.get_logger().info(
            f"RAW pose: x={laser_x:.3f}, y={laser_y:.3f}, theta={laser_yaw:.3f}"
        )
        self.get_logger().info(
            f"EKF pose (from X): x={self.system_state_X[0,0]:.3f}, y={self.system_state_X[1,0]:.3f}, theta={self.system_state_X[2,0]:.3f}"
        )
        self.get_logger().info(
            f"P diag: {self.covariance_matrix_P[0,0]:.6f}, {self.covariance_matrix_P[1,1]:.6f}, {self.covariance_matrix_P[2,2]:.6f}"
        )

#         [INFO] [1775089536.980792567] [localization]: P diag: 0.006177, 0.006114, 0.022561
# [INFO] [1775089537.102365885] [localization]: Number of lines: 45
# [INFO] [1775089537.103711895] [localization]: Number of landmarks: 45
# [INFO] [1775089537.172076329] [localization]: RAW pose: x=0.289, y=0.031, theta=0.068
# [INFO] [1775089537.173215258] [localization]: EKF pose (from X): x=0.289, y=0.031, theta=0.068
# [INFO] [1775089537.174681661] [localization]: P diag: 0.006202, 0.006139, 0.022661
# [INFO] [1775089537.278797460] [localization]: Number of lines: 45
# [INFO] [1775089537.279850571] [localization]: Number of landmarks: 45
# [INFO] [1775089537.349521106] [localization]: RAW pose: x=0.289, y=0.031, theta=0.068
# [INFO] [1775089537.350456398] [localization]: EKF pose (from X): x=0.289, y=0.031, theta=0.068
# [INFO] [1775089537.351998360] [localization]: P diag: 0.006227, 0.006164, 0.022761
# [INFO] [1775089537.485179122] [localization]: Number of lines: 43
# [INFO] [1775089537.486525872] [localization]: Number of landmarks: 43
# [INFO] [1775089537.548172460] [localization]: RAW pose: x=0.289, y=0.031, theta=0.068
# [INFO] [1775089537.549121826] [localization]: EKF pose (from X): x=0.289, y=0.031, theta=0.068
# [INFO] [1775089537.550278774] [localization]: P diag: 0.006252, 0.006189, 0.022861
# [INFO] [1775089537.683170748] [localization]: Number of lines: 44
# [INFO] [1775089537.684312659] [localization]: Number of landmarks: 44
# [INFO] [1775089537.751120057] [localization]: RAW pose: x=0.289, y=0.031, theta=0.068
# [INFO] [1775089537.752527754] [localization]: EKF pose (from X): x=0.289, y=0.031, theta=0.068
# [INFO] [1775089537.754503805] [localization]: P diag: 0.006277, 0.006214, 0.022961
# [INFO] [1775089537.891220117] [localization]: Number of lines: 43
# [INFO] [1775089537.892672631] [localization]: Number of landmarks: 43
# [INFO] [1775089537.978848319] [localization]: RAW pose: x=0.289, y=0.031, theta=0.068
# [INFO] [1775089537.979955469] [localization]: EKF pose (from X): x=0.289, y=0.031, theta=0.068
# [INFO] [1775089537.981424502] [localization]: P diag: 0.006302, 0.006239, 0.023061
# [INFO] [1775089538.082222278] [localization]: Number of lines: 44
# [INFO] [1775089538.083486915] [localization]: Number of landmarks: 44
# [INFO] [1775089538.154471405] [localization]: RAW pose: x=0.289, y=0.031, theta=0.068
# [INFO] [1775089538.155669910] [localization]: EKF pose (from X): x=0.289, y=0.031, theta=0.068
# [INFO] [1775089538.157262373] [localization]: P diag: 0.006327, 0.006264, 0.023161
# [INFO] [1775089538.259037684] [localization]: Number of lines: 35
# [INFO] [1775089538.260234374] [localization]: Number of landmarks: 35
# [INFO] [1775089538.327276633] [localization]: RAW pose: x=0.289, y=0.031, theta=0.068
# [INFO] [1775089538.329130883] [localization]: EKF pose (from X): x=0.289, y=0.031, theta=0.068
# [INFO] [1775089538.330876333] [localization]: P diag: 0.006352, 0.006289, 0.023261
# [INFO] [1775089538.473371869] [localization]: Number of lines: 43
# [INFO] [1775089538.474826827] [localization]: Number of landmarks: 43
# [INFO] [1775089538.541536297] [localization]: RAW pose: x=0.289, y=0.031, theta=0.068
# [INFO] [1775089538.542439921] [localization]: EKF pose (from X): x=0.289, y=0.031, theta=0.068
# [INFO] [1775089538.543497125] [localization]: P diag: 0.006377, 0.006314, 0.023361
# [INFO] [1775089538.665166115] [localization]: Number of lines: 39
# [INFO] [1775089538.666543700] [localization]: Number of landmarks: 39
# [INFO] [1775089538.734862115] [localization]: RAW pose: x=0.289, y=0.031, theta=0.068
# [INFO] [1775089538.736041897] [localization]: EKF pose (from X): x=0.289, y=0.031, theta=0.068
# [INFO] [1775089538.737771088] [localization]: P diag: 0.006402, 0.006339, 0.023461
# [INFO] [1775089538.854214783] [localization]: Number of lines: 47
# [INFO] [1775089538.856051459] [localization]: Number of landmarks: 47
# [INFO] [1775089538.941325413] [localization]: RAW pose: x=0.289, y=0.031, theta=0.068
# [INFO] [1775089538.942315003] [localization]: EKF pose (from X): x=0.289, y=0.031, theta=0.068
# [INFO] [1775089538.944659863] [localization]: P diag: 0.006427, 0.006364, 0.023561
# [INFO] [1775089539.042715044] [localization]: Number of lines: 43
# [INFO] [1775089539.044177577] [localization]: Number of landmarks: 43
# [INFO] [1775089539.118441091] [localization]: RAW pose: x=0.289, y=0.031, theta=0.068
# [INFO] [1775089539.119544629] [localization]: EKF pose (from X): x=0.289, y=0.031, theta=0.068
# [INFO] [1775089539.120987402] [localization]: P diag: 0.006453, 0.006389, 0.023661
# [INFO] [1775089539.261942256] [localization]: Number of lines: 44
# [INFO] [1775089539.263387103] [localization]: Number of landmarks: 44
# [INFO] [1775089539.345342294] [localization]: RAW pose: x=0.289, y=0.031, theta=0.068
# [INFO] [1775089539.346583801] [localization]: EKF pose (from X): x=0.289, y=0.031, theta=0.068
# [INFO] [1775089539.347704377] [localization]: P diag: 0.006478, 0.006414, 0.023762
# [INFO] [1775089539.450078581] [localization]: Number of lines: 44
# [INFO] [1775089539.451301254] [localization]: Number of landmarks: 44
# [INFO] [1775089539.521589646] [localization]: RAW pose: x=0.289, y=0.031, theta=0.068
# [INFO] [1775089539.522535587] [localization]: EKF pose (from X): x=0.289, y=0.031, theta=0.068
# [INFO] [1775089539.523697887] [localization]: P diag: 0.006503, 0.006439, 0.023862
# [INFO] [1775089539.631481373] [localization]: Number of lines: 44
# [INFO] [1775089539.633326179] [localization]: Number of landmarks: 44
# [INFO] [1775089539.725045879] [localization]: RAW pose: x=0.289, y=0.031, theta=0.068
# [INFO] [1775089539.726275218] [localization]: EKF pose (from X): x=0.289, y=0.031, theta=0.068
# [INFO] [1775089539.727774456] [localization]: P diag: 0.006527, 0.006464, 0.023962
# [INFO] [1775089539.833105338] [localization]: Number of lines: 41
# [INFO] [1775089539.834522387] [localization]: Number of landmarks: 41
# [INFO] [1775089539.922181982] [localization]: RAW pose: x=0.289, y=0.031, theta=0.068
# [INFO] [1775089539.923410062] [localization]: EKF pose (from X): x=0.289, y=0.031, theta=0.068
# [INFO] [1775089539.925228978] [localization]: P diag: 0.006553, 0.006489, 0.024062
# [INFO] [1775089540.057424766] [localization]: Number of lines: 50
# [INFO] [1775089540.058445599] [localization]: Number of landmarks: 50
# [INFO] [1775089540.141326860] [localization]: RAW pose: x=0.289, y=0.031, theta=0.068
# [INFO] [1775089540.142178705] [localization]: EKF pose (from X): x=0.289, y=0.031, theta=0.068
# [INFO] [1775089540.143144239] [localization]: P diag: 0.006578, 0.006514, 0.024162
# [INFO] [1775089540.256346843] [localization]: Number of lines: 53
# [INFO] [1775089540.257858193] [localization]: Number of landmarks: 53
# [INFO] [1775089540.362726356] [localization]: RAW pose: x=0.289, y=0.031, theta=0.068
# [INFO] [1775089540.363816524] [localization]: EKF pose (from X): x=0.289, y=0.031, theta=0.068
# [INFO] [1775089540.365346356] [localization]: P diag: 0.006603, 0.006539, 0.024262
# [INFO] [1775089540.481771054] [localization]: Number of lines: 36
# [INFO] [1775089540.483549006] [localization]: Number of landmarks: 36
# [INFO] [1775089540.560107786] [localization]: RAW pose: x=0.289, y=0.031, theta=0.068
# [INFO] [1775089540.561152434] [localization]: EKF pose (from X): x=0.289, y=0.031, theta=0.068
# [INFO] [1775089540.562786325] [localization]: P diag: 0.006628, 0.006564, 0.024362
# [INFO] [1775089540.666636489] [localization]: Number of lines: 47
# [INFO] [1775089540.667672285] [localization]: Number of landmarks: 47
# [INFO] [1775089540.745727340] [localization]: RAW pose: x=0.289, y=0.031, theta=0.068
# [INFO] [1775089540.747119445] [localization]: EKF pose (from X): x=0.289, y=0.031, theta=0.068
# [INFO] [1775089540.748744761] [localization]: P diag: 0.006653, 0.006589, 0.024462
# [INFO] [1775089540.880337493] [localization]: Number of lines: 45
# [INFO] [1775089540.881407067] [localization]: Number of landmarks: 45
# [INFO] [1775089540.955524188] [localization]: RAW pose: x=0.289, y=0.031, theta=0.068
# [INFO] [1775089540.956483166] [localization]: EKF pose (from X): x=0.289, y=0.031, theta=0.068
# [INFO] [1775089540.957820398] [localization]: P diag: 0.006678, 0.006614, 0.024562
# [INFO] [1775089541.061996409] [localization]: Number of lines: 45
# [INFO] [1775089541.063257990] [localization]: Number of landmarks: 45
# [INFO] [1775089541.166004710] [localization]: RAW pose: x=0.289, y=0.031, theta=0.068
# [INFO] [1775089541.167718196] [localization]: EKF pose (from X): x=0.289, y=0.031, theta=0.068
# [INFO] [1775089541.170000166] [localization]: P diag: 0.006703, 0.006639, 0.024662
# [INFO] [1775089541.301249998] [localization]: Number of lines: 52
# [INFO] [1775089541.302514302] [localization]: Number of landmarks: 52
# [INFO] [1775089541.394964808] [localization]: RAW pose: x=0.289, y=0.031, theta=0.068
# [INFO] [1775089541.396398859] [localization]: EKF pose (from X): x=0.289, y=0.031, theta=0.068
# [INFO] [1775089541.398514711] [localization]: P diag: 0.006728, 0.006664, 0.024762
# [INFO] [1775089541.542791816] [localization]: Number of lines: 40
# [INFO] [1775089541.544497746] [localization]: Number of landmarks: 40
# [INFO] [1775089541.620507231] [localization]: RAW pose: x=0.289, y=0.031, theta=0.068
# [INFO] [1775089541.621353020] [localization]: EKF pose (from X): x=0.289, y=0.031, theta=0.068
# [INFO] [1775089541.622283830] [localization]: P diag: 0.006753, 0.006689, 0.024862
# [INFO] [1775089541.724150335] [localization]: Number of lines: 48
# [INFO] [1775089541.725381230] [localization]: Number of landmarks: 48
# [INFO] [1775089541.814284723] [localization]: RAW pose: x=0.289, y=0.031, theta=0.068
# [INFO] [1775089541.815697884] [localization]: EKF pose (from X): x=0.289, y=0.031, theta=0.068
# [INFO] [1775089541.818712323] [localization]: P diag: 0.006778, 0.006714, 0.024962
# [INFO] [1775089541.962484910] [localization]: Number of lines: 55
# [INFO] [1775089541.964078244] [localization]: Number of landmarks: 55
# [INFO] [1775089542.061598096] [localization]: RAW pose: x=0.290, y=0.032, theta=0.068
# [INFO] [1775089542.062514888] [localization]: EKF pose (from X): x=0.290, y=0.032, theta=0.068
# [INFO] [1775089542.063327565] [localization]: P diag: 0.006803, 0.006738, 0.025063
# [INFO] [1775089542.168725083] [localization]: Number of lines: 35
# [INFO] [1775089542.169803121] [localization]: Number of landmarks: 35
# [INFO] [1775089542.253764627] [localization]: RAW pose: x=0.292, y=0.033, theta=0.073
# [INFO] [1775089542.255123749] [localization]: EKF pose (from X): x=0.292, y=0.033, theta=0.073
# [INFO] [1775089542.258617724] [localization]: P diag: 0.006831, 0.006761, 0.025172
# [INFO] [1775089542.393737861] [localization]: Number of lines: 38
# [INFO] [1775089542.394739267] [localization]: Number of landmarks: 38
# [INFO] [1775089542.462403200] [localization]: RAW pose: x=0.311, y=0.030, theta=0.060
# [INFO] [1775089542.464608667] [localization]: EKF pose (from X): x=0.311, y=0.030, theta=0.060
# [INFO] [1775089542.467130126] [localization]: P diag: 0.006863, 0.006778, 0.025300
# [INFO] [1775089542.562921918] [localization]: Number of lines: 39
# [INFO] [1775089542.564005827] [localization]: Number of landmarks: 39
# [INFO] [1775089542.648512148] [localization]: RAW pose: x=0.321, y=0.026, theta=0.045
# [INFO] [1775089542.649722746] [localization]: EKF pose (from X): x=0.321, y=0.026, theta=0.045
# [INFO] [1775089542.651690186] [localization]: P diag: 0.006889, 0.006807, 0.025430
# [INFO] [1775089542.763439835] [localization]: Number of lines: 47
# [INFO] [1775089542.764928128] [localization]: Number of landmarks: 47
# [INFO] [1775089542.866115518] [localization]: RAW pose: x=0.323, y=0.027, theta=0.048
# [INFO] [1775089542.867596403] [localization]: EKF pose (from X): x=0.323, y=0.027, theta=0.048
# [INFO] [1775089542.869677328] [localization]: P diag: 0.006916, 0.006833, 0.025535
# [INFO] [1775089542.975997564] [localization]: Number of lines: 34
# [INFO] [1775089542.977046675] [localization]: Number of landmarks: 34
# [INFO] [1775089543.053921989] [localization]: RAW pose: x=0.329, y=0.036, theta=0.074
# [INFO] [1775089543.055220349] [localization]: EKF pose (from X): x=0.329, y=0.036, theta=0.074
# [INFO] [1775089543.057166084] [localization]: P diag: 0.006954, 0.006868, 0.025694
# [INFO] [1775089543.173693329] [localization]: Number of lines: 35
# [INFO] [1775089543.174896575] [localization]: Number of landmarks: 35
# [INFO] [1775089543.242893059] [localization]: RAW pose: x=0.337, y=0.041, theta=0.088
# [INFO] [1775089543.243805535] [localization]: EKF pose (from X): x=0.337, y=0.041, theta=0.088
# [INFO] [1775089543.244710012] [localization]: P diag: 0.006991, 0.006903, 0.025823
# [INFO] [1775089543.342521191] [localization]: Number of lines: 43
# [INFO] [1775089543.343657101] [localization]: Number of landmarks: 43
# [INFO] [1775089543.414024316] [localization]: RAW pose: x=0.337, y=0.041, theta=0.088
# [INFO] [1775089543.415243248] [localization]: EKF pose (from X): x=0.337, y=0.041, theta=0.088
# [INFO] [1775089543.417721168] [localization]: P diag: 0.007016, 0.006928, 0.025924
# [INFO] [1775089543.545852303] [localization]: Number of lines: 36
# [INFO] [1775089543.547300206] [localization]: Number of landmarks: 36
# [INFO] [1775089543.612068483] [localization]: RAW pose: x=0.335, y=0.040, theta=0.085
# [INFO] [1775089543.613153725] [localization]: EKF pose (from X): x=0.335, y=0.040, theta=0.085
# [INFO] [1775089543.614286283] [localization]: P diag: 0.007040, 0.006952, 0.026030
# [INFO] [1775089543.716083881] [localization]: Number of lines: 27
# [INFO] [1775089543.717146437] [localization]: Number of landmarks: 27
# [INFO] [1775089543.781847452] [localization]: RAW pose: x=0.334, y=0.039, theta=0.083
# [INFO] [1775089543.783495677] [localization]: EKF pose (from X): x=0.334, y=0.039, theta=0.083
# [INFO] [1775089543.785480302] [localization]: P diag: 0.007064, 0.006976, 0.026134
# [INFO] [1775089543.893018287] [localization]: Number of lines: 26
# [INFO] [1775089543.893942097] [localization]: Number of landmarks: 26
# [INFO] [1775089543.942687126] [localization]: RAW pose: x=0.333, y=0.042, theta=0.093
# [INFO] [1775089543.943535396] [localization]: EKF pose (from X): x=0.333, y=0.042, theta=0.093
# [INFO] [1775089543.944721438] [localization]: P diag: 0.007095, 0.007001, 0.026255
# [INFO] [1775089544.046971997] [localization]: Number of lines: 26
# [INFO] [1775089544.048718910] [localization]: Number of landmarks: 26
# [INFO] [1775089544.114652581] [localization]: RAW pose: x=0.332, y=0.043, theta=0.095
# [INFO] [1775089544.115787917] [localization]: EKF pose (from X): x=0.332, y=0.043, theta=0.095
# [INFO] [1775089544.117376566] [localization]: P diag: 0.007121, 0.007025, 0.026359
# [INFO] [1775089544.246670891] [localization]: Number of lines: 31
# [INFO] [1775089544.247996586] [localization]: Number of landmarks: 31
# [INFO] [1775089544.306503163] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089544.307537403] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089544.310057547] [localization]: P diag: 0.007146, 0.007050, 0.026460
# [INFO] [1775089544.408702868] [localization]: Number of lines: 23
# [INFO] [1775089544.409752183] [localization]: Number of landmarks: 23
# [INFO] [1775089544.455712985] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089544.456736521] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089544.458305835] [localization]: P diag: 0.007171, 0.007075, 0.026560
# [INFO] [1775089544.563477072] [localization]: Number of lines: 29
# [INFO] [1775089544.565417233] [localization]: Number of landmarks: 29
# [INFO] [1775089544.638668402] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089544.639795830] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089544.641267382] [localization]: P diag: 0.007196, 0.007100, 0.026660
# [INFO] [1775089544.764159427] [localization]: Number of lines: 25
# [INFO] [1775089544.765364970] [localization]: Number of landmarks: 25
# [INFO] [1775089544.815198945] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089544.816289113] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089544.817783258] [localization]: P diag: 0.007221, 0.007125, 0.026760
# [INFO] [1775089544.917375742] [localization]: Number of lines: 26
# [INFO] [1775089544.918716456] [localization]: Number of landmarks: 26
# [INFO] [1775089544.980828900] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089544.982139743] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089544.984275262] [localization]: P diag: 0.007246, 0.007150, 0.026860
# [INFO] [1775089545.116223720] [localization]: Number of lines: 22
# [INFO] [1775089545.117706087] [localization]: Number of landmarks: 22
# [INFO] [1775089545.174773521] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089545.175875060] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089545.177388484] [localization]: P diag: 0.007271, 0.007175, 0.026960
# [INFO] [1775089545.296352292] [localization]: Number of lines: 22
# [INFO] [1775089545.297530321] [localization]: Number of landmarks: 22
# [INFO] [1775089545.346223567] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089545.346972845] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089545.347703121] [localization]: P diag: 0.007296, 0.007200, 0.027060
# [INFO] [1775089545.471283801] [localization]: Number of lines: 18
# [INFO] [1775089545.472513960] [localization]: Number of landmarks: 18
# [INFO] [1775089545.514020182] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089545.514883684] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089545.515815781] [localization]: P diag: 0.007321, 0.007225, 0.027160
# [INFO] [1775089545.624279430] [localization]: Number of lines: 20
# [INFO] [1775089545.626109918] [localization]: Number of landmarks: 20
# [INFO] [1775089545.677885054] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089545.679210993] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089545.681178300] [localization]: P diag: 0.007346, 0.007250, 0.027260
# [INFO] [1775089545.780065292] [localization]: Number of lines: 19
# [INFO] [1775089545.781457011] [localization]: Number of landmarks: 19
# [INFO] [1775089545.832217567] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089545.834214133] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089545.837120592] [localization]: P diag: 0.007371, 0.007275, 0.027360
# [INFO] [1775089545.969321757] [localization]: Number of lines: 19
# [INFO] [1775089545.971088448] [localization]: Number of landmarks: 19
# [INFO] [1775089546.027885203] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089546.028940600] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089546.029986664] [localization]: P diag: 0.007396, 0.007300, 0.027460
# [INFO] [1775089546.137433453] [localization]: Number of lines: 20
# [INFO] [1775089546.138717650] [localization]: Number of landmarks: 20
# [INFO] [1775089546.180450863] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089546.181783353] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089546.182681759] [localization]: P diag: 0.007421, 0.007325, 0.027560
# [INFO] [1775089546.283091787] [localization]: Number of lines: 23
# [INFO] [1775089546.284147739] [localization]: Number of landmarks: 23
# [INFO] [1775089546.329611403] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089546.330658652] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089546.332343620] [localization]: P diag: 0.007446, 0.007350, 0.027660
# [INFO] [1775089546.456725772] [localization]: Number of lines: 17
# [INFO] [1775089546.458766199] [localization]: Number of landmarks: 17
# [INFO] [1775089546.504769106] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089546.506001307] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089546.507663128] [localization]: P diag: 0.007471, 0.007375, 0.027760
# [INFO] [1775089546.623816778] [localization]: Number of lines: 22
# [INFO] [1775089546.625037535] [localization]: Number of landmarks: 22
# [INFO] [1775089546.669379972] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089546.670370114] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089546.671949996] [localization]: P diag: 0.007496, 0.007400, 0.027860
# [INFO] [1775089546.803511364] [localization]: Number of lines: 19
# [INFO] [1775089546.806180826] [localization]: Number of landmarks: 19
# [INFO] [1775089546.856808980] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089546.858108028] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089546.859529938] [localization]: P diag: 0.007521, 0.007425, 0.027960
# [INFO] [1775089546.960813929] [localization]: Number of lines: 26
# [INFO] [1775089546.962379368] [localization]: Number of landmarks: 26
# [INFO] [1775089547.023860793] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089547.025483117] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089547.027519472] [localization]: P diag: 0.007546, 0.007450, 0.028060
# [INFO] [1775089547.144631695] [localization]: Number of lines: 26
# [INFO] [1775089547.145923336] [localization]: Number of landmarks: 26
# [INFO] [1775089547.196858729] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089547.198606361] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089547.200889922] [localization]: P diag: 0.007571, 0.007475, 0.028160
# [INFO] [1775089547.299306016] [localization]: Number of lines: 19
# [INFO] [1775089547.300486739] [localization]: Number of landmarks: 19
# [INFO] [1775089547.349661612] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089547.351511015] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089547.353990990] [localization]: P diag: 0.007596, 0.007500, 0.028260
# [INFO] [1775089547.488294695] [localization]: Number of lines: 27
# [INFO] [1775089547.490173374] [localization]: Number of landmarks: 27
# [INFO] [1775089547.606267753] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089547.607631057] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089547.609314655] [localization]: P diag: 0.007621, 0.007525, 0.028360
# [INFO] [1775089547.707022016] [localization]: Number of lines: 22
# [INFO] [1775089547.708965135] [localization]: Number of landmarks: 22
# [INFO] [1775089547.758780376] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089547.759708300] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089547.760739365] [localization]: P diag: 0.007646, 0.007550, 0.028460
# [INFO] [1775089547.857118290] [localization]: Number of lines: 20
# [INFO] [1775089547.858329678] [localization]: Number of landmarks: 20
# [INFO] [1775089547.901624622] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089547.902428479] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089547.903144101] [localization]: P diag: 0.007671, 0.007575, 0.028560
# [INFO] [1775089548.028978696] [localization]: Number of lines: 15
# [INFO] [1775089548.030475788] [localization]: Number of landmarks: 15
# [INFO] [1775089548.066103428] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089548.067103532] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089548.068050325] [localization]: P diag: 0.007696, 0.007600, 0.028660
# [INFO] [1775089548.167568922] [localization]: Number of lines: 29
# [INFO] [1775089548.169056311] [localization]: Number of landmarks: 29
# [INFO] [1775089548.225881708] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089548.227127019] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089548.229071862] [localization]: P diag: 0.007721, 0.007625, 0.028760
# [INFO] [1775089548.351534422] [localization]: Number of lines: 26
# [INFO] [1775089548.352955501] [localization]: Number of landmarks: 26
# [INFO] [1775089548.408910396] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089548.410247665] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089548.413090266] [localization]: P diag: 0.007746, 0.007650, 0.028860
# [INFO] [1775089548.539041203] [localization]: Number of lines: 15
# [INFO] [1775089548.540682971] [localization]: Number of landmarks: 15
# [INFO] [1775089548.576045071] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089548.576851762] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089548.577644361] [localization]: P diag: 0.007771, 0.007675, 0.028960
# [INFO] [1775089548.688743424] [localization]: Number of lines: 26
# [INFO] [1775089548.690373693] [localization]: Number of landmarks: 26
# [INFO] [1775089548.757515060] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089548.758718226] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089548.760304794] [localization]: P diag: 0.007796, 0.007700, 0.029060
# [INFO] [1775089548.877167092] [localization]: Number of lines: 32
# [INFO] [1775089548.878608132] [localization]: Number of landmarks: 32
# [INFO] [1775089548.941840519] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089548.943348907] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089548.945401243] [localization]: P diag: 0.007821, 0.007725, 0.029160
# [INFO] [1775089549.071022168] [localization]: Number of lines: 22
# [INFO] [1775089549.072579961] [localization]: Number of landmarks: 22
# [INFO] [1775089549.119987515] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089549.121240622] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089549.122402865] [localization]: P diag: 0.007846, 0.007750, 0.029260
# [INFO] [1775089549.244272857] [localization]: Number of lines: 19
# [INFO] [1775089549.246349525] [localization]: Number of landmarks: 19
# [INFO] [1775089549.289437438] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089549.290330660] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089549.291199051] [localization]: P diag: 0.007871, 0.007775, 0.029360
# [INFO] [1775089549.401649007] [localization]: Number of lines: 28
# [INFO] [1775089549.403313497] [localization]: Number of landmarks: 28
# [INFO] [1775089549.473145129] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089549.474259078] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089549.475363417] [localization]: P diag: 0.007896, 0.007800, 0.029460
# [INFO] [1775089549.582600535] [localization]: Number of lines: 30
# [INFO] [1775089549.584156587] [localization]: Number of landmarks: 30
# [INFO] [1775089549.659986181] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089549.661525494] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089549.664895304] [localization]: P diag: 0.007921, 0.007825, 0.029560
# [INFO] [1775089549.761095825] [localization]: Number of lines: 26
# [INFO] [1775089549.763164568] [localization]: Number of landmarks: 26
# [INFO] [1775089549.831908581] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089549.833733321] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089549.836816297] [localization]: P diag: 0.007946, 0.007850, 0.029660
# [INFO] [1775089549.941853661] [localization]: Number of lines: 21
# [INFO] [1775089549.943096936] [localization]: Number of landmarks: 21
# [INFO] [1775089549.989259196] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089549.990451030] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089549.992393392] [localization]: P diag: 0.007971, 0.007875, 0.029760
# [INFO] [1775089550.103903706] [localization]: Number of lines: 27
# [INFO] [1775089550.105771906] [localization]: Number of landmarks: 27
# [INFO] [1775089550.175557454] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089550.176610222] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089550.178505050] [localization]: P diag: 0.007996, 0.007900, 0.029860
# [INFO] [1775089550.278297384] [localization]: Number of lines: 21
# [INFO] [1775089550.279775516] [localization]: Number of landmarks: 21
# [INFO] [1775089550.337469842] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089550.339091446] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089550.341310884] [localization]: P diag: 0.008021, 0.007925, 0.029960
# [INFO] [1775089550.451172155] [localization]: Number of lines: 23
# [INFO] [1775089550.452346287] [localization]: Number of landmarks: 23
# [INFO] [1775089550.507695443] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089550.509043415] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089550.510828121] [localization]: P diag: 0.008046, 0.007950, 0.030060
# [INFO] [1775089550.645852457] [localization]: Number of lines: 28
# [INFO] [1775089550.647330052] [localization]: Number of landmarks: 28
# [INFO] [1775089550.708774148] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089550.709869414] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089550.712139219] [localization]: P diag: 0.008071, 0.007975, 0.030160
# [INFO] [1775089550.825414095] [localization]: Number of lines: 21
# [INFO] [1775089550.827359846] [localization]: Number of landmarks: 21
# [INFO] [1775089550.886437292] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089550.887473506] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089550.889310745] [localization]: P diag: 0.008096, 0.008000, 0.030260
# [INFO] [1775089550.986496127] [localization]: Number of lines: 25
# [INFO] [1775089550.988674123] [localization]: Number of landmarks: 25
# [INFO] [1775089551.043544342] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089551.045363897] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089551.047161565] [localization]: P diag: 0.008121, 0.008025, 0.030360
# [INFO] [1775089551.152102267] [localization]: Number of lines: 37
# [INFO] [1775089551.153127852] [localization]: Number of landmarks: 37
# [INFO] [1775089551.228206045] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089551.229603941] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089551.230856809] [localization]: P diag: 0.008146, 0.008050, 0.030460
# [INFO] [1775089551.366986402] [localization]: Number of lines: 40
# [INFO] [1775089551.368789180] [localization]: Number of landmarks: 40
# [INFO] [1775089551.452413147] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089551.453617962] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089551.454583162] [localization]: P diag: 0.008171, 0.008075, 0.030560
# [INFO] [1775089551.555178500] [localization]: Number of lines: 31
# [INFO] [1775089551.556423553] [localization]: Number of landmarks: 31
# [INFO] [1775089551.622036620] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089551.623161070] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089551.624615592] [localization]: P diag: 0.008196, 0.008100, 0.030660
# [INFO] [1775089551.751697589] [localization]: Number of lines: 26
# [INFO] [1775089551.755741898] [localization]: Number of landmarks: 26
# [INFO] [1775089551.839954366] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089551.843256164] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089551.846224723] [localization]: P diag: 0.008221, 0.008125, 0.030760
# [INFO] [1775089552.008613122] [localization]: Number of lines: 36
# [INFO] [1775089552.010542098] [localization]: Number of landmarks: 36
# [INFO] [1775089552.113834676] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089552.115193019] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089552.118388585] [localization]: P diag: 0.008246, 0.008150, 0.030860
# [INFO] [1775089552.219665248] [localization]: Number of lines: 25
# [INFO] [1775089552.221214784] [localization]: Number of landmarks: 25
# [INFO] [1775089552.291559048] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089552.292868154] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089552.294310492] [localization]: P diag: 0.008271, 0.008175, 0.030960
# [INFO] [1775089552.399241656] [localization]: Number of lines: 20
# [INFO] [1775089552.400414233] [localization]: Number of landmarks: 20
# [INFO] [1775089552.451315586] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089552.452490181] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089552.453888133] [localization]: P diag: 0.008296, 0.008200, 0.031060
# [INFO] [1775089552.571232876] [localization]: Number of lines: 33
# [INFO] [1775089552.572455246] [localization]: Number of landmarks: 33
# [INFO] [1775089552.645571528] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089552.646631667] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089552.648355674] [localization]: P diag: 0.008321, 0.008225, 0.031160
# [INFO] [1775089552.786118751] [localization]: Number of lines: 32
# [INFO] [1775089552.787852349] [localization]: Number of landmarks: 32
# [INFO] [1775089552.860750477] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089552.861871057] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089552.863039486] [localization]: P diag: 0.008346, 0.008250, 0.031260
# [INFO] [1775089552.976647549] [localization]: Number of lines: 25
# [INFO] [1775089552.979075957] [localization]: Number of landmarks: 25
# [INFO] [1775089553.043624792] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089553.045445552] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089553.048242161] [localization]: P diag: 0.008371, 0.008275, 0.031360
# [INFO] [1775089553.172239079] [localization]: Number of lines: 23
# [INFO] [1775089553.173502762] [localization]: Number of landmarks: 23
# [INFO] [1775089553.229493010] [localization]: RAW pose: x=0.332, y=0.042, theta=0.094
# [INFO] [1775089553.230512911] [localization]: EKF pose (from X): x=0.332, y=0.042, theta=0.094
# [INFO] [1775089553.232309506] [localization]: P diag: 0.008396, 0.008300, 0.031460


        angle = msg.angle_min

        angle_list = []
        range_dict = {}
        if len(msg.ranges) == 0:
            return
        for point in msg.ranges: 
            if math.isinf(point) or math.isnan(point):
                angle += msg.angle_increment
                continue

            if point > msg.range_max or point < msg.range_min:
                angle += msg.angle_increment
                continue

            # scan_x = math.cos(angle + laser_yaw) * point + laser_x
            # scan_y = math.sin(angle + laser_yaw) * point + laser_y

            # TODO: confirm this works
            # TODO: after switching to odom pose for ekf we need to transform for laser here
            ekf_x = self.system_state_X[0, 0]
            ekf_y = self.system_state_X[1, 0]
            ekf_theta = self.system_state_X[2, 0]

            scan_x = math.cos(angle + ekf_theta) * point + ekf_x
            scan_y = math.sin(angle + ekf_theta) * point + ekf_y

            range_dict[angle] = (scan_x,scan_y)
            angle_list.append(angle)
            angle += msg.angle_increment

        N = 100 #number of iterations for ransac (lines to check)
        S = 5 #number of samples to fit each line to
        D = 5 #degrees from initial sample point
        # X = 0.005 #max distance (in meters) that points must be from line
        X = 0.05
        CONSENSUS = 50 #minimum number of points within X meters for a line to be considered adequate
        lines, landmarks = self.ransac(angle_list,msg.angle_increment,range_dict,N,S,D,X,CONSENSUS)
        self.get_logger().info('Number of lines: %s' % len(lines))
        self.get_logger().info('Number of landmarks: %s' % len(landmarks))
        

        marker_array = MarkerArray()
        id = 0
        for line in lines:
            points_in_line = self.points_from_line(line[0], line[1], line[2], line[3], line[4], line[5])
            line_marker = Marker()
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.header.frame_id = 'odom'
            line_marker.type = Marker.LINE_STRIP
            line_marker.scale.x = 0.005
            line_marker.color.r = 0.0
            line_marker.color.g = 1.0
            line_marker.color.b = 0.0
            line_marker.color.a = 1.0
            line_marker.id = id
            id += 1
            for point in points_in_line:
                p = Point()
                p.x = point[0]
                p.y = point[1]
                p.z = 0.0
                line_marker.points.append(p)
            marker_array.markers.append(line_marker)
        self.line_publisher.publish(marker_array)

        landmark_array = MarkerArray()
        id = 0
        for landmark in landmarks:
            landmark_marker = Marker()
            landmark_marker.header.stamp = self.get_clock().now().to_msg()
            landmark_marker.header.frame_id = 'odom'
            landmark_marker.type = Marker.SPHERE
            landmark_marker.scale.x = 0.1
            landmark_marker.scale.y = 0.1
            landmark_marker.scale.z = 0.1
            landmark_marker.color.r = 1.0
            landmark_marker.color.g = 0.0
            landmark_marker.color.b = 0.0
            landmark_marker.color.a = 1.0
            landmark_marker.id = id
            id += 1

            landmark_marker.pose.position.x = float(landmark[0])
            landmark_marker.pose.position.y = float(landmark[1])
            landmark_marker.pose.position.z = 0.0
            landmark_marker.pose.orientation.w = 1.0
            landmark_array.markers.append(landmark_marker)
        
        self.landmark_publisher.publish(landmark_array)

        # check landmarks against global db
        for landmark in landmarks:
            closest_landmark_idx, dist = self.get_closest_landmark_from_db(landmark)

            if closest_landmark_idx == -1:
                lp = LandmarkPoint(landmark[0], landmark[1], 1)
                self.landmark_db.append(lp)
                continue
            
            if dist <= self.min_distance_same_landmark:
               self.landmark_db[closest_landmark_idx].count += 1
            else:
                lp = LandmarkPoint(landmark[0], landmark[1], 1)
                self.landmark_db.append(lp)

    def get_closest_landmark_from_db(self, landmark):
        min_dist = float("inf")
        idx = -1
        for i, l in enumerate(self.landmark_db):
            dist = math.sqrt((landmark[0] - l.x)**2 + (landmark[1] - l.y)**2)
            if dist < min_dist:
                min_dist = dist
                idx = i
        return idx, min_dist
    

    # TODO: use this with EKF
    def get_closest_good_landmark(self, landmark):
        min_dist = float("inf")
        idx = -1
        for i, l in enumerate(self.landmark_db):
            if l.count < self.good_landmark_min_count:
                continue

            dist = math.sqrt((landmark[0] - l.x)**2 + (landmark[1] - l.y)**2)
            if dist < min_dist:
                min_dist = dist
                idx = i
        return idx, min_dist
    

    def wrap_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def ekf_predict(self, delta_x, delta_y, delta_theta):
        self.system_state_X[0, 0] += delta_x
        self.system_state_X[1, 0] += delta_y
        self.system_state_X[2, 0] = self.wrap_angle(self.system_state_X[2, 0] + delta_theta)

        jacobian_pred_A = np.array([
            [1.0, 0.0, -delta_y],
            [0.0, 1.0,  delta_x],
            [0.0, 0.0,  1.0],
        ], dtype=float)

        #TODO: DETERMINE ERROR
        sx = 0.05 * abs(delta_x) + 0.005
        sy = 0.05 * abs(delta_y) + 0.005
        st = 0.10 * abs(delta_theta) + 0.01

        Q = np.diag([sx * sx, sy * sy, st * st])

        self.covariance_matrix_P = jacobian_pred_A @ self.covariance_matrix_P @ jacobian_pred_A.T + Q


    def points_from_line(self, m, c, min_x, max_x,min_y,max_y):
        #set bounds of line in y direction if line near vertical (|m|>1)
        #set bounds of line in x direction if line near horizontal (|m|<=1)
        if abs(m)>1:
            start_point = ((min_y-c)/m,min_y)
            end_point  = ((max_y-c)/m,max_y) 
        else:
            start_point = (min_x, m * min_x + c)
            end_point = (max_x, m * max_x + c)   
        return [start_point, end_point]
    
    def angle_diff(self, a, b):
        return min(abs(a - b), (2*math.pi - abs(a - b)))

    def ransac(self,angle_list,angle_increment,range_dict,N,S,D,X,CONSENSUS):
        landmark_lines = []
        landmarks = []
        for i in range(N):
            rand_angle = random.choice(angle_list)

            angle_subset = [a for a in angle_list if self.angle_diff(a, rand_angle) < math.radians(D)]

            if len(angle_subset) < S:
                continue

            line_point_list = random.sample(angle_subset, S)
            point_list = [(range_dict[a][0],range_dict[a][1]) for a in line_point_list]

            m, c = self.least_squares(point_list, S)
            A = m
            B = -1
            C = c

            denom = math.sqrt(A*A + B*B)
            
            readings_on_line = []
            for a in angle_list:
                point_x = range_dict[a][0]
                point_y = range_dict[a][1]

                distance = abs(A * point_x + B * point_y + C) / denom
                if distance < X:
                    readings_on_line.append([point_x, point_y])

            if len(readings_on_line) > CONSENSUS:
                new_m, new_c = self.least_squares(readings_on_line, len(readings_on_line))
                xs = [p[0] for p in readings_on_line]
                ys = [p[1] for p in readings_on_line]
                landmark_lines.append((new_m, new_c, min(xs), max(xs), min(ys), max(ys)))


        # TODO: to check (is this orthog to static origin or odom origin) - i think we want this to be orthog to true static origin
        #find point on landmark line that we can track (point orthog. to origin)
        for line in landmark_lines:
            m,c = line[0],line[1]
            b = -1/m #orthogonal line has slope -1/m with y intercept 0 (passes through origin)
            landmark_x = c/(b-m)
            landmark_y = b *landmark_x
            landmarks.append([landmark_x,landmark_y])
        


        return landmark_lines, landmarks

    def least_squares(self,list, S):
        sum_x = 0
        sum_x2 = 0
        sum_y = 0
        sum_xy = 0
        for a in list:
            scan_x = a[0]
            scan_y = a[1]
            sum_x += scan_x
            sum_x2 += scan_x * scan_x
            sum_y += scan_y
            sum_xy += scan_x * scan_y

        m = ((S * sum_xy) - (sum_x * sum_y))  / ((S * sum_x2) - (sum_x * sum_x))
        c = (sum_y - (m * sum_x)) / S

        return m, c










            


    def publish_map(self):
        msg = LandmarkIdentification()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'

        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height

        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = self.grid

        self.map_publisher.publish(msg)

def main(args=None):
    try:
        with rclpy.init(args=args):
            landmark_identification = LandmarkIdentification()
            rclpy.spin(landmark_identification)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()