from typing import Optional
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid

import numpy as np

from ros_pathfinder.planning.costmap import Costmap2d, CostmapConfig

class PathPlanner(Node):

    MAP_TOPIC = "map"
    COSTMAP_TOPIC = "global_costmap"

    def __init__(self):
        super().__init__("path_planner")

        # TODO: put these in config
        self.publish_rate_hz = 1.0 
        self._costmap_config = CostmapConfig(
            robot_radius_m=0.20,
            safety_margin_m=0.05,
            occupied_threshold=65,
            allow_unknown=True,
            unknown_cost_multiplier=3.0
        )
        self._costmap: Optional[Costmap2d] = None

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            self.MAP_TOPIC,
            self.map_callback,
            map_qos
        )

        self.costmap_publisher = self.create_publisher(
            OccupancyGrid,
            self.COSTMAP_TOPIC,
            map_qos
        )

    def map_callback(self, map_msg: OccupancyGrid) -> None:
        occupancy_values = np.asarray(map_msg.data, dtype=np.int16).reshape(map_msg.info.height, map_msg.info.width)

        self._costmap = Costmap2d.from_occupancy(
            occupancy_values=occupancy_values,
            resolution_m=map_msg.info.resolution,
            config=self._costmap_config,
        )

        costmap_msg = OccupancyGrid()
        costmap_msg.header.frame_id = map_msg.header.frame_id
        costmap_msg.header.stamp = self.get_clock().now().to_msg()
        costmap_msg.info = map_msg.info
        costmap_msg.data = (
            self._costmap.values.reshape(-1).tolist()
        )

        self.costmap_publisher.publish(costmap_msg)



def main(args=None) -> None:
    rclpy.init(args=args)
    node = None

    try:
        node = PathPlanner()
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