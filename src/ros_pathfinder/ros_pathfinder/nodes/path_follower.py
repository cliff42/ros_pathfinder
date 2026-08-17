#Two options: manual (select points for robot to travel to), automatic (robot explores by travelling to frontier regions)

#while stationary, run djikstra to find possible paths to every node
#while moving, run a* to find paths to node

#pure pursuit algorithm

#check periodically if any of the cells in current path is occupied
#if occupied replan
#if goal is occupied, stop the robot and wait a couple seconds before replanning again
#if replanning is unsuccessful, stop robot
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Pose

from ros_pathfinder.mapping.occupancy_grid import OccupancyGrid2d
from ros_pathfinder.planning.a_star import AStarPlanner
from ros_pathfinder.planning.djikstra import Djikstra
from ros_pathfinder.planning.exploration import FrontierCellFinder
from ros_pathfinder.planning.base_planner import GridCell

import numpy as np
import time
import math

class Planner(Node):
    ODOM = "odom"
    COSTMAP = "global_costmap"
    TWIST = "cmd_vel"
    MANUAL_GOAL = "/goal_pose"
    def __init__(self):
        self.start = None
        self.goal = None
        self.path = [None]
        self.stuck = False
        super().__init__("path_planner")
        self.odom_subscription = self.create_subscription(Odometry,self.ODOM,self.odom_callback)
        self.costmap_subscription = self.create_subscription(OccupancyGrid,self.COSTMAP,self.map_callback)
        if self.mode == "manual":
            self.goal_subscription = self.create_subscription(PoseStamped,self.MANUAL_GOAL,self.goal_callback)
        else:
            self.exploration_start_time = time.time()
        self.twist_publisher = self.create_publisher


    def odom_callback(self,current:Odometry)->GridCell:
        
        #Convert to grid cell
        pass
    def goal_callback(self,goal:PoseStamped)->GridCell:
        #convert to grid cell
        pass
    def plan_path(self, costmap: OccupancyGrid):
        if self.path is None:
            if self.mode == "manual":
                if self.goal is not None:
                    #initialize djikstra map and wait for user to select goal
                    djikstraMap = Djikstra(self.start,costmap)
                else:
                    self.path = djikstraMap.find_path(self.goal)
                    if self.path is None:
                        self.get_logger.warning('Path was not found. Pick another goal location.')
            elif self.mode == "exploration":
                if time.time() - self.exploration_start_time >= self.max_exploration_time:
                    self.get_logger.warning(f'{self.max_exploration_time} s have elapsed since start of exploration. Switching to manual mode')
                    self.mode = "manual"
                solver = FrontierCellFinder(self.start,costmap)
                self.path = solver.getPath() 
            else:
                self.get_logger.warning('Pick either "manual" or "exploration" for mode')
        else:
            #pure pursuit planning
            #check if any of self.path is occupied
            if any(costmap[idx] != 0 and costmap[idx] != -1 for idx in self.path):
                path = AStarPlanner(costmap,self.start,self.goal).plan().path
                if path is None:
                    if self.stuck == False:
                        self.get_logger.warning(f'No path to goal. Waiting {self.max_wait_time} s for obstruction to move before terminating. ')
                        startTime = time.time()
                        self.struck = True
                    elapsedTime = time.time() - startTime
                    if elapsedTime >= self.max_wait_time:
                        self.get_logger.warning('Goal location is not accessible. Pick a new location')
                else:
                    #pure pursuit planning
                    self.stuck = False
                    pass
            if self.dist_to_goal(self.start,self.goal) <= self.goal_dist:
                self.get_logger.warning('Goal reached')
                self.path = None

            

    def coord_to_grid(self):
        pass
    def dist_to_goal(self):
        pass
        

    def _declare_parameters(self) -> None:
        self.declare_parameter("mode","manual")
        self.declare_parameter("lookahead",0.15)
        self.declare_parameter("max_wait_time",5)
        self.declare_parameter("max_exploration_time",300)
        self.declare_parameter("goal_dist",0.1)
    def _get_parameters(self)-> None:
        self.mode = self.get_parameter("mode").value
        self.lookahead = self.get_parameter("lookahead").value
        self.max_wait_time = self.get_parameter("max_wait_time").value
        self.max_exploration_time = self.get_parameter("max_exploration_time").value
        self.goal_dist = self.get_parameter("goal_dist").value

    def _world_to_grid(self, point: [float,float]) -> tuple[int, int]:
        x = math.floor((point[0] - self._config.origin_x_m) / self._config.resolution_m)
        y = math.floor((point[1] - self._config.origin_y_m) / self._config.resolution_m)

        return x, y

def main(args=None) -> None:
    rclpy.init(args=args)
    node = None

    try:
        node = Planner()
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
    

