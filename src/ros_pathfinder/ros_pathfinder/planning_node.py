import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from action_interfaces.action import FollowPath

from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path, OccupancyGrid, Odometry

from tf2_ros import Buffer, TransformListener

import heapq
import numpy as np
from collections import deque
import math
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        self.goal = None
        self.start = None

        '''
        Mode select for planning
        1 - Manual Planning (User selects goal for robot and robot plans path to goal )
        2 - Exploration Planning (Explore region until no reachable areas can be explored)
        3 - Computer Vision (if we get a camera)
            - April Tags - Robot identifies april tag and plans path to the april tag
            - User Input - User inputs an object type or color. Camera tries to identify object/color and plan course to
        '''
        self.mode = 1
        '''
        Planners
        A*
        RRT
        RRT*
        '''
        self.planner = "ASTAR"

        '''
        Publisher and Subscriber for Manual Planning
        Subscriber - "/map" : OccupancyGrid
            -  Occupancy Grid
            -  0: Occupied by Obstacles
            -  1: Free
            - -1: Unknown (Out of Range or blocked by obstacles)
        Subscriber - "/goal_pose" : PoseStamped
            - This is "2D Goal Pose" in rviz2
            - PoseStamped
                - Header: Time stamp and reference frame
                - Pose: Point: x,y,z Quaternion: qx,qy,qz,qw
        Publisher - "/path" : Path
            - List of poses for robot to go to over time
            - Path
                - List of PoseStamped
        '''

        
        self.create_subscription(Odometry,"slam_odom",self.setStart,10)
        self.create_subscription(OccupancyGrid,"map",self.planPath,10)

        self.create_subscription(PoseStamped,"goal_pose",self.setGoal,10)
        self._action_client = ActionClient(self,FollowPath,'follow_path')

        self.path_publisher = self.create_publisher(Path,"path",10)

        self.resolution = 0.05

    
    
    def planPath(self, occupancy_grid: OccupancyGrid):
        # wait until we have a goal & have odom data
        if self.start == None or self.goal == None:
            return

        self.get_logger().info('start: "%s", goal: "%s"' % (self.start, self.goal))
        
        #get current (startIndx) pose and convert to occupancy grid location
        # startIndx = 
        #convert goal pose to occupancy grid location (if mode == 1)
        # if self.mode == 1:
            # goalIndx = self.convertToGrid()
        if self.planner == "ASTAR":
            path = Path()
            # TODO: populate header
            path.header.frame_id = 'slam_odom'

            DIAG = 0.05*math.sqrt(2)
            neighbor_offsets = {-1:0.05,-401:DIAG,-400:0.05,-399:DIAG,1:0.05,401:DIAG,400:0.05,399:DIAG}
            #convert start and goal pose to indices

            start_g = 0
            start_f = start_g + self.heuristic(self.start,self.goal)
            closed = set()
            came_from = {self.start:None}
            q = [(start_f,self.start)]
            f = {self.start:start_f}
            g = {self.start:start_g}

            while len(q) > 0:
                total_cost,current = heapq.heappop(q)
                # self.get_logger().info('Heap Pop: Total Cost + Current: "%s","%s"' % (total_cost,current))
                closed.add(current)
                if current == self.goal:
                    path = self.path_list(path,current,came_from)
                    self.get_logger().info('Publishing path Data: "%s"' % path)
                    self.path_publisher.publish(path)
                    path_follower_goal = FollowPath.Goal()
                    path_follower_goal.path = path
                    self._action_client.send_goal_async(path_follower_goal)
                    path = None
                    break
                for neighbor in neighbor_offsets:
                    # self.get_logger().info('Current + Neighbor: "%s","%s"' % (current,neighbor))
                    adjacent = current + neighbor
                    if occupancy_grid.data[adjacent] == 0 or occupancy_grid.data[adjacent] == -1:
                        cost = neighbor_offsets[neighbor]
                        if adjacent not in closed:
                            new_cost = cost + g[current]
                            if new_cost < g.get(adjacent,float('inf')):
                                g[adjacent] = new_cost
                                f[adjacent] = new_cost + self.heuristic(adjacent,self.goal)
                                came_from[adjacent] = current
                                # self.get_logger().info('Heap Push: Total Cost + Current: "%s","%s"' % (f[adjacent],adjacent))
                                heapq.heappush(q, (f[adjacent], adjacent))
        

        
        #convert drone_path to world coordinate
            
        # elif self.planner == "RRT":
        #     NUM_ITERS = 1000
        #     for i in range(NUM_ITERS):
        #         #pick random point
        # elif self.planner == "RRTSTAR":


        #         '''
        # Map Processing
        # '''
        # if self.mode == 2:
        #     frontier_cells = set()
        #     #8 point adjacency if 1d array
        #     neighbor_offsets = [-1,-401,-400,-399,1,401,400,399]
            
        #     #identifying frontier cells
        #     for i,cell in enumerate(occupancy_grid.data):
        #         if cell == 0:
        #             for offset in neighbor_offsets:
        #                 if occupancy_grid.data[i+offset] == -1:
        #                     frontier_cells.append(i+offset)
            
        #     #identifying frontier clusters
        #     clusters = []
        #     unvisitedSet = frontier_cells.copy()
        #     visitedSet = set()

        #     while len(unvisitedSet) > 0:
        #         cell = unvisitedSet.pop()
        #         cluster = []
        #         q = deque()
        #         q.append(cell)
        #         cluster.append(cell)

        #         while len(q) > 0:
        #             indx = q[0]
        #             q.popleft()
        #             visitedSet.add(indx)

        #             for offset in neighbor_offsets:
        #                 offsetIndx = offset + indx
        #                 if offsetIndx in unvisitedSet:
        #                     q.append(offsetIndx)
        #                     cluster.append(offsetIndx)
        #                     unvisitedSet.remove(offsetIndx)
        #         clusters.append(cluster)
            
        #     #calculate centroid of all clusters
            
        #     #find centroid closest to current location
            







        # return msg
    
    def setGoal(self, pose: PoseStamped):
        indx = self.convertToGrid(pose.pose)
        self.goal = indx
    def setStart(self, odom: Odometry):
        indx = self.convertToGrid(odom.pose.pose)
        self.start = indx
    
    def convertToGrid(self,pose: Pose):
        y = int((pose.position.y + 10)/0.05)
        x = int((pose.position.x + 10)/0.05)

        indx = x + (y * 400)
        return indx
        
    def convertToPose(self,indx):
        pose = 2
        return pose
    def heuristic(self,start,goal):
        #find row, col for start + goal
        rs,cs = int(start/400), start % 400
        rg,cg = int(goal/400), goal % 400
        
        #do euclidean for two sets of row,col
        h = math.sqrt((rs-rg)**2 + (cs-cg)**2)
        return h

    def path_list(self,path,goal,came_from):
        path_list = []
        node = goal
        
        while came_from[node] is not None:
            node = came_from[node]
            path_list.append(node)
        path_list.reverse()
        for node in path_list:
            #convert node to cartesian coordinates
            r,c = int(node/400), node % 400
            y = (r - 200) * 0.05
            x = (c - 200) * 0.05
            pose = PoseStamped()
            # pose.header # TODO
            pose.header.frame_id = 'slam_odom'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        goalPose = PoseStamped()
        goalPose.header.frame_id = 'slam_odom'
        r,c = int(node/400), node % 400
        y = (r - 200) * 0.05
        x = (c - 200) * 0.05
        goalPose.pose.position.x = x
        goalPose.pose.position.y = y
        goalPose.pose.position.z = 0.0
        goalPose.pose.orientation.x = 0.0
        goalPose.pose.orientation.y = 0.0
        goalPose.pose.orientation.z = 0.0
        goalPose.pose.orientation.w = 1.0
        path.poses.append(goalPose)
        

        # goalPose = PoseStamped()
        # path.append(goal)
        
        return path


def main(args=None):
    try:
        with rclpy.init(args=args):
            path_planner = PathPlanner()
            rclpy.spin(path_planner)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    # def _send_follow_path(self, path: Path):
    #     if not self._action_client.wait_for_server(timeout_sec=0.0):
    #         self.get_logger().warn('can't run')
    #         return

    #     goal_msg = FollowPath.Goal()
    #     goal_msg.path = path

    #     self._current_path_poses = list(path.poses)
    #     self._current_wp_idx     = 0

    #     send_future = self._action_client.send_goal_async(
    #         goal_msg, feedback_callback=self._feedback_cb)
    #     send_future.add_done_callback(self._goal_response_cb)


if __name__ == '__main__':
    main()



'''
Occupancy Map Additions
- Inflation Zone
    - Currently our occupancy map generator detects everywhere that is not an obstacle as "free"
    - However robot has a width which must be accounted for
    - Also some pose uncertainty
    - Solution: "Inflate obstacles by x m so that edges of vehicle do not collide with obstacle
'''




'''
Planners : These can be used for all three modes

A-Star + B-Spline
- Breadth first based planner
- Uses heuristic (euclidean distance) to speed up path planning process
- Benefits: Generates optimal path for given discretization
- Disadvantages: High computational cost
- Issue: path is likely not dynamically feasible for robot (turns are too tight)
- Solution: B-spline or other smoothing algos to smooth plot

RRT-Star + Dubins Curves
- Random Tree Planner
- Selects random points and uses dynamically feasible dubins curves to create paths between points
- RRT: much quicker. stops when it reaches goal. suboptimal
- RRT*: slower. continues for a predefined amount of time. probabilistically optimal (approaches optimality as time -> inf)
- Dubins curves: constant radius curves that are dynamically feasible for robot


Path Planning Process
1. Points that are free or unknown can be used to generate a path
2. During path execution, if a previously unknown cell becomes occupied, path execution is called again
    - To speed up path generation, select point between current location and goal as the interim goal


'''


'''
Exploration

1. Identify frontier points
    Frontier points are points directly adjacent to "unknown" points in the occupancy map
2. Group frontier points into frontier clusters
    Use breadth first search to group adjacent frontier points into "clusters"
3. Select next point to travel to (could be closest frontier cluster or largest frontier cluster)
4. Repeat until no frontier clusters are left (room is fully explored)
'''




