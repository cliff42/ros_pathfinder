import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_interfaces.action import FollowPath

from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path, OccupancyGrid, Odometry

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

import heapq
import math
from rclpy.executors import ExternalShutdownException

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        self.goal = None
        self.start = None
        self.goal_pose = None
        self.start_pose = None
        self.map_frame = self.declare_parameter('map_frame', 'slam_odom').value
        self.allow_unknown = bool(self.declare_parameter('allow_unknown', True).value)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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
        self.create_subscription(OccupancyGrid,"/map",self.planPath,10)

        self.create_subscription(PoseStamped,"/goal_pose",self.setGoal,10)
        self._action_client = ActionClient(self,FollowPath,'follow_path')
        self.path_goal_in_flight = False

        self.path_publisher = self.create_publisher(Path,"path",10)

        self.resolution = 0.05

    
    
    def planPath(self, occupancy_grid: OccupancyGrid):
        # wait until we have a goal & have odom data
        if self.start_pose is None or self.goal_pose is None:
            return
        if self.path_goal_in_flight:
            return

        self.map_frame = occupancy_grid.header.frame_id or self.map_frame
        self.start = self.pose_to_grid_index(occupancy_grid, self.start_pose)
        self.goal = self.pose_to_grid_index(occupancy_grid, self.goal_pose)

        if self.start is None or self.goal is None:
            return

        self.get_logger().info(
            f'planning path: start={self.start}, goal={self.goal}, '
            f'frame={self.map_frame}, size={occupancy_grid.info.width}x{occupancy_grid.info.height}, '
            f'resolution={occupancy_grid.info.resolution:.3f}'
        )
        
        #get current (startIndx) pose and convert to occupancy grid location
        # startIndx = 
        #convert goal pose to occupancy grid location (if mode == 1)
        # if self.mode == 1:
            # goalIndx = self.convertToGrid()
        if self.planner == "ASTAR":
            path = Path()
            path.header.stamp = self.get_clock().now().to_msg()
            path.header.frame_id = self.map_frame

            width = occupancy_grid.info.width
            height = occupancy_grid.info.height
            resolution = occupancy_grid.info.resolution
            neighbor_steps = [
                (-1, 0, resolution),
                (1, 0, resolution),
                (0, -1, resolution),
                (0, 1, resolution),
                (-1, -1, resolution * math.sqrt(2.0)),
                (1, -1, resolution * math.sqrt(2.0)),
                (-1, 1, resolution * math.sqrt(2.0)),
                (1, 1, resolution * math.sqrt(2.0)),
            ]
            #convert start and goal pose to indices

            start_g = 0
            start_f = start_g + self.heuristic(self.start, self.goal, width, resolution)
            closed = set()
            came_from = {self.start:None}
            q = [(start_f,self.start)]
            f = {self.start:start_f}
            g = {self.start:start_g}
            found_path = False

            while len(q) > 0:
                total_cost,current = heapq.heappop(q)
                if current in closed:
                    continue
                # self.get_logger().info('Heap Pop: Total Cost + Current: "%s","%s"' % (total_cost,current))
                closed.add(current)
                if current == self.goal:
                    path = self.path_list(path,current,came_from,occupancy_grid)
                    self.log_path(path)
                    self.path_publisher.publish(path)
                    self.send_follow_path(path)
                    found_path = True
                    break
                current_x = current % width
                current_y = current // width
                for dx, dy, cost in neighbor_steps:
                    adjacent_x = current_x + dx
                    adjacent_y = current_y + dy
                    if not self.in_bounds(adjacent_x, adjacent_y, width, height):
                        continue

                    adjacent = adjacent_x + adjacent_y * width
                    if not self.is_traversable(occupancy_grid.data[adjacent]):
                        continue

                    if adjacent in closed:
                        continue

                    new_cost = cost + g[current]
                    if new_cost < g.get(adjacent,float('inf')):
                        g[adjacent] = new_cost
                        f[adjacent] = new_cost + self.heuristic(
                            adjacent,
                            self.goal,
                            width,
                            resolution
                        )
                        came_from[adjacent] = current
                        # self.get_logger().info('Heap Push: Total Cost + Current: "%s","%s"' % (f[adjacent],adjacent))
                        heapq.heappush(q, (f[adjacent], adjacent))

            if not found_path:
                self.get_logger().warn(
                    f'no path found: start={self.start}, goal={self.goal}'
                )
        

        
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
        self.goal_pose = pose
        self.goal = None
        self.path_goal_in_flight = False

        yaw = self.yaw_from_quaternion(
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        )
        self.get_logger().info(
            f'goal received: frame={pose.header.frame_id}, '
            f'x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}, yaw={yaw:.3f}'
        )

    def setStart(self, odom: Odometry):
        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.start_pose = pose
        self.start = None
    
    def convertToGrid(self,pose: Pose):
        y = int((pose.position.y + 10)/0.05)
        x = int((pose.position.x + 10)/0.05)

        indx = x + (y * 400)
        return indx
        
    def convertToPose(self,indx):
        pose = 2
        return pose

    def heuristic(self,start,goal,width,resolution):
        #find row, col for start + goal
        rs,cs = int(start/width), start % width
        rg,cg = int(goal/width), goal % width
        
        #do euclidean for two sets of row,col
        h = math.sqrt((rs-rg)**2 + (cs-cg)**2) * resolution
        return h

    def path_list(self,path,goal,came_from,occupancy_grid):
        path_nodes = []
        node = goal

        while node is not None:
            path_nodes.append(node)
            node = came_from[node]
        path_nodes.reverse()

        for node in path_nodes:
            x, y = self.grid_index_to_world(occupancy_grid, node)
            pose = PoseStamped()
            pose.header.stamp = path.header.stamp
            pose.header.frame_id = self.map_frame
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        return path

    def pose_to_grid_index(self, occupancy_grid: OccupancyGrid, pose: PoseStamped):
        map_pose = self.pose_in_frame(pose, occupancy_grid.header.frame_id)
        if map_pose is None:
            return None

        grid_x, grid_y = self.world_to_grid(occupancy_grid, map_pose.pose.position.x, map_pose.pose.position.y)
        if not self.in_bounds(grid_x, grid_y, occupancy_grid.info.width, occupancy_grid.info.height):
            self.get_logger().warn(
                f'pose outside map: frame={occupancy_grid.header.frame_id}, '
                f'x={map_pose.pose.position.x:.3f}, y={map_pose.pose.position.y:.3f}, '
                f'grid=({grid_x}, {grid_y})'
            )
            return None
        return grid_x + grid_y * occupancy_grid.info.width

    def pose_in_frame(self, pose: PoseStamped, frame_id: str):
        source_frame = pose.header.frame_id or frame_id
        if source_frame == frame_id:
            return pose

        try:
            transform = self.tf_buffer.lookup_transform(
                frame_id,
                source_frame,
                rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warn(
                f'could not transform pose {source_frame}->{frame_id}: {exc}'
            )
            return None

        transformed = PoseStamped()
        transformed.header.stamp = pose.header.stamp
        transformed.header.frame_id = frame_id

        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        q = transform.transform.rotation
        transform_yaw = self.yaw_from_quaternion(q.x, q.y, q.z, q.w)
        c = math.cos(transform_yaw)
        s = math.sin(transform_yaw)

        x = pose.pose.position.x
        y = pose.pose.position.y
        transformed.pose.position.x = tx + c * x - s * y
        transformed.pose.position.y = ty + s * x + c * y
        transformed.pose.position.z = pose.pose.position.z + transform.transform.translation.z

        pose_yaw = self.yaw_from_quaternion(
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        )
        yaw = pose_yaw + transform_yaw
        transformed.pose.orientation.x = 0.0
        transformed.pose.orientation.y = 0.0
        transformed.pose.orientation.z = math.sin(yaw / 2.0)
        transformed.pose.orientation.w = math.cos(yaw / 2.0)
        return transformed

    def world_to_grid(self, occupancy_grid: OccupancyGrid, x: float, y: float):
        origin = occupancy_grid.info.origin
        origin_yaw = self.yaw_from_quaternion(
            origin.orientation.x,
            origin.orientation.y,
            origin.orientation.z,
            origin.orientation.w,
        )
        dx = x - origin.position.x
        dy = y - origin.position.y
        c = math.cos(origin_yaw)
        s = math.sin(origin_yaw)
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy
        grid_x = int(local_x / occupancy_grid.info.resolution)
        grid_y = int(local_y / occupancy_grid.info.resolution)
        return grid_x, grid_y

    def grid_index_to_world(self, occupancy_grid: OccupancyGrid, index: int):
        width = occupancy_grid.info.width
        resolution = occupancy_grid.info.resolution
        column = index % width
        row = index // width

        local_x = (column + 0.5) * resolution
        local_y = (row + 0.5) * resolution

        origin = occupancy_grid.info.origin
        origin_yaw = self.yaw_from_quaternion(
            origin.orientation.x,
            origin.orientation.y,
            origin.orientation.z,
            origin.orientation.w,
        )
        c = math.cos(origin_yaw)
        s = math.sin(origin_yaw)
        world_x = origin.position.x + c * local_x - s * local_y
        world_y = origin.position.y + s * local_x + c * local_y
        return world_x, world_y

    def in_bounds(self, x, y, width, height):
        return 0 <= x < width and 0 <= y < height

    def is_traversable(self, value):
        return value == 0 or (self.allow_unknown and value == -1)

    def log_path(self, path):
        if not path.poses:
            self.get_logger().warn('path is empty')
            return

        first = path.poses[0].pose.position
        target = path.poses[min(1, len(path.poses) - 1)].pose.position
        final = path.poses[-1].pose.position
        initial_heading = math.atan2(target.y - first.y, target.x - first.x)
        self.get_logger().info(
            f'publishing path: poses={len(path.poses)}, frame={path.header.frame_id}, '
            f'first=({first.x:.3f}, {first.y:.3f}), '
            f'next=({target.x:.3f}, {target.y:.3f}), '
            f'final=({final.x:.3f}, {final.y:.3f}), '
            f'initial_heading={initial_heading:.3f}'
        )

    def yaw_from_quaternion(self, x, y, z, w):
        return math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z)
        )

    def send_follow_path(self, path):
        if not self._action_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn('follow_path action server is not available')
            return

        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        self.path_goal_in_flight = True
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.follow_path_goal_response)

    def follow_path_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().warn(f'follow_path goal request failed: {exc}')
            self.path_goal_in_flight = False
            return

        if not goal_handle.accepted:
            self.get_logger().warn('follow_path goal rejected')
            self.path_goal_in_flight = False
            return

        self.get_logger().info('follow_path goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.follow_path_result)

    def follow_path_result(self, future):
        try:
            result = future.result().result
        except Exception as exc:
            self.get_logger().warn(f'follow_path result failed: {exc}')
            self.path_goal_in_flight = False
            return

        self.get_logger().info(
            f'follow_path result: success={result.success}, message="{result.message}"'
        )
        self.path_goal_in_flight = False
        if result.success:
            self.goal = None


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

