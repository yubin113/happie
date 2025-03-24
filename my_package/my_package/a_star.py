import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from collections import deque

class a_star(Node):

    def __init__(self):
        super().__init__('a_Star')
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 1)
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 1)
        self.a_star_pub = self.create_publisher(Path, 'global_path', 1)
        
        self.map_msg = None
        self.odom_msg = None
        self.is_map = False
        self.is_odom = False
        self.is_found_path = False
        self.is_grid_update = False

        self.goal = [184, 224] 
        self.map_size_x = 350
        self.map_size_y = 350
        self.map_resolution = 0.05
        self.map_offset_x = -16.75
        self.map_offset_y = -12.75
    
        self.GRIDSIZE = 350 
        self.dx = [-1, 0, 0, 1, -1, -1, 1, 1]
        self.dy = [0, 1, -1, 0, -1, 1, -1, 1]
        self.dCost = [1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414]

    def grid_update(self):
        self.is_grid_update = True
        self.grid = np.array(self.map_msg.data).reshape((self.map_size_y, self.map_size_x))

    def pose_to_grid_cell(self, x, y):
        map_point_x = int((x - self.map_offset_x) / self.map_resolution)
        map_point_y = int((y - self.map_offset_y) / self.map_resolution)
        return map_point_x, map_point_y

    def grid_cell_to_pose(self, grid_cell):
        x = (grid_cell[0] * self.map_resolution) + self.map_offset_x
        y = (grid_cell[1] * self.map_resolution) + self.map_offset_y
        return [x, y]

    def odom_callback(self, msg):
        self.is_odom = True
        self.odom_msg = msg

    def map_callback(self, msg):
        self.is_map = True
        self.map_msg = msg

    def goal_callback(self, msg):
        if msg.header.frame_id == 'map':
            goal_x = msg.pose.position.x
            goal_y = msg.pose.position.y
            goal_cell = self.pose_to_grid_cell(goal_x, goal_y)
            self.goal = goal_cell

            print(
            f"geometry_msgs.msg.PoseStamped(header=std_msgs.msg.Header("
            f"stamp=builtin_interfaces.msg.Time(sec={msg.header.stamp.sec}, "
            f"nanosec={msg.header.stamp.nanosec}), frame_id='{msg.header.frame_id}'), "
            f"pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point("
            f"x={goal_x:.6f}, y={goal_y:.6f}, z={msg.pose.position.z:.6f}), "
            f"orientation=geometry_msgs.msg.Quaternion("
            f"x={msg.pose.orientation.x:.6f}, y={msg.pose.orientation.y:.6f}, "
            f"z={msg.pose.orientation.z:.6f}, w={msg.pose.orientation.w:.6f})))")
            
            if self.is_map and self.is_odom:
                if not self.is_grid_update:
                    self.grid_update()

                self.final_path = []
                x = self.odom_msg.pose.pose.position.x
                y = self.odom_msg.pose.pose.position.y
                start_grid_cell = self.pose_to_grid_cell(x, y)
                
                self.path = np.zeros((self.GRIDSIZE, self.GRIDSIZE), dtype=int)
                self.cost = np.full((self.GRIDSIZE, self.GRIDSIZE), self.GRIDSIZE*self.GRIDSIZE)

                if self.grid[start_grid_cell[0], start_grid_cell[1]] == 0 and \
                   self.grid[self.goal[0], self.goal[1]] == 0 and \
                   start_grid_cell != self.goal:
                    self.dijkstra(start_grid_cell)

                self.global_path_msg = Path()
                self.global_path_msg.header.frame_id = 'map'
                for grid_cell in reversed(self.final_path):
                    tmp_pose = PoseStamped()
                    waypoint_x, waypoint_y = self.grid_cell_to_pose(grid_cell)
                    tmp_pose.pose.position.x = waypoint_x
                    tmp_pose.pose.position.y = waypoint_y
                    tmp_pose.pose.orientation.w = 1.0
                    self.global_path_msg.poses.append(tmp_pose)
                
                if self.final_path:
                    self.a_star_pub.publish(self.global_path_msg)

    def dijkstra(self, start):
        Q = deque()
        Q.append(start)
        self.cost[start[0]][start[1]] = 1
        found = False

        while Q:
            current = Q.popleft()
            if current == self.goal:
                found = True
                break

            for i in range(8):
                next_node = (current[0] + self.dx[i], current[1] + self.dy[i])
                if 0 <= next_node[0] < self.GRIDSIZE and 0 <= next_node[1] < self.GRIDSIZE:
                    if self.grid[next_node[0], next_node[1]] < 50:
                        if self.cost[next_node[0], next_node[1]] > self.cost[current[0], current[1]] + self.dCost[i]:
                            Q.append(next_node)
                            self.path[next_node[0]][next_node[1]] = current
                            self.cost[next_node[0]][next_node[1]] = self.cost[current[0]][current[1]] + self.dCost[i]
        
        if found:
            node = self.goal
            while node != start:
                self.final_path.append(node)
                node = self.path[node[0]][node[1]]
            self.final_path.append(start)
            self.final_path.reverse()
        

def main(args=None):
    rclpy.init(args=args)
    global_planner = a_star()
    rclpy.spin(global_planner)
    global_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
