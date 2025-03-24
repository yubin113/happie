import rclpy
import numpy as np
from rclpy.node import Node
import os
from geometry_msgs.msg import Pose, PoseStamped
from squaternion import Quaternion
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData, Path
from math import pi, cos, sin
from collections import deque

class a_star(Node):
    def __init__(self):
        super().__init__('a_star')
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 1)
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 1)
        self.a_star_pub = self.create_publisher(Path, 'global_path', 1)

        self.map_msg = OccupancyGrid()
        self.odom_msg = Odometry()
        self.is_map = False
        self.is_odom = False
        self.is_grid_update = False

        self.goal = [184, 224]
        self.map_size_x = 350
        self.map_size_y = 350
        self.map_resolution = 0.05
        self.map_offset_x = -8 - 8.75
        self.map_offset_y = -4 - 8.75

        self.GRIDSIZE = 350
        self.dx = [-1, 0, 0, 1, -1, -1, 1, 1]
        self.dy = [0, 1, -1, 0, -1, 1, -1, 1]
        self.dCost = [1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414]

    def grid_update(self):
        self.is_grid_update = True
        self.grid = np.array(self.map_msg.data).reshape(self.map_size_x, self.map_size_y)

    def pose_to_grid_cell(self, x, y):
        map_point_x = int((x - self.map_offset_x) / self.map_resolution)
        map_point_y = int((y - self.map_offset_y) / self.map_resolution)
        return map_point_x, map_point_y

    def grid_cell_to_pose(self, grid_cell):
        x = grid_cell[0] * self.map_resolution + self.map_offset_x
        y = grid_cell[1] * self.map_resolution + self.map_offset_y
        return [x, y]

    def odom_callback(self, msg):
        self.is_odom = True
        self.odom_msg = msg

    def map_callback(self, msg):
        self.is_map = True
        self.map_msg = msg

    def goal_callback(self, msg):
    # RViz2에서 2D Goal Pose를 클릭하면 실행되는 콜백 함수 
    
        if msg.header.frame_id == 'map':
            goal_x = msg.pose.position.x
            goal_y = msg.pose.position.y
            goal_cell = self.pose_to_grid_cell(goal_x, goal_y)
            self.goal = goal_cell  # 목표 위치 설정

            # CMD에서 원하는 메시지 형식으로 출력 (geometry_msgs.msg.PoseStamped 형태)
            print(f"geometry_msgs.msg.PoseStamped (header=std_msgs.msg.Header("
                  f"stamp=builtin_interfaces.msg.Time (sec={msg.header.stamp.sec}, "
                  f"nanosec={msg.header.stamp.nanosec}), frame_id='{msg.header.frame_id}'), "
                  f"pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point("
                  f"x={msg.pose.position.x}, y={msg.pose.position.y}, z={msg.pose.position.z}), "
                  f"orientation=geometry_msgs.msg.Quaternion(x={msg.pose.orientation.x}, "
                  f"y={msg.pose.orientation.y}, z={msg.pose.orientation.z}, w={msg.pose.orientation.w})))")

            print(f"Goal set to: {self.goal}")  # 변환된 grid 좌표 출력

            # 경로 탐색 실행
            if self.is_map and self.is_odom:
                if not self.is_grid_update:
                    self.grid_update()

                self.final_path = []
                x = self.odom_msg.pose.pose.position.x
                y = self.odom_msg.pose.pose.position.y
                start_grid_cell = self.pose_to_grid_cell(x, y)

                self.path = [[None for _ in range(self.GRIDSIZE)] for _ in range(self.GRIDSIZE)]
                self.cost = np.full((self.GRIDSIZE, self.GRIDSIZE), np.inf)

                if self.grid[start_grid_cell[0]][start_grid_cell[1]] == 0 and self.grid[self.goal[0]][self.goal[1]] == 0 and start_grid_cell != self.goal:
                    self.dijkstra(start_grid_cell)

                # 전역 경로 메시지 생성 및 퍼블리시
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
        self.cost[start[0]][start[1]] = 0
        
        while Q:
            current = Q.popleft()
            if current == self.goal:
                break

            for i in range(8):
                next_cell = (current[0] + self.dx[i], current[1] + self.dy[i])
                if 0 <= next_cell[0] < self.GRIDSIZE and 0 <= next_cell[1] < self.GRIDSIZE:
                    if self.grid[next_cell[0]][next_cell[1]] < 50:
                        new_cost = self.cost[current[0]][current[1]] + self.dCost[i]
                        if new_cost < self.cost[next_cell[0]][next_cell[1]]:
                            Q.append(next_cell)
                            self.path[next_cell[0]][next_cell[1]] = current
                            self.cost[next_cell[0]][next_cell[1]] = new_cost

        node = self.goal
        while node and self.path[node[0]][node[1]] is not None:
            self.final_path.append(node)
            node = self.path[node[0]][node[1]]


def main(args=None):
    rclpy.init(args=args)
    global_planner = a_star()
    rclpy.spin(global_planner)
    global_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
