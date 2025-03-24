import rclpy
import numpy as np
from rclpy.node import Node

import os
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, MapMetaData

class LoadMap(Node):

    def __init__(self):
        super().__init__('load_map')
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', 1)
        self.timer = self.create_timer(1, self.timer_callback)

        # 로직 1: 맵 파라미터 설정
        self.map_size_x = 350
        self.map_size_y = 350
        self.map_resolution = 0.05
        self.map_offset_x = -16.75
        self.map_offset_y = -12.75
        self.map_data = [0] * (self.map_size_x * self.map_size_y)

        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = "map"
        self.map_meta_data = MapMetaData()
        self.map_meta_data.resolution = self.map_resolution
        self.map_meta_data.width = self.map_size_x
        self.map_meta_data.height = self.map_size_y
        self.map_meta_data.origin = Pose()
        self.map_meta_data.origin.position.x = self.map_offset_x
        self.map_meta_data.origin.position.y = self.map_offset_y
        self.map_msg.info = self.map_meta_data

        # 로직 2: 맵 데이터 읽고, 2차원 행렬로 변환
        map_dir = r"C:\Users\SSAFY\Desktop\catkin_ws\src\my_package\map"
        os.makedirs(map_dir, exist_ok=True)

        full_path = os.path.join(map_dir, 'map.txt')

        # 빈 맵 초기화 (모든 값 0)
        self.map_data = np.zeros((350, 350), dtype=np.int32)

        if os.path.exists(full_path):
            with open(full_path, 'r') as f:
                content = f.read().strip()  # 공백 제거 후 읽기
            
            # 파일이 비어 있으면 빈 맵 유지
            if content:
                line_data = list(map(int, content.split()))  # 정수 리스트로 변환

                # 데이터 개수 확인
                expected_size = 350 * 350
                if len(line_data) == expected_size:
                    self.map_data = np.array(line_data, dtype=np.int32).reshape((350, 350))
                    print("맵 데이터 로드 완료!")
                else:
                    print(f"⚠️ Warning: map.txt 데이터 개수 {len(line_data)}개가 {expected_size}개와 맞지 않음. 빈 맵 사용.")
            else:
                print("⚠️ Warning: map.txt가 비어 있음. 빈 맵 사용.")
        else:
            print(f"⚠️ Warning: {full_path} 파일이 존재하지 않음. 빈 맵 사용.")

        # 로직 3: 점유 영역(100) 근처 5x5 영역을 127로 설정
        grid = self.map_data.copy()
        for y in range(350):
            for x in range(350):
                if self.map_data[y][x] == 100:
                    for dy in range(-2, 3):
                        for dx in range(-2, 3):
                            ny, nx = y + dy, x + dx
                            if 0 <= ny < 350 and 0 <= nx < 350:
                                grid[ny][nx] = 127
        
        self.map_msg.data = grid.flatten().tolist()
        print('Map data processed successfully')

    def timer_callback(self):
        self.map_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        self.map_pub.publish(self.map_msg)


def main(args=None):
    rclpy.init(args=args)
    load_map = LoadMap()
    rclpy.spin(load_map)
    load_map.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
