import rclpy
from rclpy.node import Node
import ros2pkg
from geometry_msgs.msg import Twist, PoseStamped, Pose, TransformStamped
from ssafy_msgs.msg import TurtlebotStatus
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32
from squaternion import Quaternion
from nav_msgs.msg import Odometry, Path, OccupancyGrid, MapMetaData
from math import pi, cos, sin, sqrt
import tf2_ros
import os
import sub3.utils as utils
import numpy as np
import cv2
import time

# mapping node의 전체 로직 순서
# 1. publisher, subscriber, msg 생성
# 2. mapping 클래스 생성
# 3. 맵의 resolution, 중심좌표, occupancy에 대한 threshold 등의 설정 받기
# 4. laser scan 메시지 안의 ground truth pose 받기
# 5. lidar scan 결과 수신
# 6. map 업데이트 시작
# 7. pose 값을 받아서 좌표변환 행렬로 정의
# 8. laser scan 데이터 좌표 변환
# 9. pose와 laser의 grid map index 변환
# 10. laser scan 공간을 맵에 표시
# 11. 업데이트 중인 map publish
# 12. 맵 저장

params_map = {
    "MAP_RESOLUTION": 0.05,
    "OCCUPANCY_UP": 0.02,
    "OCCUPANCY_DOWN": 0.01,
    "MAP_CENTER": (-4.0, 10.0),
    "MAP_SIZE": (17.5, 17.5),
    "MAP_FILENAME": 'test.png',
    "MAPVIS_RESIZE_SCALE": 2.0
}

import numpy as np

def createLineIterator(P1, P2, img):
    # Bresenham's line algorithm을 구현해서 이미지에 직선을 그리는 메소드

    imageH, imageW = img.shape[:2]  # 이미지의 높이와 너비
    P1X, P1Y = P1  # 시작점
    P2X, P2Y = P2  # 끝점

    # 로직 1: 두 점을 잇는 벡터의 x, y 값과 크기 계산
    dX = P2X - P1X
    dY = P2Y - P1Y
    dXa = np.abs(dX)
    dYa = np.abs(dY)

    # 로직 2: 직선을 그릴 grid map의 픽셀 좌표를 넣을 numpy array 정의
    max_len = max(dXa, dYa)
    itbuffer = np.empty((max_len, 3), dtype=np.float32)  # RGB 채널 포함
    itbuffer.fill(np.nan)  # 초기값을 NaN으로 설정

    # 로직 3: 직선 방향 체크
    negY = P1Y > P2Y  # Y 방향이 감소하는지 확인
    negX = P1X > P2X  # X 방향이 감소하는지 확인

    # 로직 4: 수직선의 픽셀 좌표 계산
    if P1X == P2X:
        itbuffer[:, 0] = P1X  # x 값은 일정
        itbuffer[:, 1] = np.arange(P1Y, P2Y, -1 if negY else 1)  # y 값만 증가/감소

    # 로직 5: 수평선의 픽셀 좌표 계산
    elif P1Y == P2Y:
        itbuffer[:, 1] = P1Y  # y 값은 일정
        itbuffer[:, 0] = np.arange(P1X, P2X, -1 if negX else 1)  # x 값만 증가/감소

    # 로직 6: 대각선의 픽셀 좌표 계산
    else:
        steepSlope = dYa > dXa  # 기울기가 1보다 큰지 여부
        slope = dY / dX if dX != 0 else np.inf  # 기울기 계산 (0으로 나누지 않도록 처리)

        if steepSlope:
            itbuffer[:, 1] = np.arange(P1Y, P2Y, -1 if negY else 1)  # y 좌표 계산
            itbuffer[:, 0] = P1X + (itbuffer[:, 1] - P1Y) / slope  # 기울기를 이용해 x 좌표 계산
        else:
            itbuffer[:, 0] = np.arange(P1X, P2X, -1 if negX else 1)  # x 좌표 계산
            itbuffer[:, 1] = P1Y + (itbuffer[:, 0] - P1X) * slope  # 기울기를 이용해 y 좌표 계산

    # 로직 7: 맵 바깥으로 나가는 픽셀 좌표 삭제
    itbuffer = itbuffer[(itbuffer[:, 0] >= 0) & (itbuffer[:, 0] < imageW) &
                        (itbuffer[:, 1] >= 0) & (itbuffer[:, 1] < imageH)]

    return itbuffer

class Mapping:
    # 사용자가 정의한 맵 설정을 받아서 회색의 어레이로 초기화 시키고,
    # 로봇의 pose와 2d 라이다 값들을 받은 다음,
    # 라이다가 나타내는 로봇으로부터 측정된 좌표와의 직선을
    # utils_skeleton.py에 있는 createLineIterator()로
    # 그려가면서 맵을 채워서 저장할 수 있도록 만든 스크립트입니다.

    def __init__(self, params_map):
        # 로직 3. 맵의 resolution, 중심좌표, occupancy에 대한 threshold 등의 설정들을 받습니다
        self.map_resolution = params_map["MAP_RESOLUTION"]
        self.map_size = np.array(params_map["MAP_SIZE"]) / self.map_resolution
        self.map_center = params_map["MAP_CENTER"]
        self.map = np.ones((self.map_size[0].astype(np.int), self.map_size[1].astype(np.int)))*0.5
        self.occu_up = params_map["OCCUPANCY_UP"]
        self.occu_down = params_map["OCCUPANCY_DOWN"]

        self.map_filename = params_map["MAP_FILENAME"]
        self.map_vis_resize_scale = params_map["MAPVIS_RESIZE_SCALE"]

        self.T_r_l = np.array([[0,-1,0],[1,0,0],[0,0,1]])


    def update(self, pose, laser):
        # 로직 7. pose 값을 받아서 좌표변환 행렬로 정의
        n_points = laser.shape[1]
        pose_mat = utils.xyh2mat2D(pose)


        # 로직 8. laser scan 데이터 좌표 변환
        pose_mat = np.matmul(pose_mat,self.T_r_l)
        laser_mat = np.ones((3, n_points))
        laser_mat[:2, :] = laser

        laser_global = np.matmul(pose_mat, laser_mat)
        
        # 로직 9. pose와 laser의 grid map index 변환
        pose_x = (pose[0] - self.map_center[0] + (self.map_size[0]*self.map_resolution)/2) / self.map_resolution
        pose_y = (pose[1] - self.map_center[1] + (self.map_size[1]*self.map_resolution)/2) / self.map_resolution
        laser_global_x = (laser_global[0, :] - self.map_center[0] + (self.map_size[0]*self.map_resolution)/2) / self.map_resolution
        laser_global_y = (laser_global[1, :] - self.map_center[1] + (self.map_size[1]*self.map_resolution)/2) / self.map_resolution

        
        # 로직 10. laser scan 공간을 맵에 표시
        for i in range(laser_global.shape[1]):
            p1 = np.array([pose_x, pose_y]).reshape(-1).astype(np.int32)
            p2 = np.array([laser_global_x[i], laser_global_y[i]]).astype(np.int32)

            line_iter = createLineIterator(p1, p2, self.map)

            if line_iter.shape[0] == 0:
                continue

            avail_x = line_iter[:, 0].astype(np.int32)
            avail_y = line_iter[:, 1].astype(np.int32)

            ## Empty
            self.map[avail_y[:-1], avail_x[:-1]] -= self.occu_down
            self.map[avail_y[:-1], avail_x[:-1]] = np.clip(self.map[avail_y[:-1], avail_x[:-1]], 0, 1)

            ## Occupied
            self.map[avail_y[-1], avail_x[-1]] += self.occu_up
            self.map[avail_y[-1], avail_x[-1]] = np.clip(self.map[avail_y[-1], avail_x[-1]], 0, 1)
                
        self.show_pose_and_points(pose, laser_global) 

    def __del__(self):
        # 로직 12. 종료 시 map 저장
        ## Ros2의 노드가 종료될 때 만들어진 맵을 저장하도록 def __del__과 save_map이 정의되어 있습니다
        self.save_map(())

    def save_map(self):
        map_clone = self.map.copy()
        cv2.imwrite(self.map_filename, map_clone*255)


    def show_pose_and_points(self, pose, laser_global):
        tmp_map = self.map.astype(np.float32)
        map_bgr = cv2.cvtColor(tmp_map, cv2.COLOR_GRAY2BGR)

        pose_x = (pose[0] - self.map_center[0] + (self.map_size[0]*self.map_resolution)/2) / self.map_resolution
        pose_y = (pose[1] - self.map_center[1] + (self.map_size[1]*self.map_resolution)/2) / self.map_resolution

        laser_global_x = (laser_global[0, :] - self.map_center[0] + (self.map_size[0]*self.map_resolution)/2) / self.map_resolution
        laser_global_y =  (laser_global[1, :] - self.map_center[1] + (self.map_size[1]*self.map_resolution)/2) / self.map_resolution

        for i in range(laser_global.shape[1]):
            (l_x, l_y) = np.array([laser_global_x[i], laser_global_y[i]]).astype(np.int)
            center = (l_x, l_y)
            cv2.circle(map_bgr, center, 1, (0,255,0), -1)

        center = (pose_x.astype(np.int32)[0], pose_y.astype(np.int32)[0])
        
        cv2.circle(map_bgr, center, 2, (0,0,255), -1)

        map_bgr = cv2.resize(map_bgr, dsize=(0, 0), fx=self.map_vis_resize_scale, fy=self.map_vis_resize_scale)
        cv2.imshow('Sample Map', map_bgr)
        cv2.waitKey(1)

class Mapper(Node):
    def __init__(self):
        super().__init__('Mapper')
        # 로직 1 : publisher, subscriber, msg 생성
        self.subscription = self.create_subscription(LaserScan,
        '/scan',self.scan_callback,10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 1)
        
        self.map_msg=OccupancyGrid()
        self.map_msg.header.frame_id="map"
        self.map_size=int(params_map["MAP_SIZE"][0]\
            /params_map["MAP_RESOLUTION"]*params_map["MAP_SIZE"][1]/params_map["MAP_RESOLUTION"])
        

        m = MapMetaData()
        m.resolution = params_map["MAP_RESOLUTION"]
        m.width = int(params_map["MAP_SIZE"][0]/params_map["MAP_RESOLUTION"])
        m.height = int(params_map["MAP_SIZE"][1]/params_map["MAP_RESOLUTION"])
        quat = np.array([0, 0, 0, 1])
        m.origin = Pose()
        m.origin.position.x = params_map["MAP_CENTER"][0]-8.75
        m.origin.position.y = params_map["MAP_CENTER"][1]-8.75
        self.map_meta_data = m

        self.map_msg.info=self.map_meta_data

        # 로직 2 : mapping 클래스 생성
        self.mapping = Mapping(params_map)

        # 맵 저장 주기 설정 (예: 10초)
        self.save_interval = 10.0
        self.last_save_time = time.time()

        # 주기적인 맵 저장을 위한 타이머 생성
        self.create_timer(1.0, self.check_save_map)

    def check_save_map(self):
        current_time = time.time()
        if current_time - self.last_save_time >= self.save_interval:
            save_map(self, 'periodically_saved_map.txt')
            self.last_save_time = current_time

    
    def scan_callback(self, msg):
        
        # 로직 4 : laser scan 메시지 안의 ground truth pose 받기
        # - LaserScan 메시지 내에서 range_min, scan_time, time_increment 값을 사용하여 pose 추정

        # LaserScan 메시지에서 ground truth pose 정보 추출
        range_min = msg.range_min  # 최소 거리 (x 좌표로 사용)
        scan_time = msg.scan_time  # 스캔 시간 (y 좌표로 사용)
        time_increment = msg.time_increment  # 시간 간격 (theta로 사용)

        # range_min을 x로, scan_time을 y로, time_increment를 heading (theta)로 사용
        pose_x = range_min  # range_min을 x 좌표로 사용
        pose_y = scan_time  # scan_time을 y 좌표로 사용
        heading = time_increment  # time_increment을 heading (theta) 값으로 사용

        # 로직 5 : lidar scan 결과 수신
        # - LaserScan 메시지의 ranges 값을 기반으로 레이저 데이터를 처리하여 좌표로 변환

        distance = np.array(msg.ranges)  # LaserScan의 거리 데이터 (길이 360의 배열)

        # 각도 계산 (1도씩 증가하므로, 각도를 라디안으로 변환)
        angles = np.linspace(0, 2 * np.pi, len(distance), endpoint=False)  # 360개의 각도 생성 (0에서 2π까지)

        # 거리 데이터를 기반으로 x, y 좌표 계산
        x = distance * np.cos(angles)  # x = 거리 * cos(각도)
        y = distance * np.sin(angles)  # y = 거리 * sin(각도)

        laser = np.vstack((x, y))  # x, y 값을 결합하여 레이저 좌표를 생성

        # 로직 6 : map 업데이트 실행
        pose = np.array([[pose_x], [pose_y], [heading]])
        self.mapping.update(pose, laser)
        
        np_map_data = self.mapping.map.reshape(-1)
        list_map_data = [100 - int(value * 100) for value in np_map_data]
        list_map_data = [max(0, min(100, v)) for v in list_map_data]
        
        # 로직 11 : 업데이트 중인 map publish
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_msg.data = list_map_data
        self.map_pub.publish(self.map_msg)


def save_map(node, file_path):
    # 로직 12 : 맵 저장
    pkg_path = r"C:\Users\SSAFY\Desktop\catkin_ws\src\sub3\sub3"  # 패키지 경로
    back_folder = '..'  # 상위 폴더를 지정하려는 경우
    folder_name = 'map'  # 맵을 저장할 폴더 이름
    file_name = file_path  # 저장할 파일 이름 (예: "map.txt")

    # 전체 경로 생성
    full_path = os.path.join(pkg_path, back_folder, folder_name, file_name)

    # 저장할 디렉토리가 없으면 생성
    os.makedirs(os.path.dirname(full_path), exist_ok=True)

    print(f"Saving map to: {full_path}")
    
    # txt 파일에 맵 데이터 저장
    with open(full_path, 'w') as f:
        # 'node.map_msg.data'가 리스트라면, 각 값을 공백으로 구분하여 저장
        data = ' '.join(map(str, node.map_msg.data))  
        f.write(data)

    

def main(args=None):    
    rclpy.init(args=args)
    
    try:    
        run_mapping = Mapper()
        rclpy.spin(run_mapping)
    
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'run_mapping' in locals():
            print('최종 map 저장')
            save_map(run_mapping, 'map.txt')
            run_mapping.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
