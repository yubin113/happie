import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import os
from math import sqrt

class makePath(Node):
    def __init__(self):
        super().__init__('make_path')

        self.path_pub = self.create_publisher(Path, 'global_path', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.listener_callback, 10)

        # ✅ ROS 2 패키지 경로를 src 기준으로 설정
        current_dir = os.path.dirname(os.path.abspath(__file__))  # 현재 스크립트 파일 위치
        pkg_path = os.path.abspath(os.path.join(current_dir, "..", ".."))  # src/my_package 위치
        path_folder = os.path.join(pkg_path, "path")  # path 폴더 경로

        # ✅ path 폴더가 없으면 생성
        os.makedirs(path_folder, exist_ok=True)

        file_name = "test.txt"
        full_path = os.path.join(path_folder, file_name)  # 최종 파일 경로

        self.f = open(full_path, 'w')

        self.is_odom = False
        self.prev_x = 0.0
        self.prev_y = 0.0

        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'

        # ✅ 저장 경로 확인 로그 출력
        self.get_logger().info(f"Saving path data to: {full_path}")

    def listener_callback(self, msg):
        if not self.is_odom:
            self.is_odom = True
            self.prev_x = msg.pose.pose.position.x
            self.prev_y = msg.pose.pose.position.y
            return  # 첫 번째 데이터는 저장하지 않음

        waypoint_pose = PoseStamped()
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        distance = sqrt((x - self.prev_x) ** 2 + (y - self.prev_y) ** 2)

        if distance > 0.1:
            waypoint_pose.pose.position.x = x
            waypoint_pose.pose.position.y = y
            waypoint_pose.pose.orientation.w = 1.0
            self.path_msg.poses.append(waypoint_pose)
            self.path_pub.publish(self.path_msg)

            data = f"{x}\t{y}\n"
            self.f.write(data)
            self.f.flush()  # 즉시 저장

            self.prev_x = x
            self.prev_y = y

def main(args=None):
    rclpy.init(args=args)
    odom_based_make_path = makePath()
    rclpy.spin(odom_based_make_path)

    # 프로그램 종료 전 파일 닫기
    odom_based_make_path.f.flush()
    odom_based_make_path.f.close()

    odom_based_make_path.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
