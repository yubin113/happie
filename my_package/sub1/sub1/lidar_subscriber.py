import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LiDARSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        
        # /scan 토픽 구독
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def lidar_callback(self, msg):
        ranges = msg.ranges  # 거리 데이터 리스트
        min_distance = min(ranges)  # 가장 가까운 장애물 거리
        max_distance = max(ranges)  # 가장 먼 거리 측정값

        # LiDAR 세부 정보 출력
        self.get_logger().info(
            f"\nLiDAR Data Summary\n"
            f"----------------------------\n"
            f"Detectable distance range: {msg.range_min:.2f}m ~ {msg.range_max:.2f}m\n"
            f"Measurable angle range: {msg.angle_min:.2f} rad ~ {msg.angle_max:.2f} rad\n"
            f"Angle increment: {msg.angle_increment:.4f} rad\n"
            f"Closest detected distance: {min_distance:.2f} m\n"
            f"Farthest detected distance: {max_distance:.2f} m\n"
        )

        # 특정 방향의 거리값 (정면, 좌우, 후방)
        total_points = len(ranges)
        front_idx = total_points // 2  # 정면
        left_idx = int(total_points * 0.25)  # 왼쪽 45도
        right_idx = int(total_points * 0.75)  # 오른쪽 45도
        back_idx = 0  # 후방

        self.get_logger().info(
            f"Distance Measurements by Direction\n"
            f"Front: {ranges[front_idx]:.2f} m\n"
            f"Left 45-Degree: {ranges[left_idx]:.2f} m\n"
            f"Right 45-Degree: {ranges[right_idx]:.2f} m\n"
            f"Back: {ranges[back_idx]:.2f} m\n"
        )

def main(args=None):
    rclpy.init(args=args)
    node = LiDARSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
