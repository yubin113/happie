import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from math import sqrt

class astarLocalpath(Node):
    def __init__(self):
        super().__init__('a_star_local_path')
        
        # 로직 1. publisher, subscriber 만들기
        self.local_path_pub = self.create_publisher(Path, 'local_path', 10)
        self.global_path_sub = self.create_subscription(Path, '/global_path', self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.listener_callback, 10)
        
        self.odom_msg = Odometry()
        self.global_path_msg = Path()
        self.is_odom = False
        self.is_path = False
        
        # 로직 3. 주기마다 실행되는 타이머함수 생성, local_path_size 설정
        time_period = 0.05
        self.timer = self.create_timer(time_period, self.timer_callback)
        self.local_path_size = 30
    
    def listener_callback(self, msg):
        self.is_odom = True
        self.odom_msg = msg
    
    def path_callback(self, msg):
        # 로직 2. global_path 데이터 수신 후 저장
        self.is_path = True
        self.global_path_msg = msg
    
    def timer_callback(self):
        if self.is_odom and self.is_path:
            local_path_msg = Path()
            local_path_msg.header.frame_id = 'map'
            
            x = self.odom_msg.pose.pose.position.x
            y = self.odom_msg.pose.pose.position.y
            current_waypoint = -1
            min_dis = float('inf')
            
            # 로직 4. global_path 중 로봇과 가장 가까운 포인트 계산
            for i, waypoint in enumerate(self.global_path_msg.poses):
                wp_x = waypoint.pose.position.x
                wp_y = waypoint.pose.position.y
                distance = sqrt((wp_x - x) ** 2 + (wp_y - y) ** 2)
                if distance < min_dis:
                    min_dis = distance
                    current_waypoint = i
            
            # 로직 5. local_path 예외 처리 및 생성
            if current_waypoint != -1:
                if current_waypoint + self.local_path_size < len(self.global_path_msg.poses):
                    local_path_msg.poses = self.global_path_msg.poses[current_waypoint:current_waypoint + self.local_path_size]
                else:
                    local_path_msg.poses = self.global_path_msg.poses[current_waypoint:]
            
            self.local_path_pub.publish(local_path_msg)
            

def main(args=None):
    rclpy.init(args=args)
    a_star_local = astarLocalpath()
    rclpy.spin(a_star_local)
    a_star_local.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
