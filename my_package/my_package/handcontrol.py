import rclpy
from rclpy.node import Node
from ssafy_msgs.msg import TurtlebotStatus, HandControl

# HandControl 모드 상수 정의
NORMAL_MODE = 0
PREVIEW_MODE = 1
PICK_UP_MODE = 2
PUT_DOWN_MODE = 3

class HandControlNode(Node):
    def __init__(self):
        super().__init__('hand_control')
        
        # Publisher & Subscriber 생성
        self.hand_control_pub = self.create_publisher(HandControl, '/hand_control', 10)
        self.turtlebot_status_sub = self.create_subscription(
            TurtlebotStatus, '/turtlebot_status', self.turtlebot_status_cb, 10)

        self.timer = self.create_timer(1, self.timer_callback)
        
        # 메시지 변수 생성
        self.hand_control_msg = HandControl()
        self.turtlebot_status_msg = TurtlebotStatus()
        self.is_turtlebot_status = False
    
    def timer_callback(self):
        print('Select Menu [0: status_check, 1: preview, 2: pick_up, 3: put_down]')
        menu = input('>> ')
        if menu == '0':               
            self.hand_control_status()
        elif menu == '1':
            self.hand_control_preview()               
        elif menu == '2':
            self.hand_control_pick_up()   
        elif menu == '3':
            self.hand_control_put_down()
    
    def hand_control_status(self):
        """Hand Control Status 출력"""
        if self.is_turtlebot_status:
            print(f'Turtlebot Battery: {self.turtlebot_status_msg.battery_percentage}%')
            print(f'Power Supply Status: {self.turtlebot_status_msg.power_supply_status}')
            print(f'Can Use Hand: {self.turtlebot_status_msg.can_use_hand}')
            print(f'Can Put: {self.turtlebot_status_msg.can_put}')
            print(f'Can Lift: {self.turtlebot_status_msg.can_lift}')
        else:
            print('Turtlebot status not received yet.')
    
    def hand_control_preview(self):
        """Hand Control - Preview"""
        if not self.turtlebot_status_msg.can_use_hand:
            print('Hand control is not available!')
            return
        
        self.hand_control_msg.control_mode = PREVIEW_MODE
        self.hand_control_pub.publish(self.hand_control_msg)
        print('Preview mode activated.')
    
    def hand_control_pick_up(self):
        """Hand Control - Pick up"""
        if not self.turtlebot_status_msg.can_lift:
            print('Cannot pick up! Robot is not in the right state.')
            return
        
        self.hand_control_msg.control_mode = PICK_UP_MODE
        self.hand_control_pub.publish(self.hand_control_msg)
        print('Pick up command sent.')
    
    def hand_control_put_down(self):        
        """Hand Control - Put down"""
        if not self.turtlebot_status_msg.can_put:
            print('Cannot put down! Robot is not in the right state.')
            return
        
        self.hand_control_msg.control_mode = PUT_DOWN_MODE
        self.hand_control_msg.put_distance = 0.5  # 적절한 거리 설정
        self.hand_control_msg.put_height = 0.2  # 적절한 높이 설정
        self.hand_control_pub.publish(self.hand_control_msg)
        print('Put down command sent.')
    
    def turtlebot_status_cb(self, msg):
        """Turtlebot 상태 수신 콜백 함수"""
        self.is_turtlebot_status = True
        self.turtlebot_status_msg = msg


def main(args=None):
    rclpy.init(args=args)
    hand_control_node = HandControlNode()
    rclpy.spin(hand_control_node)
    hand_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()