"""
import rclpy
from rclpy.node import Node
import os, time
from ssafy_msgs.msg import TurtlebotStatus,HandControl

# Hand Control 노드는 시뮬레이터로부터 데이터를 수신해서 확인(출력)하고, 메세지를 송신해서 Hand Control기능을 사용해 보는 노드입니다. 
# 메시지를 받아서 Hand Control 기능을 사용할 수 있는 상태인지 확인하고, 제어 메시지를 보내 제어가 잘 되는지 확인해보세요. 
# 수신 데이터 : 터틀봇 상태 (/turtlebot_status)
# 송신 데이터 : Hand Control 제어 (/hand_control)


# 노드 로직 순서
# 1. publisher, subscriber 만들기
# 2. 사용자 메뉴 구성
# 3. Hand Control Status 출력
# 4. Hand Control - Preview
# 5. Hand Control - Pick up
# 6. Hand Control - Put down


class Handcontrol(Node):

    def __init__(self):
        super().__init__('hand_control')
                
        ## 로직 1. publisher, subscriber 만들기
        self.hand_control = self.create_publisher(HandControl, '/hand_control', 10)                
        self.turtlebot_status = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.turtlebot_status_cb,10)

        self.timer = self.create_timer(1, self.timer_callback)
        
        ## 제어 메시지 변수 생성 
        self.hand_control_msg=HandControl()        


        self.turtlebot_status_msg = TurtlebotStatus()
        self.is_turtlebot_status = False
        

    def timer_callback(self):
        # while True:
            # 로직 2. 사용자 메뉴 구성
            print('Select Menu [0: status_check, 1: preview, 2:pick_up, 3:put_down')
            menu=input(">>")
            if menu=='0' :               
                self.hand_control_status()
            if menu=='1' :
                self.hand_control_preview()               
            if menu=='2' :
                self.hand_control_pick_up()   
            if menu=='3' :
                self.hand_control_put_down()


    def hand_control_status(self):
        '''
        로직 3. Hand Control Status 출력
        '''

    def hand_control_preview(self):
        '''
        로직 4. Hand Control - Preview
        '''

    def hand_control_pick_up(self):
        '''
        로직 5. Hand Control - Pick up        
        '''
        
        
    def hand_control_put_down(self):        
        '''
        로직 6. Hand Control - Put down
        '''


    def turtlebot_status_cb(self,msg):
        self.is_turtlebot_status=True
        self.turtlebot_status_msg=msg
        

def main(args=None):
    rclpy.init(args=args)
    sub1_hand_control = Handcontrol()    
    rclpy.spin(sub1_hand_control)
    sub1_hand_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""
# import rclpy
# from rclpy.node import Node
# from ssafy_msgs.msg import TurtlebotStatus, HandControl

# # HandControl 모드 상수 정의
# NORMAL_MODE = 0
# PREVIEW_MODE = 1
# PICK_UP_MODE = 2
# PUT_DOWN_MODE = 3

# class HandControlNode(Node):
#     def __init__(self):
#         super().__init__('hand_control')
        
#         # Publisher & Subscriber 생성
#         self.hand_control_pub = self.create_publisher(HandControl, '/hand_control', 10)
#         self.turtlebot_status_sub = self.create_subscription(
#             TurtlebotStatus, '/turtlebot_status', self.turtlebot_status_cb, 10)

#         self.timer = self.create_timer(1, self.timer_callback)
        
#         # 메시지 변수 생성
#         self.hand_control_msg = HandControl()
#         self.turtlebot_status_msg = TurtlebotStatus()
#         self.is_turtlebot_status = False
    
#     def timer_callback(self):
#         print('Select Menu [0: status_check, 1: preview, 2: pick_up, 3: put_down]')
#         menu = input('>> ')
#         if menu == '0':               
#             self.hand_control_status()
#         elif menu == '1':
#             self.hand_control_preview()               
#         elif menu == '2':
#             self.hand_control_pick_up()   
#         elif menu == '3':
#             self.hand_control_put_down()
    
#     def hand_control_status(self):
#         """Hand Control Status 출력"""
#         if self.is_turtlebot_status:
#             print(f'Turtlebot Battery: {self.turtlebot_status_msg.battery_percentage}%')
#             print(f'Power Supply Status: {self.turtlebot_status_msg.power_supply_status}')
#             print(f'Can Use Hand: {self.turtlebot_status_msg.can_use_hand}')
#             print(f'Can Put: {self.turtlebot_status_msg.can_put}')
#             print(f'Can Lift: {self.turtlebot_status_msg.can_lift}')
#         else:
#             print('Turtlebot status not received yet.')
    
#     def hand_control_preview(self):
#         """Hand Control - Preview"""
#         if not self.turtlebot_status_msg.can_use_hand:
#             print('Hand control is not available!')
#             return
        
#         self.hand_control_msg.control_mode = PREVIEW_MODE
#         self.hand_control_pub.publish(self.hand_control_msg)
#         print('Preview mode activated.')
    
#     def hand_control_pick_up(self):
#         """Hand Control - Pick up"""
#         if not self.turtlebot_status_msg.can_lift:
#             print('Cannot pick up! Robot is not in the right state.')
#             return
        
#         self.hand_control_msg.control_mode = PICK_UP_MODE
#         self.hand_control_pub.publish(self.hand_control_msg)
#         print('Pick up command sent.')
    
#     def hand_control_put_down(self):        
#         """Hand Control - Put down"""
#         if not self.turtlebot_status_msg.can_put:
#             print('Cannot put down! Robot is not in the right state.')
#             return
        
#         self.hand_control_msg.control_mode = PUT_DOWN_MODE
#         self.hand_control_msg.put_distance = 0.5  # 적절한 거리 설정
#         self.hand_control_msg.put_height = 0.2  # 적절한 높이 설정
#         self.hand_control_pub.publish(self.hand_control_msg)
#         print('Put down command sent.')
    
#     def turtlebot_status_cb(self, msg):
#         """Turtlebot 상태 수신 콜백 함수"""
#         self.is_turtlebot_status = True
#         self.turtlebot_status_msg = msg


# def main(args=None):
#     rclpy.init(args=args)
#     hand_control_node = HandControlNode()
#     rclpy.spin(hand_control_node)
#     hand_control_node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
import os, time
from ssafy_msgs.msg import TurtlebotStatus, HandControl
from std_msgs.msg import Int16
import threading

# Hand Control 노드는 시뮬레이터로부터 데이터를 수신해서 확인(출력)하고, 메시지를 송신해서 Hand Control 기능을 테스트하는 노드입니다.
# 터틀봇의 상태를 구독하여 핸드 컨트롤이 가능한지 확인하고, 제어 명령을 발행하여 동작을 수행합니다.

class Handcontrol(Node):

    def __init__(self):
        super().__init__('hand_control')

        # 로직 1. publisher, subscriber 만들기
        # HandControl 메시지를 퍼블리시하여 핸드 컨트롤을 수행합니다.
        self.hand_control = self.create_publisher(HandControl, '/hand_control', 10)
        
        # TurtlebotStatus 메시지를 구독하여 터틀봇의 상태 정보를 받아옵니다.
        self.turtlebot_status = self.create_subscription(TurtlebotStatus, '/turtlebot_status', self.turtlebot_status_cb, 10)
        
        # Int16 메시지를 구독하여 외부에서 핸드 컨트롤 명령을 받아 실행합니다.
        self.hand_control_sub = self.create_subscription(Int16, '/hand_control_cmd', self.hand_control_cmd, 10)
        
        # 1초마다 `timer_callback` 실행 (제어 루프)
        self.timer = self.create_timer(1, self.timer_callback)

        # Hand Control 메시지 변수 생성
        self.hand_control_msg = HandControl()
        
        # Turtlebot 상태 메시지 저장 변수
        self.turtlebot_status_msg = TurtlebotStatus()
        self.is_turtlebot_status = False  # 터틀봇 상태를 정상적으로 수신했는지 여부

        # 현재 선택된 메뉴 옵션 (기본값 0)
        self.menu = 0

    def timer_callback(self):
        """
        주기적으로 실행되는 타이머 콜백 함수
        현재 메뉴 값에 따라 특정 핸드 컨트롤 기능을 실행합니다.
        """
        print(self.menu)
        
        if self.menu == 0:  # 핸드 컨트롤 상태 확인
            thread_status = threading.Thread(target=self.hand_control_status)
            thread_status.start()
        
        if self.menu == 2:  # 물체 잡기 (Pick up)
            thread_pick = threading.Thread(target=self.hand_control_pick_up)
            thread_pick.start()
        
        if self.menu == 3:  # 미리보기 후 물체 놓기 (Preview & Put Down)
            thread_preview = threading.Thread(target=self.hand_control_preview)
            thread_preview.start()
            
            thread_put = threading.Thread(target=self.hand_control_put_down)
            thread_put.start()
        
    def hand_control_status(self):
        """
        로직 3. Hand Control Status 출력
        현재 핸드 컨트롤 기능을 사용할 수 있는지 상태를 출력합니다.
        """
        if self.is_turtlebot_status:
            print("Hand Control Status:")
            print(f" - Preview 가능: {'가능' if self.turtlebot_status_msg.can_use_hand else '불가능'}")
            print(f" - 물체 놓기 가능: {'가능' if self.turtlebot_status_msg.can_put else '불가능'}")
            print(f" - 물체 집기 가능: {'가능' if self.turtlebot_status_msg.can_lift else '불가능'}")

    def hand_control_preview(self):
        """
        로직 4. Hand Control - Preview
        놓을 수 없는 곳을 확인하기 위해 사용됩니다.
        `can_put`이 False일 경우, 퍼블리시 요청을 보내서 시뮬레이션에서 빨간색으로 변경됩니다.
        """
        self.hand_control_msg.control_mode = 1  # Preview 모드
        self.hand_control_msg.put_distance = 0.3
        self.hand_control_msg.put_height = 0.3

        while not self.turtlebot_status_msg.can_put:
            self.hand_control.publish(self.hand_control_msg)
            time.sleep(0.1)  # 100ms 간격으로 퍼블리시

    def hand_control_pick_up(self):
        """
        로직 5. Hand Control - Pick up
        터틀봇이 물체를 잡을 수 있는 상태(`can_lift`)일 때만 실행됩니다.
        """
        self.hand_control_msg.control_mode = 2  # Pick up 모드
        
        while self.turtlebot_status_msg.can_lift:
            self.hand_control.publish(self.hand_control_msg)
            time.sleep(0.1)  # 100ms 간격으로 퍼블리시

    def hand_control_put_down(self):
        """
        로직 6. Hand Control - Put down
        터틀봇이 물체를 놓을 수 있는 상태(`can_put`)일 때만 실행됩니다.
        """
        self.hand_control_msg.control_mode = 3  # Put down 모드
        
        while self.turtlebot_status_msg.can_put:
            self.hand_control.publish(self.hand_control_msg)
            time.sleep(0.1)  # 100ms 간격으로 퍼블리시

    def turtlebot_status_cb(self, msg):
        """
        TurtlebotStatus 메시지를 구독하는 콜백 함수
        터틀봇의 현재 상태 정보를 업데이트합니다.
        """
        self.is_turtlebot_status = True
        self.turtlebot_status_msg = msg

    def hand_control_cmd(self, msg):
        """
        Hand Control 명령을 수신하는 콜백 함수
        외부에서 수신한 명령을 `menu` 변수에 반영하여 제어할 수 있도록 합니다.
        """
        self.menu = msg.data


def main(args=None):
    rclpy.init(args=args)
    sub1_hand_control = Handcontrol()
    rclpy.spin(sub1_hand_control)
    sub1_hand_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
