"""
#!/ C:\Python37\python.exe

import numpy as np
import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

# image parser 노드는 이미지를 받아서 opencv 의 imshow로 윈도우창에 띄우는 역할을 합니다.
# 이를 resize나 convert를 사용하여 이미지를 원하는대로 바꿔보세요.

# 노드 로직 순서
# 1. image subscriber 생성
# 2. 카메라 콜백함수에서 compressed image 디코딩
# 3. 이미지 색 채널을 gray scale로 컨버팅
# 4. 이미지 resizing
# 5. 이미지 imshow


class IMGParser(Node):

    def __init__(self):
        super().__init__(node_name='image_convertor')

        # 로직 1. image subscriber 생성
        ## 아래와 같이 subscriber가 
        ## 미리 정의된 토픽 이름인 '/image_jpeg/compressed' 에서
        ## CompressedImage 메시지를 받도록 설정된다.

        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_jpeg/compressed',
            self.img_callback,
            10)
        

    def img_callback(self, msg):

        # 로직 2. 카메라 콜백함수에서 이미지를 클래스 내 변수로 저장
        ## msg.data 는 bytes로 되어 있고 이를 uint8로 바꾼 다음
        ## cv2 내의 이미지 디코딩 함수로 bgr 이미지로 바꾸세요.        

        np_arr = np.frombuffer(msg.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        '''
        로직 3. 이미지 색 채널을 gray scale로 컨버팅
        cv2. 내의 이미지 색 채널 컨터버로 bgr 색상을 gary scale로 바꾸십시오.

        img_gray = 

        '''

        '''
        로직 4. 이미지 resizing
        cv2를 사용해서 이미지를 원하는 크기로 바꿔보십시오.

        img_resize = 
        '''

        # 로직 5. 이미지 출력 (cv2.imshow)       
        
        cv2.imshow("img_bgr", img_bgr)
        # cv2.imshow("img_gray", img_gray)
        # cv2.imshow("resize and gray", img_resize)       
        
        cv2.waitKey(1)


def main(args=None):

    ## 노드 초기화 : rclpy.init 은 node의 이름을 알려주는 것으로, ROS master와 통신을 가능하게 해줍니다.
    rclpy.init(args=args)

    ## 메인에 돌릴 노드 클래스 정의 
    image_parser = IMGParser()

    ## 노드 실행 : 노드를 셧다운하기 전까지 종료로부터 막아주는 역할을 합니다
    rclpy.spin(image_parser)


if __name__ == '__main__':

    main()
"""
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import time
from sensor_msgs.msg import CompressedImage

class IMGParser(Node):
    def __init__(self):
        super().__init__('image_convertor')
        
        # 이미지 처리 속도 조절
        self.last_time = time.time()  # 마지막 이미지 처리 시간
        self.frame_skip = 5  # 5프레임마다 한 번 처리
        self.frame_count = 0

        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_jpeg/compressed',
            self.img_callback,
            10)
    
    def img_callback(self, msg):
        current_time = time.time()

        # 0.1초(10Hz) 이상 경과한 경우에만 처리
        if current_time - self.last_time < 0.1:
            return
        
        # 프레임 스킵 적용 (5프레임마다 1번 실행)
        self.frame_count += 1
        if self.frame_count % self.frame_skip != 0:
            return
        
        # 이미지 디코딩
        np_arr = np.frombuffer(msg.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img_bgr is None:
            self.get_logger().warn("Failed to decode image")
            return
        
        # 이미지 색 변환 및 크기 조정
        img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        img_resize = cv2.resize(img_gray, (640, 480))

        # 이미지 출력
        cv2.imshow("Original Image", img_bgr)
        cv2.imshow("Gray Image", img_gray)
        cv2.imshow("Resized Image", img_resize)
        cv2.waitKey(1)

        # 마지막 처리 시간 갱신
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    image_parser = IMGParser()
    rclpy.spin(image_parser)
    cv2.destroyAllWindows()  # 노드 종료 시 창 닫기

if __name__ == '__main__':
    main()

