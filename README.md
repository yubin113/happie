# README

## 2025.03.07 학습 내용

### 1. IMU (Inertial Measurement Unit)
IMU는 관성 센서로, 회전하거나 일정한 비율로 가속되는 기준좌표계 안에서 뉴턴의 제2 법칙을 만족하도록 가상적으로 도입한 힘을 측정합니다. IMU는 가속도 센서와 자이로 센서로 구성됩니다. 이들 센서는 서로의 부족한 점을 보완하며, 로봇의 자세를 추정하는 데 사용됩니다. 자세는 이동(Translation)과 회전(Rotation)으로 정의되며, 로봇은 3 DOF(자유도)를 가집니다. 비행기의 경우 6 DOF입니다.

- 자이로 센서: 회전하는 물체의 각속도를 측정하며, 각속도를 적분하여 각도를 계산할 수 있습니다.
- 가속도 센서: 물체의 가속도와 진동을 측정합니다. 가속도 센서만으로는 기울기를 정확히 알 수 없기 때문에 자이로 센서가 보완합니다.

IMU의 출력은 Quaternion을 사용하여 오일러 각(roll, pitch, yaw)으로 변환할 수 있습니다. Quaternion은 오일러 각에서 발생할 수 있는 짐벌락 현상을 방지하는 장점이 있어, 주로 컴퓨터 그래픽스에서 사용됩니다.

#### 관련 링크:
- [오일러 각](https://ko.wikipedia.org/wiki/%EC%98%A4%EC%9D%BC%EB%9F%AC%20%EA%B0%81)
- [squaternion 파이썬 모듈](https://pypi.org/project/squaternion/)
- [IMU ROS 메시지](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html)
- [IMU Wiki](https://en.wikipedia.org/wiki/Inertial_measurement_unit)

### 2. 주행기록계 (Odometry)
주행기록계는 로봇의 기구학 모델을 이용하여 로봇의 이동을 추정합니다. 이때 추정되는 위치는 절대적인 위치가 아니라, 주행기록을 시작한 위치를 기준으로 한 상대 위치입니다. 로봇의 각 바퀴 속도와 선속도, 각속도를 이용하여 자세(x, y, δ)의 변화량을 구하고 이를 누적시켜 Odometry를 생성합니다.

#### Kinematic Odometry
로봇에 부착된 모터의 속도를 측정하여 주행기록을 추정합니다. 주행에 따라 오차가 누적될 수 있으며, 이를 보정하기 위해 IMU에서 측정한 각속도 또는 Quaternion을 활용할 수 있습니다.

#### 관련 링크:
- [Kinematics](https://en.wikipedia.org/wiki/Kinematics)
- [Visual Odometry](https://en.wikipedia.org/wiki/Visual_odometry)
- [Odometry YouTube](https://www.youtube.com/results?search_query=odometry)

### 3. 경로 (Path Planning)
경로는 로봇이 주행한 기록을 저장하고, 이를 기반으로 테스트할 수 있는 방법입니다. 경로에는 전역 경로와 지역 경로가 있으며, 전역 경로는 출발점에서 목적지까지의 전체적인 경로를 의미하고, 지역 경로는 경로의 일부를 수정하여 새로운 경로를 만드는 방식입니다.

#### Follow the Carrot
이동 로봇에 사용되는 기하학적 경로 추종 방법으로, 로봇이 목표거리만큼 떨어진 목표지향점을 향해 경로를 추종합니다. 경로 추종을 위한 각도와 속도는 목표지향점과 로봇의 위치 및 방향을 사용해 계산됩니다.

#### 관련 링크:
- [전역경로, 지역경로 계획](https://ko.wikipedia.org/wiki/%EA%B2%BD%EB%A1%9C_%ED%8F%AC%EB%8F%84_%EA%B5%AC%ED%98%84)

### 4. 기타 센서 기반 주행기록 (Visual Odometry, Lidar Odometry)

#### Visual Odometry
카메라를 이용하여 환경에서 추출된 특징점(feature)을 기반으로 이동 경로를 추정합니다. 특징점이 뚜렷한 환경에서 효과적입니다.

#### Lidar Odometry
Lidar 센서를 이용하여 환경의 변화를 기반으로 주행기록을 추정합니다. Lidar는 포인트들에 다양한 특징을 갖고 있어, 복잡한 환경에서 정확한 주행기록 추정이 가능합니다.

#### 오차 보정
Kinematic Odometry의 경우, 지면과의 마찰력 문제나 모터의 공회전으로 인해 오차가 누적될 수 있으며, 이를 IMU 데이터를 활용해 보정할 수 있습니다.

