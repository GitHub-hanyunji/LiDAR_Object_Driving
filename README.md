# LiDAR_Object_Driving
### 작성자: 2301510 한윤지
jetson의 sllidar_ros2 패키지의 sllidar_node로 jetson에서 LiDARScan publish 하고 wsl의 sliidar_ros2 패키지에서 subscrive 하여 2D로 시각화하고, 전방 장애물을 좌·우로 검출하여 가장 안전한 진행 방향을 계산한 뒤 그 방향에 맞춰 로봇의 좌·우 바퀴 속도를 계산해 dxl_nano에 publish하여 주행하는 LiDAR 기반 장애물 회피 주행 노드이다.

* 각 패키지에 대한 자세한 설명은 각 패키지 폴더 아래 README.md 파일로 자세히 작성하였습니다.
### pub 설명
- jetson 코드는 나중에 첨부하도록 하겠습니다.

### sub 설명
README: https://github.com/GitHub-hanyunji/LiDAR_Object_Driving/blob/main/wsl/README.md

### 결과영상 youtube
