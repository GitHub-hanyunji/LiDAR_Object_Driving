# Lidar Object Detect Driving


## \<Projects\>

### 1. lidarsave
`/scan` 토픽을 구독하여 LiDAR 스캔 영상(2mx2m 영역)을 10fps mp4 동영상 파일로 저장하는 패키지
#### ▶ rapi5
https://github.com/GitHub-hanyunji/Ros2_rapi5 <br>
sllidar_ros2 패키지에서 sllidar_node 사용

#### ▶ wsl
아래 REAMD.md 파일에 상세 설명되어 있음<br>
https://github.com/GitHub-hanyunji/LiDAR_Object_Driving/tree/main/rapi5/wsl/lidarsave

### \<결과영상\>
https://github.com/user-attachments/assets/09a2a884-8dc3-4794-a441-9246fd0f249b


### 2. lidarsim
lidarsave 패키지에서 저장한 LiDAR 스캔 동영상을 입력으로 받아 장애물 회피 알고리즘을 시뮬레이션하는 패키지.
스캔 영상에서 에러를 계산하고 속도 명령을 `/vel_cmd` 토픽으로 발행하여 Dynamixel을 구동
#### ▶ rapi5
https://github.com/GitHub-hanyunji/LiDAR_Object_Driving/tree/main/rapi5/rapi5 <br>
=> launch.py 사용 -> sllidar_ros2와 dxl_nano 패키지가 있음 

#### ▶ wsl
아래 REAMD.md 파일에 상세 설명되어 있음<br>
https://github.com/GitHub-hanyunji/LiDAR_Object_Driving/tree/main/rapi5/wsl/lidarsim

### \<결과영상\>
[![Demo Video](https://img.youtube.com/vi/mBE5Uw5gzeE/0.jpg)](https://www.youtube.com/watch?v=mBE5Uw5gzeE)

### 3. lidar object detect driving
실제 LiDAR(`/scan` 토픽)를 구독하여 장애물 회피 알고리즘을 실시간으로 수행하는 패키지.
전방 180도 영역에서 좌/우 최단거리 장애물을 검출하고 중앙 방향으로 속도 명령을 계산하여 자율주행
#### ▶ rapi5
https://github.com/GitHub-hanyunji/LiDAR_Object_Driving/tree/main/rapi5/rapi5 <br>
=> launch.py 사용 -> sllidar_ros2와 dxl_nano 패키지가 있음 

#### ▶ wsl
아래 REAMD.md 파일에 상세 설명되어 있음<br>
https://github.com/GitHub-hanyunji/LiDAR_Object_Driving/tree/main/rapi5/wsl/lidar_driving

### \<블록도\>
<img width="571" height="932" alt="image" src="https://github.com/user-attachments/assets/6c15c5b4-6e42-441d-8b69-3292e8f70354" />



### \<결과영상\>
[![Demo Video](https://img.youtube.com/vi/d_WCQva6Nt0/0.jpg)](https://www.youtube.com/watch?v=d_WCQva6Nt0)
