# Lidar Object Detect Driving


## \<Projects\>

### 1. lidarsave
`/scan` 토픽을 구독하여 LiDAR 스캔 영상(2mx2m 영역)을 10fps mp4 동영상 파일로 저장하는 패키지
#### ▶ rapi5 
#### ▶ wsl

### \<결과영상\>

### 2. lidarsim
lidarsave 패키지에서 저장한 LiDAR 스캔 동영상을 입력으로 받아 장애물 회피 알고리즘을 시뮬레이션하는 패키지.
스캔 영상에서 에러를 계산하고 속도 명령을 `/vel_cmd` 토픽으로 발행하여 Dynamixel을 구동
#### ▶ rapi5
#### ▶ wsl

### \<결과영상\>

### 3. lidar object detect driving
실제 LiDAR(`/scan` 토픽)를 구독하여 장애물 회피 알고리즘을 실시간으로 수행하는 패키지.
전방 180도 영역에서 좌/우 최단거리 장애물을 검출하고 중앙 방향으로 속도 명령을 계산하여 자율주행
#### ▶ rapi5
#### ▶ wsl

### \<블록도\>
<img width="515" height="936" alt="image" src="https://github.com/user-attachments/assets/c6ec9d21-714c-40db-889c-c89166dee7e3" />


### \<결과영상\>
