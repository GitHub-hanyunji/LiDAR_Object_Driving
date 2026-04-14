# launch.py

### 작성자: 2301510 한윤지

`camera_ros2`, `dxl_nano`, `sllidar_ros2` 세 개의 노드를 동시에 실행하는 런치 파일이다.
카메라 영상 퍼블리시, LiDAR 스캔 퍼블리시, Dynamixel 구독을 한 번에 시작할 수 있다.

---

## 실행 노드 구성

| 패키지 | executable | 노드 이름 | 역할 |
|---|---|---|---|
| `camera_ros2` | `pub` | `campub` | 카메라 영상 토픽 발행 |
| `dxl_nano` | `sub` | `node_dxlsub` | 속도 명령 토픽 구독 → Dynamixel 구동 |
| `sllidar_ros2` | `sllidar_node` | `scan` | LiDAR `/scan` 토픽 발행 |

---

## 실행 방법
launch 파일은 ros 작업디렉토리/launch/ 폴더 안에 작성되어있어야 한다.
```sh
ros2 launch launch.py
```

---

## 코드

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_ros2',
            executable='pub',
            name='campub',
            output='screen',
        ),
        Node(
            package='dxl_nano',
            executable='sub',
            name='node_dxlsub',
            output='screen',
        ),
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='scan',
            output='screen',
        )
    ])
```

---

## 노드 설명

#### 1. campub 노드 (`camera_ros2`)
카메라 영상을 캡처하여 토픽으로 발행하는 노드이다.
lidarsim 패키지에서 저장한 LiDAR 스캔 영상을 `CompressedImage` 토픽으로 발행한다.

```python
Node(
    package='camera_ros2',
    executable='pub',
    name='campub',
    output='screen',
)
```

---

#### 2. node_dxlsub 노드 (`dxl_nano`)
속도 명령 토픽을 구독하여 Dynamixel 모터를 구동하는 노드이다.
WSL2의 lidardrive 노드가 발행한 `topic_dxlpub` 토픽을 구독하고,
수신된 `l_vel`, `r_vel` 값에 따라 좌·우 바퀴를 제어한다.

```python
Node(
    package='dxl_nano',
    executable='sub',
    name='node_dxlsub',
    output='screen',
)
```

---

#### 3. scan 노드 (`sllidar_ros2`)
LiDAR 센서 데이터를 읽어 `/scan` 토픽으로 발행하는 노드이다.
WSL2의 lidardrive 노드가 `/scan` 토픽을 구독하여 장애물 회피 알고리즘을 수행한다.

```python
Node(
    package='sllidar_ros2',
    executable='sllidar_node',
    name='scan',
    output='screen',
)
```
