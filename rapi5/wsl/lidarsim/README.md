
# lidarsim.cpp / lidarsim.hpp / main.cpp

### 작성자: 2301510 한윤지

`lidarsave` 패키지에서 저장한 LiDAR 스캔 영상을 `CompressedImage` 토픽으로 수신하여
장애물 회피 알고리즘을 시뮬레이션하는 노드이다.
실제 LiDAR 대신 저장된 영상을 입력으로 사용하므로, 실제 로봇 없이도 알고리즘 검증이 가능하다.

CompressedImage 수신 → 디코딩 → ROI 설정 → 이진화 → 객체 분석 → 좌/우 최단거리 장애물 탐지 → 에러 계산 → 좌/우 속도 계산 → 속도 publish

---

## 파일 구성

| 파일 | 설명 |
|---|---|
| `lidarsim.hpp` | 클래스 선언, 멤버 변수 및 함수 정의 |
| `lidarsim.cpp` | 각 함수 구현부 |
| `main.cpp` | ROS2 초기화 및 노드 실행 |

---

## lidarsave와의 차이점

| 항목 | lidarsave | lidarsim |
|---|---|---|
| 입력 | `/scan` (LaserScan) | `/image/compressed_13` (CompressedImage) |
| 목적 | LiDAR 스캔 영상 저장 | 저장된 영상으로 시뮬레이션 |
| 출력 | `lidar.mp4` | `topic_dxlpub` + `output.mp4` |

---

## 클래스 구조 (`lidarsim.hpp`)

`rclcpp::Node`를 상속하는 `LiDARDetectNode` 클래스이다.
`/image/compressed_13` 토픽을 구독하고 `topic_dxlpub` 토픽을 발행한다.

```cpp
#ifndef LIDAR_DETECT_NODE_HPP
#define LIDAR_DETECT_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/compressed_image.hpp"
#include <geometry_msgs/msg/vector3.hpp>
#include <opencv2/opencv.hpp>

class LiDARDetectNode : public rclcpp::Node
{
public:
    LiDARDetectNode();

private:
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub; // 영상 구독
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub;          // 속도 발행

    int width, height;               // OpenCV frame 크기
    cv::Point p_center, roi_center;  // 로봇 중심점 / ROI 중심점
    cv::Point left_pt, right_pt;     // 좌/우 탐지 객체 중심점

    cv::VideoWriter writer;          // mp4 저장
    bool video_key = false;          // 녹화 ON/OFF 플래그

    bool left_ok  = false;           // 왼쪽 장애물 탐지 여부
    bool right_ok = false;           // 오른쪽 장애물 탐지 여부
    int range = 0;                   // 라이다 탐지 범위
    double left_dist, right_dist;    // 좌/우 객체까지의 거리
    double left_angle, right_angle;  // 좌/우 객체의 각도
    int left_idx = -1, right_idx = -1;
    geometry_msgs::msg::Vector3 vel_msg; // 발행할 속도 메시지
    bool mode = false;               // 주행 모드 플래그
    double k = 0.5;                  // 비례 제어 이득
    double mid_angle = 180.0;        // 중앙 진행 방향 각도 (초기값: 직진)

    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    int getch();
    bool kbhit();
    void Set(cv::Mat& frame);
    void FindObject(cv::Mat& frame, cv::Mat& labels, cv::Point& p_center,
                    cv::Mat& stats, cv::Mat& centroids);
    void Draw(cv::Mat& frame, cv::Mat stats, cv::Mat centroids,
              cv::Mat& label, int labels, cv::Point p_center);
};

#endif
```

---

## 함수 설명

#### 1. 생성자 `LiDARDetectNode()`
`/image/compressed_13` 토픽을 구독하고 `topic_dxlpub` 발행자를 초기화한다.
frame 크기(500×500), 중심점, 거리·각도 초기값을 설정한다.

```cpp
LiDARDetectNode::LiDARDetectNode() : Node("linedetect_wsl")
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    auto fn = std::bind(&LiDARDetectNode::image_callback, this, std::placeholders::_1);
    sub = this->create_subscription<sensor_msgs::msg::CompressedImage>(
          "/image/compressed_13", qos, fn);
    pub = this->create_publisher<geometry_msgs::msg::Vector3>("topic_dxlpub", qos);

    width = 500; height = 500;
    p_center = Point(width / 2, height / 2);
    left_dist = 1e9; right_dist = 1e9;
    left_angle = 0.0; right_angle = 0.0;
    vel_msg.x = vel_msg.y = vel_msg.z = 0.0;
}
```

---

#### 2. `Set()` 함수
입력 영상의 상단 1/2만 잘라서 ROI로 만들고, 그 안에서 빨간색 포인트만 추출해
객체 검출에 사용할 이진 영상으로 변환하는 전처리 함수이다.

```cpp
void LiDARDetectNode::Set(Mat& frame)
{
    frame = frame(Rect(Point(0, 0), Point(frame.cols, frame.rows / 2)));  // 상단 1/2 ROI
    Mat mask;
    inRange(
        frame,
        Scalar(0, 0, 150),   // 빨간색 최소값
        Scalar(80, 80, 255), // 빨간색 최대값
        mask
    );
    frame = mask;  // 흰색 = 빨간 객체, 검정 = 배경
    roi_center = Point(frame.cols / 2, frame.rows);
}
```

---

#### 3. `FindObject()` 함수
ROI 이진 영상에서 연결된 객체들을 분리한 뒤, 로봇 기준 좌·우 영역에서
가장 가까운 장애물 하나씩을 찾아 거리와 방향 각도를 계산하는 객체 추적 함수이다.

```cpp
void LiDARDetectNode::FindObject(Mat& frame, Mat& labels, Point& p_center,
                                  Mat& stats, Mat& centroids)
{
    int cnt = connectedComponentsWithStats(frame, labels, stats, centroids);
    int mid_x = frame.cols / 2;
    left_ok = false; right_ok = false;

    for (int i = 1; i < cnt; i++) {
        // 각 객체에서 로봇과 가장 가까운 점 탐색
        // 너무 먼 객체(>200px) 제외
        // 좌/우 각각 가장 가까운 객체 1개씩 선택
        // left_dist, left_angle / right_dist, right_angle 갱신
    }
}
```

---

#### 4. `Draw()` 함수
좌·우 장애물 방향을 화살표로 시각화하고, 그 사이의 안전한 진행 방향(`mid_angle`)을
계산해 중앙 화살표로 표시하는 시각화 함수이다.

| 상황 | mid_angle 계산 |
|---|---|
| 좌·우 모두 장애물 | `(left_angle + right_angle) / 2 + 180` |
| 오른쪽만 장애물 | `(right_angle + (-90)) / 2 + 180` |
| 왼쪽만 장애물 | `(left_angle + 90) / 2 + 180` |
| 장애물 없음 | `180` (직진) |

```cpp
arrowedLine(frame, p_center, left_arrow_end,  Scalar(0, 255, 0), 2);  // 왼쪽 (초록)
arrowedLine(frame, p_center, right_arrow_end, Scalar(255, 0, 0), 2);  // 오른쪽 (파랑)
arrowedLine(frame, p_center, mid_arrow_end,   Scalar(0, 0, 255), 2);  // 중앙 (빨강)
```

---

#### 5. `image_callback()` 함수
`/image/compressed_13` 토픽으로 수신된 CompressedImage를 디코딩하여
장애물 회피 알고리즘을 수행하고 속도를 발행하는 핵심 제어 루프이다.

```cpp
void LiDARDetectNode::image_callback(sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    // 1. CompressedImage 디코딩 → OpenCV Mat
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (frame.empty()) return;  // 디코딩 실패 시 종료

    // 2. video_key=true 이면 writer.write(frame) → mp4 저장
    // 3. Set(roi)      → ROI + 이진화
    // 4. FindObject()  → 좌/우 최단거리 장애물 검출
    // 5. Draw()        → mid_angle 계산 + 시각화
    // 6. error = mid_angle - 180
    // 7. leftvel  =   50 - k * error
    //    rightvel = -(50 + k * error)
    // 8. kbhit() 감지
    //      's' → video_key=true, mode=true  (녹화 + 주행 시작)
    //      'q' → video_key=false, mode=false (녹화 + 주행 중지)
    //      'r' → 라이다 탐지 범위 재설정
    // 9. mode=true  → vel_msg = (leftvel, rightvel, 0)
    //    mode=false → vel_msg = (0, 0, 0)
    // 10. pub->publish(vel_msg) → topic_dxlpub 발행
}
```

---

#### 6. `getch()` / `kbhit()` 함수
엔터 없이 키보드 입력을 즉시 감지하기 위한 터미널 제어 함수이다.

```cpp
int  LiDARDetectNode::getch();   // 키 하나 즉시 읽기
bool LiDARDetectNode::kbhit();   // 키 입력 대기 여부 확인 (non-blocking)
```

---

## main.cpp

ROS2를 초기화하고 `LiDARDetectNode`를 실행한다.
`rclcpp::spin()`으로 무한 대기하며 `/image/compressed_13` 메시지가 들어올 때마다 콜백을 호출한다.

```cpp
#include <lidarsim/lidarsim.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);                               // ROS2 초기화
    rclcpp::spin(std::make_shared<LiDARDetectNode>());      // 노드 실행 (무한 대기)
    rclcpp::shutdown();                                     // ROS2 종료
}
```

---

## 키보드 제어

| 키 | 동작 |
|---|---|
| `s` | 녹화 시작 (`output.mp4`) + 주행 시작 |
| `q` | 녹화 중지 + 주행 중지 |
| `r` | 라이다 탐지 범위 재설정 |

---

## 토픽

| 방향 | 토픽명 | 타입 | 설명 |
|---|---|---|---|
| 구독 | `/image/compressed_13` | `sensor_msgs/CompressedImage` | lidarsave로 저장한 LiDAR 스캔 영상 |
| 발행 | `topic_dxlpub` | `geometry_msgs/Vector3` | 좌·우 바퀴 속도 (x=lvel, y=rvel) |
