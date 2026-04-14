# lidarsave.cpp

### 작성자: 2301510 한윤지

LiDAR의 `/scan` 토픽을 구독하여 2D로 시각화하고, 그 결과를 mp4 동영상 파일로 저장하는 노드이다.
흰 바탕(500×500) 위에 LiDAR 포인트를 빨간 점으로 그려 `lidar.mp4`로 저장한다.

라이다 `/scan` 구독 → 극좌표 → 2D 화면 좌표 변환 → 빨간 점 그리기 → 화면 출력 + mp4 저장

---

## 파일 구성

| 파일 | 설명 |
|---|---|
| `lidarsave.cpp` | 노드 전체 구현 (scanCb, camera_save, main 포함) |

---

## 함수 설명

#### 1. `camera_save()` 함수
VideoWriter를 최초 1회만 초기화하고, 이후 매 프레임을 mp4 파일에 저장하는 함수이다.
`writer_initialized` 플래그로 중복 초기화를 방지한다.

```cpp
void camera_save(const cv::Mat& frame, double fps, const std::string name,
                 cv::VideoWriter& writer, bool& writer_initialized)
{
    // writer가 초기화되지 않았을 때 1회만 open
    if (!writer_initialized) {
        int fourcc = cv::VideoWriter::fourcc('m','p','4','v');
        writer.open(name, fourcc, fps, cv::Size(frame.cols, frame.rows), true);
        if (writer.isOpened())
            writer_initialized = true;
    }
    // 매 프레임 저장
    if (writer_initialized)
        writer.write(frame);
}
```

---

#### 2. `scanCb()` 함수
`/scan` 토픽 콜백 함수이다.
수신된 LiDAR 데이터를 극좌표 → 2D 화면 좌표로 변환하여 흰 바탕 위에 빨간 점으로 그린다.
무한대(inf) 값은 `std::isfinite()`로 필터링하여 제외한다.

```cpp
static void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    // 500x500 흰 바탕 frame 초기화
    frame.setTo(cv::Scalar(255, 255, 255));

    int count = scan->ranges.size();  // 포인트 개수

    for (int i = 0; i < count; i++) {
        float r = scan->ranges[i];
        if (!std::isfinite(r)) continue;  // inf / nan 제외

        float theta = scan->angle_min + scan->angle_increment * i;
        float r_px  = r * 100;  // 1m = 100px

        // 극좌표 → 화면 좌표
        int px = 250 + r_px * sin(theta);
        int py = 250 + r_px * cos(theta);  // 전방 = 위쪽

        cv::circle(frame, cv::Point(px, py), 2, cv::Scalar(0, 0, 255), -1);  // 빨간 점
    }

    cv::imshow("frame", frame);
    cv::waitKey(1);
    camera_save(frame, 10, "lidar.mp4", lidar_writer, lidar_writer_initialized);
}
```

---

#### 3. `main()` 함수
ROS2를 초기화하고 `sllidar_client` 노드를 생성하여 `/scan` 토픽을 구독한다.

```cpp
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("sllidar_client");
    auto lidar_info_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
                          "scan", rclcpp::SensorDataQoS(), scanCb);
    rclcpp::spin(node);   // 무한 대기 → scan 수신 시 scanCb 호출
    rclcpp::shutdown();
    return 0;
}
```

---

## 좌표 변환

LiDAR 극좌표를 500×500 픽셀 화면 좌표로 변환하는 방식이다.

| 항목 | 값 |
|---|---|
| 화면 중심 | (250, 250) |
| 스케일 | 1m = 100px |
| 전방 방향 | 화면 위쪽 (py 감소 방향) |
| 변환식 | `px = 250 + r×100 × sin(θ)` |
|  | `py = 250 + r×100 × cos(θ)` |

---

## 저장 파일

| 파일명 | FPS | 해상도 | 내용 |
|---|---|---|---|
| `lidar.mp4` | 10fps | 500×500 | 흰 바탕 + 빨간 점 LiDAR 스캔 영상 |

---

## 토픽

| 방향 | 토픽명 | 타입 | 설명 |
|---|---|---|---|
| 구독 | `/scan` | `sensor_msgs/LaserScan` | LiDAR 스캔 데이터 |
