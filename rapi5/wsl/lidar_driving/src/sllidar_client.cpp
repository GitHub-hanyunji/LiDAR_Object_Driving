/*
 *  SLLIDAR ROS2 CLIENT
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2022 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */

#include <lidar_driving/lidar_driving.hpp>
#include <math.h>
#include <cmath>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>


#define RAD2DEG(x) ((x)*180./M_PI) // 라디안-> 도 변환 메크로
#define STDIN_FILENO 0

using namespace cv;


// LiDARDetectNode 클래스
// 생성자
// rclcpp::Node를 상속한 클래스
LiDARDetectNode::LiDARDetectNode() : Node("linedetect_wsl")
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));   // qos
    // callback 함수 bind
    auto fn=std::bind(&LiDARDetectNode::lidar_callback, this, std::placeholders::_1);
    sub = this->create_subscription<sensor_msgs::msg::LaserScan>("scan",qos,fn); // 구독
    pub = this->create_publisher<geometry_msgs::msg::Vector3>("topic_dxlpub",qos);  // 발행

    // 시각화 frame 크기 설정
    width = 500;
    height = 500;

    // 기준 중심점 설정
    p_center = Point(width / 2, height / 2);
    left_pt=Point(width/4,height/4);
    right_pt=Point(width*3/4,height/4);
    // 거리,각도 초기값
    left_dist  = 1e9;
    right_dist = 1e9;
    left_angle = 0.0;
    right_angle = 0.0;

    // 바퀴 값
    vel_msg.x = 0.0;
    vel_msg.y = 0.0;
    vel_msg.z = 0.0; // 필요 없으면 0

    RCLCPP_INFO(this->get_logger(), "Line Detect Node Started");
}

// 엔터 없이 키 하나 즉시 입력받는 함수
int LiDARDetectNode::getch(void)
{
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}
// 키보드 제어 함수
bool LiDARDetectNode::kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return true;
    }
    return false;
}

// 관심영역 설정 함수
void LiDARDetectNode::Set(Mat& frame){
    frame = frame(Rect(Point(0, 0), Point(frame.cols, frame.rows/2)));  // ROI 상단 1/4
    Mat mask;
    // 빨간색만 추출
    // 빨간색이 아니면 검은색, 빨간색은 흰색으로 mask
    inRange(
        frame,
        Scalar(0, 0, 150),   // 각 채널의 최소 허용값
        Scalar(80, 80, 255), // 각 채널의 최대 허용값
        mask
    );

    frame = mask;  // 흰색 = 빨간 객체, 검정 = 배경
    roi_center = Point(frame.cols / 2, frame.rows);

}

// 객체 추적 함수
void LiDARDetectNode::FindObject(Mat& frame, Mat& labels,Point& p_center, Mat& stats, Mat& centroids)
{
    // frame에서 붙어 있는 흰색 픽셀 묶음을 객체로 판단 -> cnt: 배경 포함 판단된 객체 개수
    // labels: 각 픽셀
    int cnt = connectedComponentsWithStats(frame, labels, stats, centroids);

    int mid_x = frame.cols / 2;  // 화면 중앙

    // 상태 변수 초기화
    left_ok = false;
    right_ok = false;
    double left_best = 1e9;
    double right_best = 1e9;

    for (int i = 1; i < cnt; i++) {

        Point closest_pt;
        double min_dist = 1e9;

        // 핵심: 마스크에서 가장 가까운 점 찾기
        for (int y = 0; y < labels.rows; y++) {
            for (int x = 0; x < labels.cols; x++) {
                if (labels.at<int>(y, x) == i) {
                    Point pt(x, y);
                    // 로봇 기준으로 가장 가까운 점 찾기
                    double d = norm(pt - p_center);
                    if (d < min_dist) {
                        min_dist = d;
                        closest_pt = pt;
                    }
                }
            }
        }

        // 너무 먼 객체 제외
        if (min_dist > 70) continue;

        // 로봇 기준 방향 계산
        double dx = closest_pt.x - p_center.x;
        double dy = closest_pt.y - p_center.y;
        double angle = atan2(dx, dy) * 180.0 / M_PI;

        // 좌 / 우 분리
        // 왼쪽에서 가장 가까운 객체 하나만 선택
        if (closest_pt.x < mid_x) {
            if (min_dist < left_best) {
                left_best = min_dist;
                left_ok = true;
                left_dist = min_dist;
                left_angle = angle;
                left_idx = i;
            }
        }
        // 오른쪽에서 가장 가까운 객체 하나만 선택
        else {
            if (min_dist < right_best) {
                right_best = min_dist;
                right_ok = true;
                right_dist = min_dist;
                right_angle = angle;
                right_idx = i;
            }
        }
    }
}


// 시각화 함수
void LiDARDetectNode::Draw(Mat& frame, Mat stats, Mat centroids,Mat& label,int labels, Point p_center)
{
    // 이진 이미지를 컬러 이미지로 변환
    if (frame.channels() == 1) {
        // Gray → BGR
        cvtColor(frame, frame, COLOR_GRAY2BGR);
    }

    // 화살표 끝점 변수 선언
    Point left_arrow_end;
    Point right_arrow_end;

    int size=120;  // 화살표 길이
    // 왼쪽 화살표
    if (left_ok) {
        // 객체가 있을 때 → 객체 방향
        left_arrow_end = Point(
            p_center.x + left_dist * sin(left_angle * M_PI / 180.0),
            p_center.y + left_dist * cos(left_angle * M_PI / 180.0)
        );
    } else {
        // 없을 때 → 왼쪽으로 쭉
        left_arrow_end = Point(
            p_center.x - size,
            p_center.y
        );
        
    }

    // 오른쪽 화살표
    if (right_ok) {
        // 객체가 있을 때 → 객체 방향
        right_arrow_end = Point(
            p_center.x + right_dist * sin(right_angle * M_PI / 180.0),
            p_center.y + right_dist * cos(right_angle * M_PI / 180.0)
        );
    } else {
        // 없을 때 → 오른쪽으로 쭉
        right_arrow_end = Point(
            p_center.x + size,
            p_center.y
        );
    }

    if (left_ok && right_ok) {
        // 양쪽 모두 장애물 ->+180은 화면 좌표계 보정
        mid_angle = ((left_angle + right_angle) / 2.0)+180;
    }
    else if (!left_ok && right_ok) {
        // 오른쪽만 있음 → 왼쪽은 비어있음
        double left_default = -90.0;   // 왼쪽 끝
        mid_angle = ((right_angle + left_default) / 2.0)+180;
    }
    else if (left_ok && !right_ok) {
        // 왼쪽만 있음 → 오른쪽은 비어있음
        double right_default = 90.0;   // 오른쪽 끝
        mid_angle = ((left_angle + right_default) / 2.0)+180;
    }
    else {
        // 아무것도 없음 → 직진
        mid_angle = 180.0;
    }

    // 각도정규화
    while (mid_angle < 0)   mid_angle += 360;
    while (mid_angle >= 360) mid_angle -= 360;

    // 중앙 화살표 끝점 계산
    Point mid_arrow_end(
        p_center.x +size * sin(mid_angle * M_PI / 180.0),
        p_center.y + size * cos(mid_angle * M_PI / 180.0)
    );

    // 화살표 그리기
    arrowedLine(frame,p_center,left_arrow_end,Scalar(0, 255, 0),2,LINE_AA,0,0.05);  //왼쪽
    arrowedLine(frame,p_center,right_arrow_end,Scalar(255, 0, 0),2,LINE_AA,0,0.05); // 오른쪽
    arrowedLine(frame, p_center, mid_arrow_end,Scalar(0, 0, 255), 2, LINE_AA, 0, 0.05);  // 중간
}

// call-back 함수
void LiDARDetectNode::lidar_callback(sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // 타임 스탬프
    // 프레임 처리 속도 측정하기 위한 시작 타임스탬프
    auto start = std::chrono::steady_clock::now();
    static Mat frame(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
    frame.setTo(Scalar(255,255,255));  // frame 초기화
    // lidar 한바퀴에서 측정된 포인트 개수
    int count = scan->scan_time / scan->time_increment;
    // printf("[SLLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
    // printf("[SLLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min),
    //        RAD2DEG(scan->angle_max));

    Point center(250,250);  // frame 중심점
    char ch;
    // lidar->2d 좌표 변환
    for (int i = 0; i < count; i++) 
    {
        // 각도 계산 -> 사람이 보기 쉬운 각도
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        //printf("[SLLIDAR INFO]: angle-distance : [%f, %f]\n", degree, scan->ranges[i]);

        float theta = degree * M_PI / 180.0;  // 삼각함수용 라디안
        float r_px = scan->ranges[i] * 100;   // 1 m = 50 px, 2m=100px
        
        // 극좌표 -> 화면 좌표
        int px = 250 + r_px * sin(theta);
        // y축은 아래로 갈수록 +
        int py = 250 + r_px * cos(theta);  // +rpx는 전방 아래쪽, -rpx는 전방 위쪽으로 되어있음

        // 점그리기
        circle(frame, Point(px, py), 2, Scalar(0,0,255), -1);

    }
    circle(frame, center, 2, Scalar(0,255,0), 3); // 센터 값
    if(video_key) writer.write(frame);  // 비디오 저장
      

    if(frame.empty()) return;

    cv::Mat roi = frame.clone();  
    Set(roi);  // roi 설정

    cv::Mat labels, stats, centroids;
    // 객체 검출
    FindObject(roi,labels,roi_center,stats,centroids);
    // 시각화
    Draw(roi, stats, centroids, labels,stats.rows, roi_center);  // 관심영역 시각화
    Draw(frame, stats, centroids, labels,stats.rows, p_center);  // frame 시각화


    // error 계산
    int error = mid_angle-180;
    
    //프레임 처리 시간을 초 단위로 변환
    auto end = std::chrono::steady_clock::now();
    float t = std::chrono::duration<float,std::milli>(end - start).count();

    // 속도 계산
    float leftvel = 50 - k* error;
    float rightvel = -(50 + k* error);
    
    if(kbhit()) // 없으면 제어 멈춤
    {
      ch = getch();
      if(ch == 's'){
        int fourcc = cv::VideoWriter::fourcc('m','p','4','v');
        double fps = 10.0;  // 원하는 FPS로 설정해도 됨
        writer.open("sllidar.mp4", fourcc, fps,
                    cv::Size(frame.cols, frame.rows), true);
        if (!writer.isOpened()) {
            printf("VideoWriter 열기 실패\n");
        } else {
            printf("VideoWriter 초기화 완료! 영상을 저장합니다.\n");
            video_key=true;
        }
        
        mode=true;
      }
      else if(ch=='r'){
        std::cout<<"라이다의 범위를 입력하세요(화면창 500x500 기준 좌우 5m(10m)):";
        std::cin>>range;
      }
      else if(ch=='q'){
        video_key=false;
        writer.release();
        mode=false;
      }
    }
    if(mode){
        vel_msg.x = leftvel;
        vel_msg.y = rightvel;
        vel_msg.z = 0.0; // 필요 없으면 0
    }
    else if(!mode){
        vel_msg.x = 0.0;
        vel_msg.y = 0.0;
        vel_msg.z = 0.0; // 필요 없으면 0
    }
    pub->publish(vel_msg);
    RCLCPP_INFO(this->get_logger(), "err:%d lvel:%f rvel:%f time:%f", error,vel_msg.x,vel_msg.y, t);
    cv::imshow("frame", frame);  // 원본
    cv::imshow("Track", roi);    // ROI
    cv::waitKey(1);
}

