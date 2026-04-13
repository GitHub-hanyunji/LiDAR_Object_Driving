#ifndef LIDAR_DETECT_NODE_HPP
#define LIDAR_DETECT_NODE_HPP

#include <rclcpp/rclcpp.hpp>                     // Node 상속
#include "sensor_msgs/msg/laser_scan.hpp"   
#include <geometry_msgs/msg/vector3.hpp>
#include <opencv2/opencv.hpp>   


// rclcpp::Node를 상속하는 LiDARDetectNode 클래스
class LiDARDetectNode : public rclcpp::Node
{
public:
    LiDARDetectNode();  // 생성자

private:
    // 토픽 구독 Subscriber
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub;
    // 토픽 발행 Publisher -> 바퀴 속도 값 전달
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub;

    int width, height; // opencv frame 크기
    // p_center -> 로봇 중심점, roi_center-> 관심영역에서의 로봇 중심점
    cv::Point p_center,roi_center;  
    // left/right 의 탐지된 객체 중심점 
    cv::Point left_pt, right_pt; 

    // 비디오 저장
    cv::VideoWriter writer;
    bool video_key;

    // 객체 탐지 상태 관련 변수
    bool left_ok  = false;
    bool right_ok = false;
    int range;  // 라이다 탐지 범위
    double left_dist, right_dist;  // 좌/우 객체까지의 실제 거리값
    double left_angle, right_angle; // 좌/우 객체의 각도
    int left_idx = -1, right_idx = -1;
    geometry_msgs::msg::Vector3 vel_msg; // publish 할 메시지
    bool mode =false;  // 바퀴 모드 전환용 플래그
    double k = 0.5;
    double mid_angle;  // 중앙 각도

    // callback
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    // keyboard
    int getch();
    bool kbhit();
    void Set(cv::Mat& frame);   //roi 설정
    void FindObject(cv::Mat& frame, cv::Mat& labels,cv::Point& p_center, cv::Mat& stats, cv::Mat& centroids); // 객체추적함수
    void Draw(cv::Mat& frame,cv::Mat stats,cv::Mat centroids,cv::Mat& label,int labels,cv::Point p_center); // 시각화함수
};

#endif // LIDAR_DETECT_NODE_HPP