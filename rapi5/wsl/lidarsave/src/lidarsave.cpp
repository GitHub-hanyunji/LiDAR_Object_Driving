/*
 *  SLLIDAR ROS2 CLIENT
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2022 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <opencv2/opencv.hpp>
#include <string>   
#include <math.h>


#define RAD2DEG(x) ((x)*180./M_PI)

using namespace std::placeholders;


void camera_save(const cv::Mat& frame,double fps,const std::string name,cv::VideoWriter& writer,bool& writer_initialized)
{
  
  if (!writer_initialized)
  {
      int fourcc = cv::VideoWriter::fourcc('m','p','4','v');
      // double fps = 30.0;  // 원하는 FPS로 설정해도 됨
      writer.open(name, fourcc, fps,cv::Size(frame.cols, frame.rows), true);
      if (!writer.isOpened()) {
          printf("VideoWriter 열기 실패");
      } else {
          writer_initialized = true;
          printf("VideoWriter 초기화 완료");
      }
  }
  // ----- 프레임 저장 -----
  if (writer_initialized)
      writer.write(frame);
}

static void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan) 
{
    static cv::Mat frame(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
    static cv::VideoWriter lidar_writer;
    static bool lidar_writer_initialized = false;
    frame.setTo(cv::Scalar(255,255,255));  // frame 초기화
    // int count = scan->scan_time / scan->time_increment;
    int count = scan->ranges.size();
    printf("[SLLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
    printf("[SLLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min),RAD2DEG(scan->angle_max));

    for (int i = 0; i < count; i++) 
    {
      float r = scan->ranges[i];
      if (!std::isfinite(r)) continue;
      float theta = scan->angle_min + scan->angle_increment * i;
        // degree 계산 (출력용)
      float degree = RAD2DEG(theta);
      float r_px = r* 100;   // 1 m = 50 px, 2m=100px
      // 극좌표 -> 화면 좌표
      int px = 250 + r_px * sin(theta);
      // y축은 아래로 갈수록 +
      int py = 250 + r_px * cos(theta);  // +rpx는 전방 아래쪽, -rpx는 전방 위쪽으로 되어있음
      // 점그리기
      cv::circle(frame, cv::Point(px, py), 2, cv::Scalar(0,0,255), -1);
      printf("[SLLIDAR INFO]: angle-distance : [%f, %f]\n", degree, r);
    }
    cv::imshow("frame", frame);
    cv::waitKey(1);
    camera_save(frame,10,"lidar.mp4",lidar_writer,lidar_writer_initialized);

}


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("sllidar_client");

  auto lidar_info_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
                        "scan", rclcpp::SensorDataQoS(), scanCb);
  
  rclcpp::spin(node);

  rclcpp::shutdown();


  return 0;
}
