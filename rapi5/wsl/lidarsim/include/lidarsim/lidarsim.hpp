#ifndef LIDAR_DETECT_NODE_HPP
#define LIDAR_DETECT_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/compressed_image.hpp"   // LaserScan -> CompressedImage
#include <geometry_msgs/msg/vector3.hpp>
#include <opencv2/opencv.hpp>

class LiDARDetectNode : public rclcpp::Node
{
public:
    LiDARDetectNode();

private:
    // LaserScan -> CompressedImage
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub;

    int width, height;
    cv::Point p_center, roi_center;
    cv::Point left_pt, right_pt;

    cv::VideoWriter writer;
    bool video_key = false;

    bool left_ok  = false;
    bool right_ok = false;
    int range = 0;
    double left_dist, right_dist;
    double left_angle, right_angle;
    int left_idx = -1, right_idx = -1;
    geometry_msgs::msg::Vector3 vel_msg;
    bool mode = false;
    double k = 0.5;
    double mid_angle = 180.0;

    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

    int getch();
    bool kbhit();
    void Set(cv::Mat& frame);
    void FindObject(cv::Mat& frame, cv::Mat& labels, cv::Point& p_center, cv::Mat& stats, cv::Mat& centroids);
    void Draw(cv::Mat& frame, cv::Mat stats, cv::Mat centroids, cv::Mat& label, int labels, cv::Point p_center);
};

#endif