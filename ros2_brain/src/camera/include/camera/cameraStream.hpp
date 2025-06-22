#include <iostream>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>


using namespace std::chrono_literals;

class CameraStream: public rclcpp::Node{
private:
    cv::VideoCapture cap_;
    image_transport::Publisher camPub_;
    rclcpp::TimerBase::SharedPtr camPub_timer_;
    void camPub_callback();
public:
    CameraStream();
    ~CameraStream();
};

int main(int argc, char * argv[]);