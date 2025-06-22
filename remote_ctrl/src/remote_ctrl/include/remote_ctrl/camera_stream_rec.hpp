#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CameraStreamRec: public rclcpp::Node{
private:
    image_transport::Subscriber camSub_;
    void camSub_callback_(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
public:
    CameraStreamRec();
    ~CameraStreamRec();
};

int main(int argc, char* argv[]);