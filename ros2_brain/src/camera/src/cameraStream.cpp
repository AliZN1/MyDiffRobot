#include "camera/cameraStream.hpp"

CameraStream::CameraStream(): Node("camera_stream"), cap_(0, cv::CAP_V4L2) { // '0' is default webcam
    if(!cap_.isOpened()){
        RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
        rclcpp::shutdown();
    }
    camPub_ = image_transport::create_publisher(this, "camera/rawImage");

    camPub_timer_ = this->create_wall_timer(33ms, std::bind(&CameraStream::camPub_callback, this));
}
CameraStream::~CameraStream(){}

void CameraStream::camPub_callback(){
    cv::Mat frame;
    cap_ >> frame;
    if(frame.empty()) return;

    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = "camera_frame";

    sensor_msgs::msg::Image::SharedPtr msg =
    cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();

    camPub_.publish(*msg);
}

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraStream>());
    rclcpp::shutdown();
    return 0;
}