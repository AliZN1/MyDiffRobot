#include "remote_ctrl/camera_stream_rec.hpp"


CameraStreamRec::CameraStreamRec(): Node("camera_stream_rec"){
    camSub_ = image_transport::create_subscription(this, "camera/rawImage", std::bind(&CameraStreamRec::camSub_callback_, this, std::placeholders::_1), "raw");
}

CameraStreamRec::~CameraStreamRec(){}

void CameraStreamRec::camSub_callback_(const sensor_msgs::msg::Image::ConstSharedPtr &msg){
    try{
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::imshow("Received Image", frame);
        cv::waitKey(1);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "CV_bridge exception: %s", e.what());
    }
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraStreamRec>());
    rclcpp::shutdown();
    return 0;
}