#include "embedded_comm/serialManager.hpp"

SerialManager::SerialManager():Node("publish_serial"), ser(devPortName, baudRate){
    publisher_ = this->create_publisher<std_msgs::msg::String>("/serialRead", 1);
    subscription_ = this->create_subscription<interfaces::msg::CtrlCmd>("/control_cmd", 1, std::bind(&SerialManager::subscription_callback, this, _1));
    timer_ = create_wall_timer(500ms, std::bind(&SerialManager::timer_callback, this));
}

void SerialManager::timer_callback(){
    auto msg = std_msgs::msg::String();
    msg.data = "Hello, worlds! " + ser.readLine();
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
    publisher_->publish(msg);
}

void SerialManager::subscription_callback(const interfaces::msg::CtrlCmd::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->linear_vel);
}

void SerialManager::processSerialReading(){

}

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialManager>());
    rclcpp::shutdown();
    return 0;
}


// #include "utilities.hpp"
// #include <thread>


// int main(){
//     Serial ser("/dev/ttyACM0", 115200);
//     std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    
//     char cmd[] = "#00:5;\n";
//     ser.writeSerial(cmd, sizeof(cmd));

//     while(true){
//         if(ser.isAvailable(10)){
//             std::string res = ser.readLine();
//             std::cout << res;
//         }
//     }
// }