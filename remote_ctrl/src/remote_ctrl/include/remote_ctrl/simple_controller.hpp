#include <iostream>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/ctrl_cmd.hpp"

#define const_speed 8

using namespace std::chrono_literals;

class SimpleController: public rclcpp::Node{
private:
    rclcpp::Publisher<interfaces::msg::CtrlCmd>::SharedPtr ctrl_cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    termios oldt;
    interfaces::msg::CtrlCmd msg;
public:
    SimpleController();
    ~SimpleController();
    void configT(); 
    void resetT();
    char waitForKey();
    void reset_msg();
    void timer_callback();
};

int main(int argc, char * argv[]);