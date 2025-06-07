#include <chrono>
#include <memory>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "serial.hpp"
#include "interfaces/msg/ctrl_cmd.hpp"

#define devPortName "/dev/ttyACM0"
#define baudRate 115200


using namespace std::chrono_literals;
using std::placeholders::_1;

class SerialManager : public rclcpp::Node{
private:
    Serial ser;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<interfaces::msg::CtrlCmd>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback();
    void subscription_callback(const interfaces::msg::CtrlCmd::SharedPtr msg);
    void processSerialReading();
public:
    SerialManager();
};

int main(int argc, char * argv[]);