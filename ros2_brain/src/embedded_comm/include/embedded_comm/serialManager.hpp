#include <iostream>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/ctrl_cmd.hpp"
#include "interfaces/msg/imu.hpp"
#include "interfaces/msg/encoders.hpp"
#include "serial.hpp"

#define devPortName "/dev/ttyACM0"
#define baudRate B115200


using namespace std::chrono_literals;
using std::placeholders::_1;

enum Serial_cmd {
    move_speed,
    heading_ang,
    encoder_pub,
    imu_pub,
};
enum SensorDataType { 
    Encoders_t, 
    IMU_t
};
enum SensorDataLen{
    enc = 2, // encoders
    imu = 1,
};

class SerialManager : public rclcpp::Node{
private:
    Serial ser;
    rclcpp::Publisher<interfaces::msg::Encoders>::SharedPtr enc_pub_;
    rclcpp::Publisher<interfaces::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Subscription<interfaces::msg::CtrlCmd>::SharedPtr ctrl_cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int16_t linearVel_;
    int16_t headingAng_;
    bool encPub_status_;
    bool imuPub_status_;
    void timer_readSer();
    void ctrl_cmd_callback(const interfaces::msg::CtrlCmd::SharedPtr msg);
    void send_cmd(uint8_t index, int16_t value);
    bool processSerialReading(std::string &line, uint8_t &index, std::vector<double> &values);
    template <typename T>
    void checkAndSend(Serial_cmd cmd, T new_val, T &old_val){
        if(old_val != new_val){
            send_cmd(cmd, new_val);
            old_val = new_val;
        }
    }
public:
    SerialManager();
    ~SerialManager();
};

std::vector<double> tokenize(std::string &s, std::string del = ",");
int main(int argc, char * argv[]);