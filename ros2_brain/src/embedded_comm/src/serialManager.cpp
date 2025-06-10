#include "embedded_comm/serialManager.hpp"


SerialManager::SerialManager():Node("publish_serial"), ser(devPortName, baudRate){
    enc_pub_ = this->create_publisher<interfaces::msg::Encoders>("/emb/enc", 1);
    imu_pub_ = this->create_publisher<interfaces::msg::Imu>("/emb/imu", 1);
    ctrl_cmd_sub_ = this->create_subscription<interfaces::msg::CtrlCmd>("/control_cmd", 1, std::bind(&SerialManager::ctrl_cmd_callback, this, _1));
    timer_ = create_wall_timer(50ms, std::bind(&SerialManager::timer_readSer, this));
    linearVel_ = 0;
    headingAng_ = 0;
    encPub_status_ = false;
    imuPub_status_ = false;
}

SerialManager::~SerialManager(){ }

void SerialManager::timer_readSer(){
    if(!ser.isAvailable()) return;

    std::string msg = ser.readLine();
    uint8_t index;
    std::vector<double> values;

    bool status = processSerialReading(msg, index, values);
    if(!status) return;
    
    switch(index){
        case SensorDataType::Encoders_t: {
            if(values.size() != SensorDataLen::enc) break;
            interfaces::msg::Encoders msg;
            msg.right = values[0];
            msg.left = values[1];
            enc_pub_->publish(msg);
            break;
        }
        case SensorDataType::IMU_t: {
            if(values.size() != SensorDataLen::imu) break;
            interfaces::msg::Imu msg;
            msg.yaw = values[0];
            imu_pub_->publish(msg);
            break;
        }
        default:
            break;
    }
}

void SerialManager::ctrl_cmd_callback(const interfaces::msg::CtrlCmd::SharedPtr msg){
    checkAndSend(Serial_cmd::move_speed, msg->linear_vel, linearVel_);
    checkAndSend(Serial_cmd::heading_ang, msg->yaw, headingAng_);
    checkAndSend(Serial_cmd::encoder_pub, msg->enc_pub, encPub_status_);
    checkAndSend(Serial_cmd::imu_pub, msg->imu_pub, imuPub_status_);
}

void SerialManager::send_cmd(uint8_t index, int16_t value){
    char cmd[SERIAL_BUFFER_SIZE];
    sprintf(cmd, "#%02d:%d;\n", index, value);
    ser.writeSerial(cmd, strlen(cmd));
}

bool SerialManager::processSerialReading(std::string &line, uint8_t &index, std::vector<double> &values){
    size_t size = line.size();
    if(size == 0 || line.at(0) != '@' || line.at(size - 2) != ';'){
        std::cout << line;
        return 0;
    } 

    std::string index_str = line.substr(1, 2);
    std::string values_str = line.substr(4, size-6); //10.01,20.01,0.02
    try{
        index = stoi(index_str);
        values = tokenize(values_str);
    }
    catch(const std::invalid_argument &e){
        std::cerr <<"erro in data conversion, " << e.what() << '\n';
        return 0;
    }

    return 1;
}

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialManager>());
    rclcpp::shutdown();
    return 0;
}

std::vector<double> tokenize(std::string &s, std::string del){
    int del_size = del.size();
    std::vector<double> tokens;

    int start, end = -1*del_size;
    do {
        start = end + del_size;
        end = s.find(del, start);
        tokens.push_back(stod(s.substr(start, end - start)));
    } while(end != -1);
    
    return tokens;
}