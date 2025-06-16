#include "remote_ctrl/simple_controller.hpp"

SimpleController::SimpleController():Node("remote_controller"){
    ctrl_cmd_pub_ = this->create_publisher<interfaces::msg::CtrlCmd>("/control_cmd", 1);
    timer_ = this->create_wall_timer(100ms, std::bind(&SimpleController::timer_callback, this));
    reset_msg();
    configT();
}

SimpleController::~SimpleController(){
    reset_msg();
    ctrl_cmd_pub_->publish(msg);
    resetT();
 }

void SimpleController::configT(){
    termios newt;

    tcgetattr(STDIN_FILENO, &oldt); // get the standard config of the terminal
    newt = oldt; // make copy of the config

    newt.c_lflag &= ~(ICANON | ECHO); // Disable canonical mode and echo
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); // Set new terminal attributes immediately
}

void SimpleController::resetT(){
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // Restore original terminal settings
}

char SimpleController::waitForKey(){
    char ch;
    read(STDIN_FILENO, &ch, 1);
    return ch;
}

void SimpleController::timer_callback(){
    if(!rclcpp::ok())
        return;
        
    char res = waitForKey();

    switch(res){
        case 'w':{
            if(msg.linear_vel == 0)
                msg.linear_vel = const_speed;
            else if(msg.linear_vel < 0)
                msg.linear_vel = 0;
            break;
        }
        case 's':{
            if(msg.linear_vel == 0)
                msg.linear_vel = -const_speed;
            else if(msg.linear_vel > 0)
                msg.linear_vel = 0;
            break;
        }
        case 'd':{
            if(msg.yaw + 90 > 180)
                msg.yaw = 180;
            else
                msg.yaw += 90;
            break;
        }
        case 'a':{
            if(msg.yaw - 90 < -180)
                msg.yaw = -180;
            else
                msg.yaw -= 90;
            break;
        }
        case 'e':{
            if(msg.yaw + 45 > 180)
                msg.yaw = 180;
            else
                msg.yaw += 45;
            break;
        }
        case 'q':{
            if(msg.yaw - 45 < -180)
                msg.yaw = -180;
            else
                msg.yaw -= 45;
            break;
        }
        case 'n':
            msg.enc_pub = !msg.enc_pub;
            break;
        case 'm':
            msg.imu_pub = !msg.imu_pub;
            break;
        default:
            break;
    }

    ctrl_cmd_pub_->publish(msg);
}

void SimpleController::reset_msg(){
    msg.linear_vel = 0;
    msg.yaw = 0;
    msg.enc_pub = false;
    msg.imu_pub = false;
}

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleController>());
    rclcpp::shutdown();
    return 0;
}
