#include "Wheels/WheelsCon.hpp"

// --------------------- Wheel interface class
Wheel::Wheel(const uint8_t pinA, const uint8_t pinB):
    pin_A(pinA), pin_B(pinB), pid(wheel_PID_Kp, wheel_PID_Ki, wheel_PID_Kd, wheel_PID_Kff, 0.01) {
        pid.set_saturation(motor_saturation_max, motor_saturation_min);
    }

Wheel::~Wheel(){}

void Wheel::moveForward(uint8_t speed){
    analogWrite(pin_A, 0);
    analogWrite(pin_B, speed);
}

void Wheel::moveBackward(uint8_t speed){
    analogWrite(pin_A, speed);
    analogWrite(pin_B, 0);
}

void Wheel::move(int16_t speed){
    if(speed < -254 || speed > 254)
        return;

    if(speed > 0)
        moveForward(speed);
    else if(speed < 0)
        moveBackward(abs(speed));
}

void Wheel::stop(){
    // pid.update_setpoint(0);
    pid.reset();
    analogWrite(pin_A, 0);
    analogWrite(pin_B, 0);
}

// -------------------- wheels controller class
WheelsCon::WheelsCon(uint8_t (&motorDriverPins)[4], EncodersManager &encodersManager, IMU &imu_ref, uint16_t num_tick):
    Task(num_tick), wheel_R(motorDriverPins[0], motorDriverPins[1]), wheel_L(motorDriverPins[2], motorDriverPins[3]), encodersM(encodersManager), imu(imu_ref), pid_rotation(PID_R_kp, PID_R_ki, PID_R_kd) {
        stopped = true;
        R_stopped = true;
        pid_rotation.set_saturation(7, -7);
    }

WheelsCon::~WheelsCon(){}

void WheelsCon::execute(){
    if(!stopped){
        float angVel[2];
        encodersM.getAngVel(angVel);
        driveLinear(angVel);
    }
    if(!R_stopped){
        float headingAng = imu.get_gyroYaw();
        driveAngular(headingAng);
    }
}

void WheelsCon::set_linearVel(int8_t speed){
    if(speed < -15 || speed > 15) return;
    stopped = false;

    wheel_R.pid.update_setpoint(speed);
    wheel_L.pid.update_setpoint(speed);
}

void WheelsCon::driveLinear(float (&angVel)[2]){
    double velCommand_R = wheel_R.pid.run(angVel[0]); // PID controller has saturation control
    double velCommand_L = wheel_L.pid.run(angVel[1]); 
    wheel_R.move((int16_t)velCommand_R);
    wheel_L.move((int16_t)velCommand_L);
}

void WheelsCon::set_angularPos(uint16_t angularPos){ //positive speed clockwise rotation and vice versa
    if(angularPos > 360) return;
    R_stopped = false;

    headAng_setpoint = angularPos;
    i = 0;
    pid_rotation.update_setpoint(angularPos);
}

void WheelsCon::driveAngular(float cur_angle){
    if(abs(headAng_setpoint - cur_angle) < 3){
        if(i > 5) stop();
        else i++;
        return;
    }

    // Serial.print("angle: ");
    // Serial.print(cur_angle);
    double angVel = pid_rotation.run(cur_angle);
    if(abs(angVel) < 2) angVel > 0 ? angVel = 2 : angVel = -2;

    // Serial.print("speed: ");
    // Serial.println(angVel);

    stopped = false;
    wheel_L.pid.update_setpoint(angVel);
    wheel_R.pid.update_setpoint(-angVel);
}
 
void WheelsCon::stop(){
    stopped = true;
    R_stopped = true;
    wheel_R.stop();
    wheel_L.stop();
}