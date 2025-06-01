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
WheelsCon::WheelsCon(uint8_t (&motorDriverPins)[4], EncodersManager &encodersManager, uint16_t num_tick):
    Task(num_tick), wheel_R(motorDriverPins[0], motorDriverPins[1]), wheel_L(motorDriverPins[2], motorDriverPins[3]), encodersM(encodersManager) {
        stopped = true;
    }

WheelsCon::~WheelsCon(){}

void WheelsCon::execute(){
    if(stopped) return;

    float angVel[2];
    encodersM.getAngVel(angVel);
    // PID controller has saturation control
    double velCommand_R = wheel_R.pid.run(angVel[0]);
    double velCommand_L = wheel_L.pid.run(angVel[1]);
    
    wheel_R.move((int16_t)velCommand_R);
    wheel_L.move((int16_t)velCommand_L);

    // Serial.print("speed: ");
    // Serial.print(angVel[0]);
    // Serial.print(", cmd: ");
    // Serial.println(velCommand_R);
}

void WheelsCon::move_cmd(int8_t speed){
    stopped = false;
    if(speed < -15 || speed > 15) return;

    wheel_R.pid.update_setpoint(speed);
    wheel_L.pid.update_setpoint(speed);
    Serial.println("Setpoints updated");
}

void WheelsCon::rotateCW(uint8_t speed){
    wheel_R.moveBackward(speed);
    wheel_L.moveForward(speed);
}

void WheelsCon::rotateCCW(uint8_t speed){
    wheel_R.moveForward(speed);
    wheel_L.moveBackward(speed);
}

void WheelsCon::stop(){
    stopped = true;
    wheel_R.stop();
    wheel_L.stop();
}