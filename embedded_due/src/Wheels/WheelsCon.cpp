#include "Wheels/WheelsCon.hpp"

// --------------------- Wheel interface class
Wheel::Wheel(const uint8_t pinA, const uint8_t pinB):
    pin_A(pinA), pin_B(pinB), pid(wheel_PID_Kp, wheel_PID_Ki, wheel_PID_Kd, wheel_PID_Kff, 0.01) {
        pid.set_saturation(motor_saturation_max, motor_saturation_min);
    }

Wheel::~Wheel(){}

/**
 * @brief Rotates the wheel clockwise at given speed.
 * 
 * @param[in] speed A PWM signal (between 0 to 255) sent to the motor driver.
 */
void Wheel::moveForward(uint8_t speed){
    analogWrite(pin_A, 0);
    analogWrite(pin_B, speed);
}

/**
 * @brief Rotates the wheel counterclockwise at given speed.
 * 
 * @param[in] speed A PWM signal (between 0 to 255) sent to the motor driver.
 */
void Wheel::moveBackward(uint8_t speed){
    analogWrite(pin_A, speed);
    analogWrite(pin_B, 0);
}

/**
 * @brief Rotates the wheel clockwise/counterclockwise at given speed.
 * 
 * @param[in] speed A PWM signal (between 0 to 255) sent to the motor driver.
 *      The sign of the speed determines the direction: positive for clockwise, 
 *      negative for counterclockwise.
 */
void Wheel::move(int16_t speed){
    if(speed < -254 || speed > 254)
        return;

    if(speed > 0)
        moveForward(speed);
    else if(speed < 0)
        moveBackward(abs(speed));
}

/**
 * @brief Stops the wheel from moving.
 * 
 * It resets the wheel associated PID speed controller.
 */
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

/**
 * @brief Performs the primary operations related to the wheels control.
 * 
 * Runs the speed controllers of the wheels and reads the feedback 
 * from IMU and encoders.
 */
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

/**
 * @brief Udates setpoint of wheels speed controllers.
 * 
 * Keeps both wheels at the same speed to move the robot linearly.
 * 
 * @param[in] speed Desired angular speed (rad/s) for wheels to rotate at. 
 *      The wheels can rotate at a speed between -15 to 15.
 */
void WheelsCon::set_linearVel(int8_t speed){
    if(speed < -15 || speed > 15) return;
    stopped = false;

    wheel_R.pid.update_setpoint(speed);
    wheel_L.pid.update_setpoint(speed);
}

/**
 * @brief Take care for the controller to move the robot linearly.
 * 
 * Runs the PID controllers and rotates the wheels at the corresponding 
 * speeds computed by them.
 * 
 * @param[in] angVel Reference to a 2-element array containing current 
 * angular velocity of each wheel.
 */
void WheelsCon::driveLinear(float (&angVel)[2]){
    double velCommand_R = wheel_R.pid.run(angVel[0]); // PID controller has saturation control
    double velCommand_L = wheel_L.pid.run(angVel[0]); // this value has to change to angVel[1]. This is just due to a broken encoder.
    
    wheel_R.move((int16_t)velCommand_R);
    wheel_L.move((int16_t)velCommand_L);
}

/**
 * @brief Udates setpoint of steering angle controller.
 * 
 * There is a PID controller that ensure robot rotates around its axis
 * correctly to move with the desired steering angle.
 * 
 * @param[in] angularPos Desired steering angle (in degrees) for the 
 *      robot to rotate toward. Range of accepted input is from -180 to 180.
 */
void WheelsCon::set_angularPos(int16_t angularPos){ //positive speed clockwise rotation and vice versa
    if(angularPos > 180 || angularPos < -180) return;
    R_stopped = false;

    headAng_setpoint = angularPos;
    i = 0;
    pid_rotation.update_setpoint(angularPos);
}

/**
 * @brief Take care for the controller to rotate the robot in place.
 * 
 * Runs the steering PID controller and rotates the wheels at the corresponding 
 * speeds (in reverse directions) using their corresponding controller, 
 * to achieve in-place rotation.
 * 
 * @param[in] cur_angVel current steering angle of the robot.
 */
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

/**
 * @brief Prevent both wheels from rotating.
 * 
 * Stops both rotation and transition controllers.
 */
void WheelsCon::stop(){
    stopped = true;
    R_stopped = true;
    wheel_R.stop();
    wheel_L.stop();
}