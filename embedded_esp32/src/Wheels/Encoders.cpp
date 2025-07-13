#include "Wheels/Encoders.hpp"

// ------------------------------------ Encoder
Encoder::Encoder(uint8_t pin_num, bool reverseDir): 
    pin(pin_num), uperLimit(1023), lowerLimit(0), omega(0.0), last_angle(0.0), numTurns(0)
{
    dir = reverseDir? -1 : 1;
};

Encoder::~Encoder(){};

/**
 * @brief Sets a limit on the expected analog input range from the encoder.
 * 
 * This helps achieve better resolution and allows the encoder angle
 * to approach values close to 0 and 2π more accurately.
 * 
 * @param[in] min Maximum expected analog signal from encoder.
 * @param[in] max Minimum expected analog signal from encoder.
 */
void Encoder::setLimits(uint16_t min, uint16_t max){
    lowerLimit = min;
    uperLimit = max;
}

/**
 * @brief Updates the last angle using the encoder initial angle.
 * 
 * Initializing the encoder angular position and updates the last angle 
 * position. These values are used to find the absolute displacement.
 */
void Encoder::initLastAngle(){
    initAngle = readAngle();
    last_angle = initAngle;
    last_angPos = 0;
}

/**
 * @brief Reads the corresponding analog input and returns the angle.
 * 
 * The analog input is processed based on the upper and lower limits, 
 * then converted to radians.
 * 
 * @return Encoder's current position [0, 2π].
 */
double Encoder::readAngle(){
    double read = analogRead(pin);
    if(read > uperLimit)
        read = uperLimit;
    else if(read < lowerLimit)
        read = lowerLimit;
    
    return (read - lowerLimit)/(uperLimit - lowerLimit) * 2 * pi;
}

/**
 * @brief Computes encoder absolute angular position.
 * 
 * Computes the angular difference each time it's called. Based on the 
 * difference, it detects full rotations and keeps track of the 
 * encoder's absolute position.
 * 
 * @return Encoder's current angular position relative to its 
 *      initial position.
 */
double Encoder::updateAngDisp(){
    double currentAngle = readAngle();
    double delta =  currentAngle - last_angle;
    last_angle = currentAngle;
    // update angular displacement
    if(delta > pi){
        numTurns--;
    }
    else if (delta < -pi){
        numTurns++;
    }
    return (numTurns * 2*pi + currentAngle - initAngle) * dir;
}

/**
 * @brief Computes angular velocity that encoder is rotating with.
 * 
 * Differentiate the angular position to compute angular velocity.
 * It updates the relative class members.
 * 
 * @param[in] dt Delta t between two time intervals of capturing angular position.
 * @param[in] cur_angPos Encoder's current angular position.
 */
void Encoder::computeOmega(uint32_t &dt, double &cur_angPos){
    if(dt == 0)
        omega = 0;
    else
        omega = (cur_angPos - last_angPos) / static_cast<double>(dt) * 1e3;

    last_angPos = cur_angPos;
}

/**
 * @brief Getter function for encoder's current angular velocity.
 * 
 * @return Encoder's current angular velocity.
 */
float Encoder::getOmega(){
    return omega;
}

// ------------------------------------ EncoderManager
EncodersManager::EncodersManager(uint8_t pin_right, uint8_t pin_left, SerialPublisher &sp, uint16_t num_ticks)
    : Task(num_ticks), enc_R(pin_right), enc_L(pin_left, true), serialPublisher(sp), publish(false) {}

EncodersManager::~EncodersManager(){}

/**
 * @brief Performs the primary operations related to the encoders.
 * 
 * Updates the angular position and velocity of the  encoders. 
 * Publishes the results if activated.
 */
void EncodersManager::execute(){
    double cur_angPosArr[2]; // current ang position for wheels right an left respectively
    angularPos(cur_angPosArr);
    pub_encoders(cur_angPosArr);
    angularVel(cur_angPosArr);
}

/**
 * @brief Computes both encoders current angular position.
 * 
 * @param[out] angPosArr A 2-element array to store the current 
 * angular positions of the encoders.
 */
void EncodersManager::angularPos(double (&angPosArr)[2]){
    angPosArr[0] = enc_R.updateAngDisp();
    angPosArr[1] = enc_L.updateAngDisp();
}

/**
 * @brief Computes both encoders' current angular velocity.
 * 
 * To avoid division by zero, it returns `0` for both encoders' 
 * angular velocity if time stamps have less than 1 millisecond
 * difference.
 * 
 * @param[out] cru_angPosArr A 2-element array to store the 
 *      current angular position of the encoders.
 */
void EncodersManager::angularVel(double (&cur_angPosArr)[2]){
    uint32_t now = millis();
    uint32_t dt = now - last_omegaStamp;

    if(dt < omega_min_dt){
        cur_angPosArr[0] = 0;
        cur_angPosArr[1] = 0;
        return;
    }

    enc_R.computeOmega(dt, cur_angPosArr[0]);
    enc_L.computeOmega(dt, cur_angPosArr[1]);
    last_omegaStamp = now;
}

/**
 * @brief Stores the inital angle of both encoders.
 * 
 * Time stamp for angular velocity is initialized as well.
 */
void EncodersManager::initLastAngles(){
    enc_R.initLastAngle();
    enc_L.initLastAngle();

    last_omegaStamp = micros();
}

/**
 * @brief To access encoders' angular velocity
 * 
 * @param[out] angVel A 2-element array to store the current 
 *      angular velocity of the encoders.
 */
void EncodersManager::getAngVel(float (&angVel)[2]){
    angVel[0] = enc_R.getOmega();
    angVel[1] = enc_L.getOmega();
}

/**
 * @brief Publishes encoders' absolute angular position.
 * 
 * Publishes the encoder positions if the publisher is enabled 
 * (disabled by default).
 * 
 * @param[in] cur_angPosArr A 2-element array to store the current 
 *      angular position of the encoders.
 */
void EncodersManager::pub_encoders(double (&cur_angPosArr)[2]){
    if(!publish) return;

    char result_str[maxNumChar];
    sprintf(result_str, "@%02d:%.4f,%.4f;\n", Encoders_t, cur_angPosArr[0], cur_angPosArr[1]);
    serialPublisher.push_msg(result_str);
}

/**
 * @brief Enables or disables the encoder position publisher, which 
 * sends data over serial.
 * 
 * @param[in] state Use `true` to enable and `false` to disable the publisher.
 */
void EncodersManager::set_publish(bool state){
    publish = state;
}
