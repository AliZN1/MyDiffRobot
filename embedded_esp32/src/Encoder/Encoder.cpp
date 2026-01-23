#include "Encoder/Encoder.hpp"


Encoder::Encoder(uint8_t pin_num, bool reverseDir): 
    pin(pin_num), uperLimit(DAC_resolution), lowerLimit(0), omega(0.0), last_angle(0.0), numTurns(0)
{
    dir = reverseDir? -1 : 1;
};

Encoder::~Encoder(){};

// /**
//  * @brief This function runs the process the encoder is required to do.
//  */
// void Encoder::runTask(){
//     setLimits(0, 4095);
//     initLastAngle();

//     while(1){
//       float angPos = updateAngDisp();
//       if(xQueueSendToBack(encoder_q, &angPos, (TickType_t)0) != pdPASS)
//         Serial.println("Fail to update encoder data!");

//       vTaskDelay(pdMS_TO_TICKS(CONTROL_TICK));
//     }
// }

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
float Encoder::readAngle(){
    float read = analogRead(pin);
    
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
float Encoder::updateAngDisp(){
    float currentAngle = readAngle();
    
    float delta =  currentAngle - last_angle;
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
void Encoder::computeOmega(uint32_t &dt, float &cur_angPos){
    if(dt == 0)
        omega = 0;
    else
        omega = (cur_angPos - last_angPos) / static_cast<float>(dt) * 1e3;

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